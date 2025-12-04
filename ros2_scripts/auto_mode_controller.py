#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from irobot_create_msgs.msg import DockStatus
import subprocess
import os
import signal
import psutil

class AutoModeController(Node):
    def __init__(self):
        super().__init__('mode_auto_node')  # Garder le nom du nœud cohérent avec les logs JS
        
        # Maintenir le nom du service cohérent avec le code JS
        self.srv = self.create_service(SetBool, '/mode_auto', self.handle_auto_mode)
        
        # S'abonner au topic de statut du dock
        self.dock_subscription = self.create_subscription(
            DockStatus,
            '/dock_status',
            self.dock_status_callback,
            10
        )
        
        # Initialiser les variables d'état
        self.auto_mode_active = False
        self.was_undocked = False
        self.is_docked = False
        self.processes = []
        
        # Configurer les chemins des scripts (cohérent avec la structure de fichiers JS)
        self.home_dir = os.path.expanduser("~")
        self.scripts = [
            os.path.join(self.home_dir, 'detector.py'),
            os.path.join(self.home_dir, 'follow_waypoint.py')
        ]
        
        # S'assurer que les scripts sont exécutables
        for script in self.scripts:
            os.chmod(script, 0o755)

    def dock_status_callback(self, msg):
        """Callback pour surveiller l'état du dock"""
        # Stocker l'état précédent
        previous_docked = self.is_docked
        
        # Mettre à jour l'état actuel
        self.is_docked = msg.is_docked
        
        # Si le mode auto est actif et que le robot s'est détaché du dock
        if self.auto_mode_active and not self.is_docked and previous_docked:
            self.was_undocked = True
            self.get_logger().info("Robot détaché du dock pendant le mode auto")
        
        # Si le mode auto est actif, que le robot était détaché et qu'il revient au dock
        if self.auto_mode_active and self.is_docked and not previous_docked and self.was_undocked:
            self.get_logger().info("Robot revenu au dock, fin du cycle auto")
            self.kill_all_processes()
            self.auto_mode_active = False
            self.was_undocked = False

    def handle_auto_mode(self, request, response):
        """Maintenir une structure de réponse de service compatible avec le code JS"""
        try:
            if request.data:
                if self.auto_mode_active:
                    response.success = False
                    response.message = "Auto mode is already active"
                    self.get_logger().warn("⚠️ Mode automatique déjà activé, demande de démarrage ignorée")
                    return response
                
                self.start_processes()
                self.auto_mode_active = True
                self.was_undocked = False
                response.success = True
                response.message = "Auto mode activated"
                self.get_logger().info("🚀 Mode automatique démarré avec succès")
            else:
                self.kill_all_processes()
                self.auto_mode_active = False
                self.was_undocked = False
                response.success = True
                response.message = "Auto mode deactivated"
                self.get_logger().info("🛑 Mode automatique arrêté")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"❌ Échec de l'appel du service : {str(e)}")
            self.auto_mode_active = False
            self.was_undocked = False
            
        return response

    def start_processes(self):
        """Démarrer les processus en gardant le format de log cohérent avec JS"""
        self.get_logger().info("Démarrage du mode autonome...")
        for script in self.scripts:
            try:
                # Utiliser un groupe de processus pour une terminaison complète
                proc = subprocess.Popen(
                    ['python3', script],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid
                )
                self.processes.append(proc)
                self.get_logger().debug(f"✅ Processus démarré {os.path.basename(script)} (PID: {proc.pid})")
            except Exception as e:
                self.get_logger().error(f"🔥 Échec du démarrage du processus : {str(e)}")
                self.kill_all_processes()
                raise

    def kill_all_processes(self):
        """Forcer l'arrêt de tous les processus enfants (compatible avec le traitement des délais JS)"""
        self.get_logger().info("Arrêt du mode autonome...")
        for proc in self.processes:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    self.get_logger().debug(f"⛔ Groupe de processus terminé (PGID: {os.getpgid(proc.pid)})")
                except ProcessLookupError:
                    pass
        self.processes.clear()

    def __del__(self):
        """Nettoyer lors de la destruction"""
        self.kill_all_processes()

def main(args=None):
    rclpy.init(args=args)
    controller = AutoModeController()
    
    try:
        controller.get_logger().info("🟢 Contrôleur de mode auto prêt")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Arrêt demandé...")
    finally:
        controller.kill_all_processes()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
