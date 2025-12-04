#!/usr/bin/env python3

# mettre dans ros2 ~
# lacer pour dock/undock via rosbridge

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from irobot_create_msgs.action import Dock, Undock
import threading

class DockBridgeNode(Node):
    def __init__(self):
        super().__init__('dock_bridge_node')
        
        # Créer des clients d'action
        self._dock_client = ActionClient(self, Dock, '/dock')
        self._undock_client = ActionClient(self, Undock, '/undock')
        
        # Créer des services
        self._dock_service = self.create_service(
            Trigger, '/trigger_dock', self.handle_dock)
        self._undock_service = self.create_service(
            Trigger, '/trigger_undock', self.handle_undock)
        
        # Verrou pour suivre l'état des actions
        self._lock = threading.Lock()
        self._dock_in_progress = False
        self._undock_in_progress = False
        
        self.get_logger().info('Dock bridge node started')
    
    def handle_dock(self, request, response):
        """Traiter la demande de service dock"""
        self.get_logger().info('Received dock request')
        
        with self._lock:
            # Vérifier si une action est en cours
            if self._dock_in_progress:
                response.success = False
                response.message = 'Dock action already in progress'
                return response
            
            if self._undock_in_progress:
                response.success = False
                response.message = 'Undock action in progress, please wait'
                return response
            
            # Marquer le début de l'action dock
            self._dock_in_progress = True
        
        # Démarrer un thread pour traiter l'appel d'action
        threading.Thread(target=self._send_dock_goal).start()
        
        response.success = True
        response.message = 'Dock action started'
        return response
    
    def handle_undock(self, request, response):
        """Traiter la demande de service undock"""
        self.get_logger().info('Received undock request')
        
        with self._lock:
            # Vérifier si une action est en cours
            if self._undock_in_progress:
                response.success = False
                response.message = 'Undock action already in progress'
                return response
            
            if self._dock_in_progress:
                response.success = False
                response.message = 'Dock action in progress, please wait'
                return response
            
            # Marquer le début de l'action undock
            self._undock_in_progress = True
        
        # Démarrer un thread pour traiter l'appel d'action
        threading.Thread(target=self._send_undock_goal).start()
        
        response.success = True
        response.message = 'Undock action started'
        return response
    
    def _send_dock_goal(self):
        """Envoyer un but dock et attendre sa complétion"""
        try:
            # Attendre le serveur d'action
            self.get_logger().info('Waiting for dock action server...')
            if not self._dock_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('Dock action server not available')
                with self._lock:
                    self._dock_in_progress = False
                return
            
            # Envoyer le but
            self.get_logger().info('Sending dock goal...')
            goal_msg = Dock.Goal()
            
            # Envoyer le but et obtenir un futur
            send_goal_future = self._dock_client.send_goal_async(goal_msg)
            
            # Attendre de manière synchrone que le but soit accepté
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('Dock goal rejected')
                with self._lock:
                    self._dock_in_progress = False
                return
            
            self.get_logger().info('Dock goal accepted, waiting for result...')
            
            # Obtenir le futur du résultat
            result_future = goal_handle.get_result_async()
            
            # Attendre de manière synchrone le résultat
            rclpy.spin_until_future_complete(self, result_future)
            
            self.get_logger().info('Dock action completed')
        except Exception as e:
            self.get_logger().error(f'Error in dock action: {str(e)}')
        finally:
            # Marquer l'action comme terminée dans tous les cas
            with self._lock:
                self._dock_in_progress = False
    
    def _send_undock_goal(self):
        """Envoyer un but undock et attendre sa complétion"""
        try:
            # Attendre le serveur d'action
            self.get_logger().info('Waiting for undock action server...')
            if not self._undock_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('Undock action server not available')
                with self._lock:
                    self._undock_in_progress = False
                return
            
            # Envoyer le but
            self.get_logger().info('Sending undock goal...')
            goal_msg = Undock.Goal()
            
            # Envoyer le but et obtenir un futur
            send_goal_future = self._undock_client.send_goal_async(goal_msg)
            
            # Attendre de manière synchrone que le but soit accepté
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('Undock goal rejected')
                with self._lock:
                    self._undock_in_progress = False
                return
            
            self.get_logger().info('Undock goal accepted, waiting for result...')
            
            # Obtenir le futur du résultat
            result_future = goal_handle.get_result_async()
            
            # Attendre de manière synchrone le résultat
            rclpy.spin_until_future_complete(self, result_future)
            
            self.get_logger().info('Undock action completed')
        except Exception as e:
            self.get_logger().error(f'Error in undock action: {str(e)}')
        finally:
            # Marquer l'action comme terminée dans tous les cas
            with self._lock:
                self._undock_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = DockBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
