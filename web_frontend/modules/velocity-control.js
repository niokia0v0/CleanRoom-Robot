/**
 * velocity-control.js
 * Module de contrôle de la vitesse du robot
 * Gère les commandes de mouvement et la synchronisation des contrôles de vitesse
 */

(function() {
    // Variables locales du module
    let intervalMouvement = null; // Intervalle pour l'envoi périodique des commandes
    
    // Références aux éléments d'interface
    let inputLineaire, sliderLineaire;
    let inputAngulaire, sliderAngulaire;
    let btnForward, btnLeft, btnRight;
    
    // Variable pour le topic cmd_vel
    let cmdVelTopic = null;
    
    // Variables pour le contrôle clavier
    const activeKeys = new Set(); // Ensemble des touches actives
    let keyboardControlActive = false; // État du contrôle clavier
    let animationFrameId = null; // ID pour cancelAnimationFrame
    
    // Constantes physiques
    const WHEEL_BASE = 0.235; // Distance entre les roues en mètres
    const MAX_SPEED = 0.3; // Vitesse linéaire maximale en m/s
    
    /**
     * Initialisation du module
     */
    function init() {
        // Récupération des références aux éléments DOM
        inputLineaire = document.getElementById('input-lineaire');
        sliderLineaire = document.getElementById('slider-lineaire');
        inputAngulaire = document.getElementById('input-angulaire');
        sliderAngulaire = document.getElementById('slider-angulaire');
        
        btnForward = document.getElementById('btn-forward');
        btnLeft = document.getElementById('btn-left');
        btnRight = document.getElementById('btn-right');
        
        // Configuration de la synchronisation des contrôles
        synchroniserControles(inputLineaire, sliderLineaire);
        synchroniserControles(inputAngulaire, sliderAngulaire);
        
        // Écoute des événements de nettoyage
        document.addEventListener('app-cleanup', cleanup);
        document.addEventListener('ros-disconnected', cleanup);
        
        // Écoute de l'événement de connexion ROS
        document.addEventListener('ros-connected', initializeCmdVelTopic);

        document.addEventListener('semiauto-nav-stopped', function() {
            // S'assurer que toute commande de mouvement existante est arrêtée
            if(intervalMouvement) {
                clearInterval(intervalMouvement);
                intervalMouvement = null;
            }
            
            // Envoyer une commande de vitesse nulle pour garantir l'arrêt du robot
            if (cmdVelTopic && RobotApp.isConnected) {
                const twistStamped = new ROSLIB.Message({
                    twist: {
                        linear: { x: 0, y: 0, z: 0 },
                        angular: { x: 0, y: 0, z: 0 }
                    }
                });
                cmdVelTopic.publish(twistStamped);
            }
        });

        console.log("Module velocity-control initialisé ");
    }

    
    /**
     * Initialise le topic cmd_vel
     */
    function initializeCmdVelTopic() {
        // Vérification de la connexion ROS
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation du topic cmd_vel sans connexion ROS");
            return;
        }
        
        // Création du topic pour les commandes de vitesse
        cmdVelTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/TwistStamped'
        });
        
        console.log("Topic cmd_vel initialisé");
        
        // Déclencher un événement pour informer que le topic de vitesse est prêt
        RobotApp.triggerEvent('vel-topic-initialized');
    }
    
    /**
     * Synchronise les contrôles de vitesse (input number et slider)
     * @param {HTMLElement} input - Élément input de type number
     * @param {HTMLElement} slider - Élément input de type range (slider)
     */
    function synchroniserControles(input, slider) {
        input.addEventListener('input', () => slider.value = input.value);
        slider.addEventListener('input', () => input.value = slider.value);
    }
    
    /**
     * Envoie des commandes de mouvement au robot
     * @param {number} linear - Vitesse linéaire (m/s)
     * @param {number} angular - Vitesse angulaire (rad/s)
     */
    function move(linear, angular) {
        // Arrêt préalable de tout intervalle existant et nettoyage
        if(intervalMouvement) {
            clearInterval(intervalMouvement);
            intervalMouvement = null;
        }
        
        // Vérification de la connexion
        if(!RobotApp.ros || !RobotApp.isConnected) {
            alert('Veuillez d\'abord établir la connexion!');
            return;
        }
        
        // Création et envoi du message de commande
        const sendTwist = () => {
            const twistStamped = new ROSLIB.Message({
                twist: {
                    linear: { x: linear, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: angular }
                }
            });
            
            if (cmdVelTopic) {
                cmdVelTopic.publish(twistStamped);
            } else {
                console.error("Topic cmd_vel non initialisé");
            }
        };
        
        // Premier envoi immédiat puis configuration de l'envoi périodique
        sendTwist();
        intervalMouvement = setInterval(sendTwist, 50); // Intervalle: 50ms période
    }
    
    /**
     * Arrête le mouvement du robot
     */
    function stop() {
        // Arrêt de l'envoi périodique des commandes et nettoyage
        if(intervalMouvement) {
            clearInterval(intervalMouvement);
            intervalMouvement = null;
        }
        
        // Vérification de la connexion
        if (!RobotApp.ros || !RobotApp.isConnected) {
            return;
        }
        
        // Envoi d'une commande d'arrêt si connecté
        const twistStamped = new ROSLIB.Message({
            twist: {
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 }
            }
        });
        
        if (cmdVelTopic) {
            cmdVelTopic.publish(twistStamped);
        } else {
            console.error("Topic cmd_vel non initialisé");
        }
    }
    
    /**
     * Nettoie les ressources du module
     */
    function cleanup() {
        // Arrêt de l'intervalle de mouvement
        if(intervalMouvement) {
            clearInterval(intervalMouvement);
            intervalMouvement = null;
        }
        
        // Nettoyage du topic cmd_vel
        if (cmdVelTopic) {
            cmdVelTopic = null;
        }
        
        // Nettoyage du contrôle clavier
        activeKeys.clear();
        
        // Arrêt du contrôle clavier
        if (keyboardControlActive) {
            keyboardControlActive = false;
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
                animationFrameId = null;
            }
        }
    }
    
    /**
     * Calcule les vitesses linéaire et angulaire en fonction des touches pressées
     * @returns {Object} Objet contenant les vitesses linéaire et angulaire
     */
    function calculateVelocitiesFromKeys() {
        // Vérifier si les touches de contrôle sont actives
        const forwardActive = activeKeys.has('j') || activeKeys.has('J');
        const leftActive = activeKeys.has('k') || activeKeys.has('K');
        const rightActive = activeKeys.has('l') || activeKeys.has('L');
        
        // Si aucune touche n'est active, retourner des vitesses nulles
        if (!forwardActive && !leftActive && !rightActive) {
            return { linear: 0, angular: 0 };
        }
        
        // Obtenir les valeurs de vitesse depuis les contrôles UI
        const linearSpeed = forwardActive ? (parseFloat(inputLineaire.value) || 0) : 0;
        const angularValue = parseFloat(inputAngulaire.value) || 0;
        
        // Calculer la vitesse angulaire en fonction des touches gauche/droite
        let angularSpeed = 0;
        if (leftActive) angularSpeed += angularValue;
        if (rightActive) angularSpeed -= angularValue;
        
        // Calculer les vitesses des roues selon le modèle différentiel
        const leftWheelSpeed = linearSpeed - (angularSpeed * WHEEL_BASE) / 2;
        const rightWheelSpeed = linearSpeed + (angularSpeed * WHEEL_BASE) / 2;
        
        // Vérifier si les vitesses des roues dépassent la limite
        const maxWheelSpeed = Math.max(Math.abs(leftWheelSpeed), Math.abs(rightWheelSpeed));
        
        // Si une des roues dépasse la vitesse maximale, ajuster les deux proportionnellement
        if (maxWheelSpeed > MAX_SPEED) {
            const scaleFactor = MAX_SPEED / maxWheelSpeed;
            return {
                linear: linearSpeed * scaleFactor,
                angular: angularSpeed * scaleFactor
            };
        }
        
        // Retourner les vitesses calculées
        return { linear: linearSpeed, angular: angularSpeed };
    }
    
    /**
     * Envoie les commandes de vitesse en continu basées sur les touches actives
     */
    function updateKeyboardControl() {
        if (!keyboardControlActive) return;
        
        // Calculer les vitesses basées sur les touches actives
        const { linear, angular } = calculateVelocitiesFromKeys();
        
        // Si aucune touche n'est active, arrêter le contrôle
        if (linear === 0 && angular === 0 && activeKeys.size === 0) {
            stopKeyboardControl();
            return;
        }
        
        // Envoyer la commande de mouvement
        if (RobotApp.ros && RobotApp.isConnected && cmdVelTopic) {
            const twistStamped = new ROSLIB.Message({
                twist: {
                    linear: { x: linear, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: angular }
                }
            });
            cmdVelTopic.publish(twistStamped);
        }
        
        // Planifier la prochaine mise à jour
        animationFrameId = requestAnimationFrame(updateKeyboardControl);
    }
    
    /**
     * Démarre le contrôle par clavier
     */
    function startKeyboardControl() {
        if (keyboardControlActive) return;
        
        keyboardControlActive = true;
        updateKeyboardControl();
    }
    
    /**
     * Arrête le contrôle par clavier et envoie une commande d'arrêt
     */
    function stopKeyboardControl() {
        if (!keyboardControlActive) return;
        
        keyboardControlActive = false;
        
        if (animationFrameId) {
            cancelAnimationFrame(animationFrameId);
            animationFrameId = null;
        }
        
        // Envoyer une commande d'arrêt
        if (RobotApp.ros && RobotApp.isConnected && cmdVelTopic) {
            const twistStamped = new ROSLIB.Message({
                twist: {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 }
                }
            });
            cmdVelTopic.publish(twistStamped);
        }
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Gestionnaires d'événements pour les touches
    document.addEventListener('keydown', function(event) {
        // Vérifier si la touche est une des touches de contrôle (j, k, l)
        const key = event.key.toLowerCase();
        if (key === 'j' || key === 'k' || key === 'l') {
            // Empêcher le défilement par défaut
            event.preventDefault();
            
            // Ajouter la touche à l'ensemble des touches actives
            activeKeys.add(event.key);
            
            // Démarrer le contrôle si ce n'est pas déjà fait
            startKeyboardControl();
        }
    });
    
    document.addEventListener('keyup', function(event) {
        // Vérifier si la touche est une des touches de contrôle (j, k, l)
        const key = event.key.toLowerCase();
        if (key === 'j' || key === 'k' || key === 'l') {
            // Empêcher le défilement par défaut
            event.preventDefault();
            
            // Retirer la touche de l'ensemble des touches actives
            activeKeys.delete(event.key);
            
            // Si aucune touche de contrôle n'est active, arrêter le contrôle
            if (!activeKeys.has('j') && !activeKeys.has('J') && 
                !activeKeys.has('k') && !activeKeys.has('K') && 
                !activeKeys.has('l') && !activeKeys.has('L')) {
                stopKeyboardControl();
            }
        }
    });
    
    // Arrêter le contrôle quand la fenêtre perd le focus
    window.addEventListener('blur', function() {
        activeKeys.clear();
        stopKeyboardControl();
    });
    
    // Exposer l'API publique du module
    window.VelocityControl = {
        move: move,
        stop: stop
    };
})();
