/**
 * dock-control.js
 * Module de contrôle des fonctions de dock et undock via service
 */

(function() {
    // Variables locales du module
    let btnDock, btnUndock;
    let feedbackArea;
    let dockService = null;
    let undockService = null;
    let isDocked = false;
    let dockStatusTopic = null;
    let isWarningActive = false;
    let positionTopic = null;
    
    /**
     * Initialisation du module
     */
    function init() {
        // Récupération des références aux éléments DOM
        btnDock = document.getElementById('btn-dock');
        btnUndock = document.getElementById('btn-undock');
        feedbackArea = document.getElementById('dock-feedback');
        
        // Configuration des événements pour les boutons
        if (btnDock) btnDock.addEventListener('click', handleDockClick);
        if (btnUndock) btnUndock.addEventListener('click', sendUndockRequest);
        
        // Écoute des événements ROS
        document.addEventListener('ros-connected', initializeServices);
        document.addEventListener('ros-disconnected', cleanup);
        document.addEventListener('ros-error', cleanup);
        document.addEventListener('app-cleanup', cleanup);
        
        console.log("Module dock-control initialisé");
    }
    
    /**
     * Initialise les services pour dock et undock
     */
    function initializeServices() {
        // Vérification de la connexion ROS
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation des services sans connexion ROS");
            return;
        }
        
        // Création du service pour dock
        dockService = new ROSLIB.Service({
            ros: RobotApp.ros,
            name: '/trigger_dock',
            serviceType: 'std_srvs/srv/Trigger'
        });
        
        // Création du service pour undock
        undockService = new ROSLIB.Service({
            ros: RobotApp.ros,
            name: '/trigger_undock',
            serviceType: 'std_srvs/srv/Trigger'
        });
        
        // Création du topic pour l'état du dock
        dockStatusTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/dock_status',
            messageType: 'irobot_create_msgs/msg/DockStatus'
        });

        // S'abonner au topic d'état du dock
        dockStatusTopic.subscribe(function(message) {
            if (message && typeof message.is_docked !== 'undefined') {
                const previousState = isDocked;
                isDocked = message.is_docked;
                
                // Mettre à jour le feedback si l'état a changé
                if (previousState !== isDocked) {
                    if (isDocked) {
                        showFeedback("Robot amarré à la station de charge");
                    } else {
                        showFeedback("Robot détaché de la station de charge");
                    }
                }
                
                updateButtonsState();
            }
        });

        // S'abonner au topic d'odométrie pour connaître la position
        positionTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/odom',
            messageType: 'nav_msgs/msg/Odometry'
        });

        // Activer les boutons
        updateButtonsState();
        
        // Afficher l'état initial
        showFeedback(isDocked ? "Robot amarré à la station de charge" : "Robot détaché de la station de charge");
        
        console.log("Services dock/undock initialisés");
    }
    
    /**
     * Gère le clic sur le bouton dock
     */
    function handleDockClick() {
        // Si l'avertissement est actif, traiter comme une confirmation
        if (isWarningActive) {
            isWarningActive = false;
            sendDockRequest();
            return;
        }

        // Vérifier la distance par rapport à l'origine
        if (positionTopic) {
            positionTopic.subscribe(function(message) {
                const position = message.pose.pose.position;
                const distance = Math.sqrt(position.x * position.x + position.y * position.y);
                
                // Se désabonner immédiatement après avoir reçu un message
                positionTopic.unsubscribe();
                
                if (distance > 2.5) {
                    // Afficher l'avertissement
                    showFeedback("Distance trop grande. Rapprochez-vous ou cliquez à nouveau pour forcer.");
                    isWarningActive = true;
                } else {
                    // Distance acceptable, procéder au dock
                    sendDockRequest();
                }
            });
        } else {
            // Si le topic de position n'est pas disponible, procéder sans vérification
            sendDockRequest();
        }
    }
    
    /**
     * Envoie une requête de dock (mise à quai)
     */
    function sendDockRequest() {
        if (!dockService || !RobotApp.isConnected) {
            showFeedback("Erreur: Service ROS non disponible");
            return;
        }
        
        // Désactiver les boutons pendant l'opération
        setButtonsState(true);
        showFeedback("Recherche de la station de charge...");
        
        // Créer la requête de service
        const request = new ROSLIB.ServiceRequest({});
        
        // Appeler le service
        dockService.callService(request, function(result) {
            if (result.success) {
                showFeedback("Amarrage en cours... " + result.message);
            } else {
                showFeedback("Échec de la mise à quai: " + result.message);
            }
            updateButtonsState();
        }, function(error) {
            showFeedback("Erreur lors de l'appel du service: " + error);
            updateButtonsState();
        });
    }
    
    /**
     * Envoie une requête de undock (désarrimage)
     */
    function sendUndockRequest() {
        if (!undockService || !RobotApp.isConnected) {
            showFeedback("Erreur: Service ROS non disponible");
            return;
        }
        
        // Désactiver les boutons pendant l'opération
        setButtonsState(true);
        showFeedback("Détachement de la station en cours...");
        
        // Créer la requête de service
        const request = new ROSLIB.ServiceRequest({});
        
        // Appeler le service
        undockService.callService(request, function(result) {
            if (result.success) {
                showFeedback("Détachement en cours... " + result.message);
            } else {
                showFeedback("Échec du désarrimage: " + result.message);
            }
            updateButtonsState();
        }, function(error) {
            showFeedback("Erreur lors de l'appel du service: " + error);
            updateButtonsState();
        });
    }
    
    /**
     * Affiche un message de feedback
     * @param {string} message - Message à afficher
     */
    function showFeedback(message) {
        if (feedbackArea) {
            feedbackArea.textContent = message;
        }
    }
    
    /**
     * Met à jour l'état des boutons en fonction de l'état du dock
     */
    function updateButtonsState() {
        if (!btnDock || !btnUndock) return;
        
        // Activer/désactiver les boutons en fonction de l'état docké
        btnDock.disabled = isDocked;
        btnUndock.disabled = !isDocked;
    }
    
    /**
     * Définit l'état activé/désactivé des boutons
     * @param {boolean} disabled - true pour désactiver, false pour activer
     */
    function setButtonsState(disabled) {
        if (btnDock) btnDock.disabled = disabled;
        if (btnUndock) btnUndock.disabled = disabled;
    }
    
    /**
     * Nettoie les ressources du module
     */
    function cleanup() {
        // Réinitialiser les services
        dockService = null;
        undockService = null;
        
        // Désactiver les boutons
        setButtonsState(true);
        
        // Effacer le feedback
        showFeedback("");
        
        // Réinitialiser l'état d'avertissement
        isWarningActive = false;
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Exposer l'API publique du module
    window.DockControl = {
        dock: handleDockClick,
        undock: sendUndockRequest,
        isDocked: function() { return isDocked; }
    };
})();
