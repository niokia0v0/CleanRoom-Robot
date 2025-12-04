/**
 * ros-controls.js
 * Module de gestion de la connexion ROS et des événements associés
 * Ce module gère l'établissement de la connexion, la déconnexion et l'état de la connexion avec le serveur ROS
 */

(function() {
    // Variables locales au module
    let statusElement; // Élément d'affichage de l'état de connexion
    let connectButton; // Bouton de connexion
    let disconnectButton; // Bouton de déconnexion

    /**
     * Initialisation du module
     */
    function init() {
        // Récupération des éléments du DOM
        statusElement = document.getElementById('status-connection');
        connectButton = document.getElementById('btn-connect');
        disconnectButton = document.getElementById('btn-disconnect');

        // Configuration des écouteurs d'événements
        connectButton.addEventListener('click', connectToROS);
        disconnectButton.addEventListener('click', disconnectFromROS);

        // Écoute des événements de nettoyage global
        document.addEventListener('app-cleanup', cleanup);

        console.log("Module ros-controls initialisé");
    }

    /**
     * Établir la connexion avec le serveur ROS
     */
    function connectToROS() {
        // Nettoyage préalable de la connexion existante
        if (RobotApp.ros) {
            RobotApp.ros.close();
            RobotApp.ros.removeAllListeners();
            RobotApp.ros = null;
        }

        // Mise à jour de l'interface utilisateur
        updateUI('connecting');
        
        // Création d'une nouvelle instance de connexion ROS
        RobotApp.ros = new ROSLIB.Ros({
            url: RobotApp.config.rosbridge
        });

        // Gestionnaire d'événement de connexion réussie
        RobotApp.ros.on('connection', function() {
            RobotApp.isConnected = true;
            updateUI('connected');
            console.log('Connecté à ROSBridge');
            
            // Déclencher un événement pour informer les autres modules
            RobotApp.triggerEvent('ros-connected');
            
            // Vérifier les topics disponibles (pour debug)
            checkAvailableTopics();
        });

        // Gestionnaire d'erreurs de connexion
        RobotApp.ros.on('error', function(error) {
            RobotApp.isConnected = false;
            updateUI('error');
            console.error('Erreur de connexion:', error);
            
            // Déclencher un événement pour informer les autres modules
            RobotApp.triggerEvent('ros-error', { error });
            
            // Nettoyage des ressources en cas d'erreur
            cleanup();
        });

        // Gestionnaire de fermeture de connexion
        RobotApp.ros.on('close', function() {
            RobotApp.isConnected = false;
            updateUI('disconnected');
            console.log('Connexion fermée');
            
            // Déclencher un événement pour informer les autres modules
            RobotApp.triggerEvent('ros-disconnected');
            
            // Nettoyage des ressources
            cleanup();
        });
    }

    /**
     * Déconnecter du serveur ROS
     */
    function disconnectFromROS() {
        if (RobotApp.ros) {
            RobotApp.ros.close();
        }
    }

    /**
     * Vérifier les topics disponibles (fonction de debug)
     */
    function checkAvailableTopics() {
        if (!RobotApp.ros) return;
        
        RobotApp.ros.getTopics(function(topics) {
            console.log("Topics disponibles:", topics);
        });
    }

    /**
     * Mettre à jour l'interface utilisateur selon l'état de connexion
     * @param {string} etat - État de connexion ('connecting', 'connected', 'disconnected', 'error')
     */
    function updateUI(etat) {
        switch(etat) {
            case 'connecting':
                statusElement.textContent = 'État : Connexion en cours...';
                statusElement.style.color = 'blue';
                connectButton.disabled = true;
                disconnectButton.disabled = true;
                break;
            case 'connected':
                statusElement.textContent = 'État : Connecté';
                statusElement.style.color = 'green';
                connectButton.disabled = true;
                disconnectButton.disabled = false;
                break;
            case 'disconnected':
                statusElement.textContent = 'État : Déconnecté';
                statusElement.style.color = 'red';
                connectButton.disabled = false;
                disconnectButton.disabled = true;
                break;
            case 'error':
                statusElement.textContent = 'État : Erreur de connexion';
                statusElement.style.color = 'orange';
                connectButton.disabled = false;
                disconnectButton.disabled = true;
                break;
        }
    }

    /**
     * Nettoyer les ressources du module
     */
    function cleanup() {
        // La gestion du topic cmd_vel a été déplacée vers velocity-control.js
        
        // Fermeture de la connexion ROS
        if (RobotApp.ros) {
            RobotApp.ros.close();
            RobotApp.ros.removeAllListeners();
            RobotApp.ros = null;
        }
    }

    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);

    // Exposer certaines fonctions pour les autres modules
    window.RosControls = {
        connect: connectToROS,
        disconnect: disconnectFromROS,
        isConnected: function() { return RobotApp.isConnected; }
    };
})();
