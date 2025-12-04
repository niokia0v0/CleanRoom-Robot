/**
 * position-display.js
 * Module d'affichage de la position du robot
 * Ce module gère l'affichage des coordonnées du robot
 */

(function() {
    // Variables locales du module
    let odomTopic = null;         // Topic ROS pour l'odométrie
    let isSubscribed = false;     // État d'abonnement au topic
    
    // Éléments DOM
    let xValueElement;            // Élément pour afficher la coordonnée X
    let yValueElement;            // Élément pour afficher la coordonnée Y
    let thetaValueElement;        // Élément pour afficher l'angle theta
    let resetButton;              // Bouton pour réinitialiser l'odométrie (désactivé)
    let feedbackElement;          // Élément pour afficher le feedback
    
    /**
     * Initialisation du module
     */
    function init() {
        // Récupération des éléments DOM
        xValueElement = document.getElementById('position-x-value');
        yValueElement = document.getElementById('position-y-value');
        thetaValueElement = document.getElementById('position-theta-value');
        resetButton = document.getElementById('btn-reset-position');
        feedbackElement = document.getElementById('reset-feedback');
        
        // Créer un élément caché pour stocker l'angle en radians
        const thetaRadiansElement = document.createElement('span');
        thetaRadiansElement.id = 'position-theta-radians';
        thetaRadiansElement.style.display = 'none';
        document.body.appendChild(thetaRadiansElement);

        // Vérification des éléments
        if (!xValueElement || !yValueElement || !thetaValueElement) {
            console.error("Éléments d'affichage de position non trouvés");
            return;
        }
        
        // Désactiver le bouton de réinitialisation et afficher un message
        if (resetButton) {
            resetButton.disabled = true;
            resetButton.title = "Service de réinitialisation non disponible";
            resetButton.addEventListener('click', function() {
                showFeedback("Service de réinitialisation non disponible");
            });
        }
        
        // Écoute des événements ROS
        document.addEventListener('ros-connected', initializePositionTopics);
        document.addEventListener('ros-disconnected', cleanupPosition);
        document.addEventListener('ros-error', cleanupPosition);
        document.addEventListener('app-cleanup', cleanupPosition);
        
        console.log("Module position-display initialisé");
    }
    
    /**
     * Initialise les topics après connexion ROS
     */
    function initializePositionTopics() {
        // Ne rien faire si déjà abonné
        if (isSubscribed) return;
        
        // Vérifier que ROS est connecté
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation des topics de position sans connexion ROS");
            return;
        }
        
        // Créer le topic pour l'odométrie
        odomTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/odom',
            messageType: 'nav_msgs/msg/Odometry'
        });
        
        // S'abonner au topic d'odométrie
        odomTopic.subscribe(function(message) {
            updatePositionDisplay(message);
        });
        
        isSubscribed = true;
        console.log("Abonnement au topic d'odométrie réussi");
    }
    
    /**
     * Met à jour l'affichage de la position
     * @param {Object} message - Message ROS contenant les données d'odométrie
     */
    function updatePositionDisplay(message) {
        // Extraire les coordonnées de position
        const position = message.pose.pose.position;
        const orientation = message.pose.pose.orientation;
        
        // Calculer l'angle theta à partir du quaternion
        const theta = quaternionToTheta(orientation);
        document.getElementById('position-theta-radians').textContent = theta.toString();
        const thetaDMS = radiansToDMS(theta);
        
        // Mettre à jour l'affichage
        xValueElement.textContent = position.x.toFixed(3);
        yValueElement.textContent = position.y.toFixed(3);
        thetaValueElement.textContent = thetaDMS;
    }
    
    /**
     * Convertit un quaternion en angle theta (en radians)
     * @param {Object} q - Quaternion (x, y, z, w)
     * @returns {number} - Angle theta en radians
     */
    function quaternionToTheta(q) {
        // Formule pour extraire l'angle yaw (theta) d'un quaternion
        return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
    
    /**
     * Affiche un message de feedback
     * @param {string} message - Message à afficher
     */
    function showFeedback(message) {
        if (feedbackElement) {
            feedbackElement.textContent = message;
        }
    }
    

    function radiansToDMS(radians) {
        // Convertir les radians en degrés
        let degrees = radians * (180 / Math.PI);
      
        // Gérer le signe positif/négatif
        const sign = degrees < 0 ? -1 : 1;
        degrees = Math.abs(degrees);
      
        // Calculer degrés, minutes, secondes
        const deg = Math.floor(degrees);
        const minutes = (degrees - deg) * 60;
        const min = Math.floor(minutes);
        const sec = ((minutes - min) * 60).toFixed(1);
      
        // Formater en notation degrés-minutes-secondes
        return `${sign * deg}°${min}'${sec}"`;
    }

    /**
     * Nettoie les ressources du module
     */
    function cleanupPosition() {
        // Se désabonner du topic d'odométrie
        if (odomTopic) {
            odomTopic.unsubscribe();
            odomTopic = null;
        }
        
        // Réinitialiser l'affichage
        if (xValueElement) xValueElement.textContent = "---";
        if (yValueElement) yValueElement.textContent = "---";
        if (thetaValueElement) thetaValueElement.textContent = "---";
        
        // Effacer le feedback
        showFeedback("");
        
        isSubscribed = false;
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Exposer certaines fonctions pour les autres modules
    window.PositionDisplay = {
        isAvailable: function() { return isSubscribed; }
    };
})();
