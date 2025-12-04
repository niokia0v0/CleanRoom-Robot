/**
 * battery.js
 * Module de gestion de l'affichage de la batterie
 * Ce module gère l'affichage du niveau de batterie sur la carte
 */

(function() {
    // Variables locales du module
    let batteryTopic = null;      // Topic ROS pour l'état de la batterie
    let batteryPanel = null;      // Élément DOM pour le panneau de batterie
    let percentageElement = null; // Élément pour l'affichage du pourcentage
    let isSubscribed = false;     // État d'abonnement au topic
    
    /**
     * Initialisation du module
     */
    function init() {
        // Récupération de l'élément de batterie existant
        batteryPanel = document.querySelector('.battery-panel');
        
        if (!batteryPanel) {
            console.error("Élément battery-panel non trouvé dans le HTML");
            return;
        }
        
        // Récupération de l'élément d'affichage 
        percentageElement = batteryPanel.querySelector('.battery-percentage');

        voltageElement = batteryPanel.querySelector('#battery-voltage');
        currentElement = batteryPanel.querySelector('#battery-current');
        powerElement = batteryPanel.querySelector('#battery-power');

        
        if (!percentageElement||!voltageElement || !currentElement || !powerElement) {
            console.error("Élément non trouvé dans le panneau de batterie");
            return;
        }
        
        // Écoute des événements ROS
        document.addEventListener('ros-connected', initializeBatteryTopic);
        document.addEventListener('ros-disconnected', cleanupBattery);
        document.addEventListener('ros-error', cleanupBattery);
        document.addEventListener('app-cleanup', cleanupBattery);
        
        console.log("Module batterie initialisé");
    }
    
    /**
     * Initialise le topic de batterie après connexion ROS
     */
    function initializeBatteryTopic() {
        // Ne rien faire si déjà abonné
        if (isSubscribed) return;
        
        // Vérifier que ROS est connecté
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation du topic batterie sans connexion ROS");
            return;
        }
        
        // Créer le topic pour l'état de la batterie
        batteryTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/battery_state',
            messageType: 'sensor_msgs/msg/BatteryState'
        });
        
        // S'abonner au topic de batterie
        batteryTopic.subscribe(function(message) {
            // Mise à jour de l'affichage de l'état de la batterie
            updateBatteryDisplay(message);
        });
        
        isSubscribed = true;
        console.log("Abonnement au topic de batterie réussi");
    }
    
    /**
     * Met à jour l'affichage de l'état de la batterie
     * @param {Object} message - Message ROS contenant les données de batterie
     */
    function updateBatteryDisplay(message) {
        // Vérifier que les éléments existent
        if (!batteryPanel || !percentageElement) {
            console.error("Éléments d'affichage de batterie non disponibles");
            return;
        }
        
        // Calculer le pourcentage de batterie
        const batteryPercentage = Math.round(message.percentage * 100);

        const voltage = message.voltage?.toFixed(2) ?? '---';
        const current = Math.abs(message.current ?? 0).toFixed(2);
        const power = (message.voltage * Math.abs(message.current ?? 0)).toFixed(2);

        // Tension, courant, puissance
        
        
        // Mettre à jour le texte
        percentageElement.textContent = `${batteryPercentage}%`;

        voltageElement.textContent = `${voltage} V`;
        currentElement.textContent = `${current} A`;
        powerElement.textContent = `${power} W`;
        
        powerElement.style.color = message.current < 0 ? '#F44336' : '#4CAF50';// Charge vert - décharge rouge

        // Supprimer les classes précédentes
        percentageElement.classList.remove('battery-high', 'battery-medium', 'battery-low');
        
        // Ajouter la classe appropriée selon le niveau de batterie
        if (batteryPercentage <= 20) {
            percentageElement.classList.add('battery-low');
        } else if (batteryPercentage <= 50) {
            percentageElement.classList.add('battery-medium');
        } else {
            percentageElement.classList.add('battery-high');
        }
        
    }
    
    /**
     * Nettoie les ressources du module
     */
    function cleanupBattery() {
        // Se désabonner du topic de batterie
        if (batteryTopic) {
            batteryTopic.unsubscribe();
            batteryTopic = null;
            isSubscribed = false;
            console.log("Désabonnement du topic de batterie");
        }

        const resetElement = (element, defaultValue) => {
            if (element) {
                element.textContent = defaultValue;
                element.style.color = '';
            }
        };

        resetElement(voltageElement, '--- V');
        resetElement(currentElement, '--- A'); 
        resetElement(powerElement, '--- W');
        
        // Réinitialiser l'affichage avec la couleur blanche par défaut
        if (percentageElement) {
            percentageElement.textContent = '---';
            percentageElement.classList.remove('battery-high', 'battery-medium', 'battery-low');
            // La couleur blanche est déjà définie dans le CSS
        }
    }

    
    /**
     * Réinitialise l'abonnement au topic de batterie
     * Utile si la connexion ROS est rétablie
     */
    function reinitialize() {
        cleanupBattery();
        initializeBatteryTopic();
    }
    
    /**
     * Vérifie si le topic de batterie est disponible
     * @returns {boolean} - true si le topic est disponible, false sinon
     */
    function isTopicAvailable() {
        return isSubscribed && batteryTopic !== null;
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Exposer certaines fonctions pour les autres modules
    window.BatteryModule = {
        updateDisplay: updateBatteryDisplay,
        reinitialize: reinitialize,
        isAvailable: isTopicAvailable
    };
})();
