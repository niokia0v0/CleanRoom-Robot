/**
 * auto-mode.js
 * Module de gestion du mode automatique
 * Ce module permet d'activer et désactiver le mode automatique du robot
 */

(function() {
    // Variables locales du module
    let autoModeButton; // Bouton pour activer/désactiver le mode auto
    let tooltip; // Infobulle pour les messages d'aide
    let autoModeService = null; // Service ROS pour le mode auto
    let dockStatusTopic = null; // Topic pour vérifier l'état du dock
    let isDocked = false; // État actuel du dock
    let isAutoMode = false; // État actuel du mode auto
    let tooltipTimeout = null; // Timeout pour l'infobulle
    let isServiceCallPending = false; // Indique si un appel de service est en cours
    let serviceCallTimeoutId = null; // ID du timeout pour l'appel de service
    let wasUndockedDuringAuto = false; // Indique si le robot était non docké pendant le mode auto

    
    /**
     * Initialisation du module
     */
    function init() {
        // Créer le bouton de mode auto
        createAutoModeButton();
        
        // Écoute des événements
        document.addEventListener('ros-connected', initializeTopicsAndServices);
        document.addEventListener('ros-disconnected', cleanup);
        document.addEventListener('ros-error', cleanup);
        document.addEventListener('app-cleanup', cleanup);
        document.addEventListener('keydown', handleKeyDown);
        
        console.log("Module auto-mode initialisé");
    }
    
    /**
     * Crée le bouton de mode automatique et l'ajoute à l'interface
     */
    function createAutoModeButton() {
        // Trouver l'élément keyboard-controls-info
        const keyboardControls = document.querySelector('.keyboard-controls-info');
        
        if (!keyboardControls) {
            console.error("Élément keyboard-controls-info non trouvé");
            return;
        }
        
        // Créer un conteneur flex pour aligner le texte et le bouton
        const container = document.createElement('div');
        container.className = 'keyboard-auto-container';
        container.style.display = 'flex';
        container.style.alignItems = 'center';
        container.style.flexWrap = 'wrap';
        
        const originalText = keyboardControls.textContent || keyboardControls.innerText;
        keyboardControls.innerHTML = '';
        
        const textSpan = document.createElement('span');
        textSpan.textContent = originalText;
        textSpan.style.marginRight = '10px';
        container.appendChild(textSpan);
        
        // Créer le bouton
        autoModeButton = document.createElement('button');
        autoModeButton.id = 'btn-auto-mode';
        autoModeButton.className = 'auto-mode-button';
        autoModeButton.textContent = 'Mode Auto';
        autoModeButton.disabled = true;
        autoModeButton.addEventListener('click', toggleAutoMode);
        
        // Appliquer le style au bouton
        autoModeButton.style.backgroundColor = '#2196F3';
        autoModeButton.style.color = 'white';
        autoModeButton.style.padding = '8px 15px';
        autoModeButton.style.borderRadius = '4px';
        autoModeButton.style.border = 'none';
        autoModeButton.style.cursor = 'pointer';
        autoModeButton.style.transition = 'background-color 0.3s';
        
        // Créer l'infobulle
        tooltip = document.createElement('div');
        tooltip.className = 'auto-mode-tooltip';
        tooltip.textContent = 'Revenir à la station de charge pour activer le mode auto';
        
        // Ajouter un gestionnaire d'événements pour afficher l'infobulle
        autoModeButton.addEventListener('mouseenter', showTooltip);
        autoModeButton.addEventListener('mouseleave', hideTooltip);
        
        // Ajouter le bouton au conteneur
        container.appendChild(autoModeButton);
        
        // Ajouter le tooltip au conteneur
        container.appendChild(tooltip);
        
        // Ajouter le conteneur à l'élément keyboard-controls-info
        keyboardControls.appendChild(container);
    }

    /**
     * Affiche l'infobulle après un délai
     */
    function showTooltip() {
        // Ne pas afficher l'infobulle si le bouton est actif
        if (isDocked || isAutoMode) return;
        
        tooltipTimeout = setTimeout(() => {
            const buttonRect = autoModeButton.getBoundingClientRect();
            tooltip.style.left = `${buttonRect.left}px`;
            tooltip.style.display = 'block';
        }, 500);
    }
    
    /**
     * Cache l'infobulle
     */
    function hideTooltip() {
        clearTimeout(tooltipTimeout);
        tooltip.style.display = 'none';
    }
    
    /**
     * Initialise les topics et services après connexion ROS
     */
    function initializeTopicsAndServices() {
        // Vérifier que ROS est connecté
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation sans connexion ROS");
            return;
        }
        
        // Créer le service pour le mode auto
        autoModeService = new ROSLIB.Service({
            ros: RobotApp.ros,
            name: '/mode_auto',
            serviceType: 'std_srvs/srv/SetBool'
        });
        
        // Créer le topic pour l'état du dock
        dockStatusTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/dock_status',
            messageType: 'irobot_create_msgs/msg/DockStatus'
        });
        
        // S'abonner au topic d'état du dock
        dockStatusTopic.subscribe(function(message) {
            try {
                // Dans le message DockStatus, is_docked est un champ direct
                if (message && typeof message.is_docked !== 'undefined') {
                    const previousDockState = isDocked;
                    isDocked = message.is_docked;
                    
                    // Si le robot est en mode auto et se détache du dock, marquer qu'il était non docké
                    if (isAutoMode && previousDockState && !isDocked) {
                        wasUndockedDuringAuto = true;
                        //console.log("Robot en mode auto détaché du dock");
                    }
                    
                    // Si le robot est en mode auto, a été détaché, et revient au dock, terminer le mode auto
                    if (isAutoMode && !previousDockState && isDocked && wasUndockedDuringAuto) {
                        //console.log("Robot revenu au dock, fin du cycle auto");
                        completeAutoModeCycle();
                    }
                    
                    updateButtonState();
                } else {
                    console.warn("Message de dock invalide ou incomplet:", message);
                }
            } catch (e) {
                console.error("Erreur lors du traitement des données de dock:", e, "Message:", message);
            }
        });

        
        console.log("Topics et services du mode auto initialisés");
    }
    
    /**
     * Met à jour l'état du bouton en fonction de l'état du dock et du mode auto
     */
    function updateButtonState() {
        if (!autoModeButton) return;
        
        if (isServiceCallPending) {
            // Service en cours d'appel, désactiver le bouton
            autoModeButton.disabled = true;
            autoModeButton.style.backgroundColor = '#CCCCCC';
            return;
        }
        
        if (isAutoMode) {
            // Mode auto actif, bouton actif
            autoModeButton.disabled = false;
            autoModeButton.style.backgroundColor = '#FF5722'; // Orange pour indiquer "Arrêter"
            autoModeButton.textContent = 'Arrêter Auto';
        } else if (isDocked) {
            // Docké mais pas en mode auto, bouton actif
            autoModeButton.disabled = false;
            autoModeButton.style.backgroundColor = '#2196F3'; // Bleu pour indiquer "Démarrer"
            autoModeButton.textContent = 'Mode Auto';
        } else {
            // Ni docké ni en mode auto, bouton inactif
            autoModeButton.disabled = true;
            autoModeButton.style.backgroundColor = '#CCCCCC';
            autoModeButton.textContent = 'Mode Auto';
        }
    }
    
    /**
     * Active ou désactive le mode automatique
     */
    function toggleAutoMode() {
        // Vérifier que le service est disponible
        if (!autoModeService || !RobotApp.isConnected) {
            console.error("Service mode auto non disponible");
            return;
        }
        
        // Si déjà en attente d'un appel de service, ne rien faire
        if (isServiceCallPending) return;
        
        // Si on essaie d'activer le mode auto mais que le robot n'est pas docké, ne rien faire
        if (!isAutoMode && !isDocked) {
            console.warn("Impossible d'activer le mode auto: robot non docké");
            return;
        }
        
        // Définir la valeur à envoyer (inverse de l'état actuel)
        const newAutoModeState = !isAutoMode;
        
        // Marquer comme en cours d'appel
        isServiceCallPending = true;
        updateButtonState();
        
        // Créer la requête
        const request = new ROSLIB.ServiceRequest({
            data: newAutoModeState
        });
        
        //console.log("Appel du service mode_auto avec data:", newAutoModeState);
        
        // Nettoyer tout timeout existant
        if (serviceCallTimeoutId) {
            clearTimeout(serviceCallTimeoutId);
            serviceCallTimeoutId = null;
        }
        
        try {
            // Appeler le service
            autoModeService.callService(request, 
                // Fonction de succès - probablement jamais appelée
                function(result) {
                    console.log("Réponse du service mode_auto:", result);
                    completeServiceCall(newAutoModeState);
                }, 
                // Fonction d'erreur
                function(error) {
                    console.error("Erreur lors de l'appel du service mode auto:", error);
                    
                    // Si c'est une erreur de timeout, on considère que la commande a été exécutée
                    if (error && error.toString().includes("Timeout")) {
                        console.log("Timeout du service, on suppose que la commande a été exécutée");
                        completeServiceCall(newAutoModeState);
                    } else {
                        // Pour les autres erreurs, on considère que ça a échoué
                        completeServiceCall(isAutoMode); // Garder l'état actuel
                        alert("Erreur lors de la modification du mode auto");
                    }
                }
            );
        } catch (e) {
            console.error("Exception lors de l'appel du service:", e);
            completeServiceCall(isAutoMode); // Garder l'état actuel
            alert("Erreur lors de la modification du mode auto");
        }
        
        // Supposer que la commande a été exécutée après un court délai
        // même si nous ne recevons pas de réponse du service
        serviceCallTimeoutId = setTimeout(() => {
            //console.log("Considérant que la commande a été exécutée après délai");
            completeServiceCall(newAutoModeState);
        }, 2000); // Attendre 2 secondes puis supposer que ça a fonctionné
    }
    
    /**
     * Termine l'appel de service et met à jour l'état
     * @param {boolean} newState - Nouvel état du mode auto
     */
    function completeServiceCall(newState) {
        // Annuler le timeout si existant
        if (serviceCallTimeoutId) {
            clearTimeout(serviceCallTimeoutId);
            serviceCallTimeoutId = null;
        }
        
        // Seulement mettre à jour si l'appel est toujours en cours
        if (isServiceCallPending) {
            isAutoMode = newState;
            isServiceCallPending = false;
            
            // Si le mode auto est activé, réinitialiser wasUndockedDuringAuto
            if (isAutoMode) {
                wasUndockedDuringAuto = false;
                document.dispatchEvent(new CustomEvent('auto-mode-started'));
            } else {
                document.dispatchEvent(new CustomEvent('auto-mode-stopped'));
            }
            
            updateButtonState();
            //console.log("Mode auto " + (isAutoMode ? "activé" : "désactivé"));
        }
    }

    
    /**
     * Gère les événements clavier pour désactiver le mode auto
     * @param {KeyboardEvent} event - Événement clavier
     */
    function handleKeyDown(event) {
        // Si le mode auto est actif et que J, K ou L est pressé, désactiver le mode auto
        const key = event.key.toLowerCase();
        if (isAutoMode && (key === 'j' || key === 'k' || key === 'l')) {
            if (autoModeService && RobotApp.isConnected && !isServiceCallPending) {
                toggleAutoMode();
            }
        }
    }
    
    /**
     * Termine automatiquement un cycle de mode auto lorsque le robot revient au dock
     */
    function completeAutoModeCycle() {
        // Vérifier que le service est disponible
        if (!autoModeService || !RobotApp.isConnected || !isAutoMode) {
            return;
        }
        
        //console.log("Fin automatique du cycle mode auto");
        
        // Réinitialiser l'état du mode auto
        isAutoMode = false;
        wasUndockedDuringAuto = false;
        
        // Déclencher l'événement de fin de mode auto
        document.dispatchEvent(new CustomEvent('auto-mode-stopped'));
        document.dispatchEvent(new CustomEvent('auto-mode-cycle-completed'));
        
        // Mettre à jour l'état du bouton
        updateButtonState();
    }

    /**
     * Nettoie les ressources du module
     */
    function cleanup() {
        // Annuler le timeout si existant
        if (serviceCallTimeoutId) {
            clearTimeout(serviceCallTimeoutId);
            serviceCallTimeoutId = null;
        }
        
        // Se désabonner du topic d'état du dock
        if (dockStatusTopic) {
            dockStatusTopic.unsubscribe();
            dockStatusTopic = null;
        }
        
        // Réinitialiser les variables
        autoModeService = null;
        isDocked = false;
        isAutoMode = false;
        isServiceCallPending = false;
        
        // Mettre à jour l'état du bouton
        updateButtonState();
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Exposer certaines fonctions pour les autres modules
    window.AutoMode = {
        isActive: function() { return isAutoMode; }
    };
})();
