/**
 * semiauto-nav.js
 * Module de navigation semi-automatique
 * Ce module permet de naviguer vers un point cible cliqué sur la carte
 */

(function() {
    // Variables locales du module
    let targetPoint = null;       // Point cible {x, y} en coordonnées du monde
    let targetMarker = null;      // Élément DOM pour le marqueur de cible
    let navigationState = 'IDLE'; // États: 'IDLE', 'MOVING', 'PAUSED'
    let blinkInterval = null;     // Intervalle pour le clignotement du marqueur
    let cmdVelTopic = null;       // Topic pour les commandes de vitesse
    let obstacleCheckInterval = null; // Intervalle pour vérifier les obstacles
    let lidarData = [];           // Données du lidar
    let moveInterval = null;      // Intervalle pour l'envoi des commandes de mouvement
    
    // Paramètres de navigation
    const TARGET_THRESHOLD = 0.01;      // Seuil de distance pour considérer la cible atteinte (m)
    const LINEAR_SPEED = 0.25;           // Vitesse linéaire (m/s)
    const ANGULAR_SPEED = 1.5;          // Vitesse angulaire (rad/s)
    const ANGLE_THRESHOLD = 0.05;       // Seuil d'angle pour considérer l'orientation correcte (rad)
    const OBSTACLE_DISTANCE = 0.5;      // Distance minimale pour détecter un obstacle (m)
    const OBSTACLE_ANGLE_RANGE = Math.PI/2; // Plage d'angle pour détecter les obstacles (90 degrés)
    
    /**
     * Initialisation du module
     */
    function init() {
        // Création du marqueur de cible
        createTargetMarker();
        
        // Écoute des événements
        document.addEventListener('ros-connected', initializeTopics);
        document.addEventListener('ros-disconnected', cleanup);
        document.addEventListener('ros-error', cleanup);
        document.addEventListener('app-cleanup', cleanup);
        document.addEventListener('display-mode-changed', handleDisplayModeChange);
        document.addEventListener('keydown', handleKeyDown);
        
        // Écoute de l'événement personnalisé pour la sélection de cible
        document.addEventListener('navigation-target-set', handleTargetSet);
        
        // Écoute des données lidar
        document.addEventListener('lidar-data-updated', function(event) {
            if (event.detail && Array.isArray(event.detail.data)) {
                lidarData = event.detail.data;
            }
        });

        // Écouter les événements du mode auto
        document.addEventListener('auto-mode-started', function() {
            // Arrêter la navigation semi-automatique si elle est en cours
            if (navigationState !== 'IDLE') {
                stopNavigation();
            }
        });

        console.log("Module de navigation semi-automatique initialisé");
    }
    
    /**
     * Crée le marqueur de cible sur la carte
     */
    function createTargetMarker() {
        // Vérifier si le marqueur existe déjà
        if (targetMarker) return;
        
        // Créer l'élément DOM du marqueur
        targetMarker = document.createElement('div');
        targetMarker.className = 'target-marker';
        targetMarker.style.display = 'none';
        
        // Ajouter au conteneur de la carte
        const mapContainer = document.querySelector('.map-container');
        if (mapContainer) {
            mapContainer.appendChild(targetMarker);
        }
    }
    
    /**
     * Initialise les topics après connexion ROS
     */
    function initializeTopics() {
        // Vérifier que ROS est connecté
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation des topics sans connexion ROS");
            return;
        }
        
        // Créer le topic pour les commandes de vitesse
        cmdVelTopic = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/TwistStamped'
        });
        
        console.log("Topics de navigation initialisés");
    }
    
    /**
     * Gère l'événement de définition de cible
     * @param {CustomEvent} event - Événement avec les coordonnées de la cible
     */
    function handleTargetSet(event) {
        // Vérifier si le mode auto est actif
        if (window.AutoMode && window.AutoMode.isActive()) {
            // Afficher un avertissement
            alert("Mode auto actif. Appuyez sur J, K ou L pour désactiver le mode auto avant d'utiliser la mode semi-auto.");
            return;
        }

        // Extraire les coordonnées de la cible
        const worldCoordinates = event.detail.coordinates;
        
        // Arrêter la navigation en cours si existante
        stopNavigation();
        
        // Définir la nouvelle cible
        targetPoint = worldCoordinates;
        
        // Afficher le marqueur de cible
        showTargetMarker(event.detail.mapCoordinates);
        
        // Vérifier si le robot est docké
        checkDockStatusAndStart(event.detail.mapCoordinates);
    }

    /**
     * Vérifie l'état du dock et démarre la navigation
     * @param {Object} mapCoordinates - Coordonnées du point cible sur la carte
     */
    function checkDockStatusAndStart(mapCoordinates) {
        // Vérifier si le topic de statut du dock est disponible via le module DockControl
        if (window.DockControl && typeof window.DockControl.isDocked === 'function') {
            if (window.DockControl.isDocked()) {
                
                // Utiliser le service undock du module DockControl
                window.DockControl.undock();
                
                // Attendre que le robot soit détaché avant de démarrer la navigation
                const checkInterval = setInterval(function() {
                    if (!window.DockControl.isDocked()) {
                        clearInterval(checkInterval);
                        startNavigation();
                    }
                }, 500); // Vérifier toutes les 500ms
                
                // Timeout de sécurité au cas où le statut ne change pas
                setTimeout(function() {
                    clearInterval(checkInterval);
                    startNavigation();
                }, 5000); // 5 secondes de timeout
            } else {
                // Si le robot n'est pas docké, démarrer directement la navigation
                startNavigation();
            }
        } else {
            // Si le module DockControl n'est pas disponible, simplement démarrer la navigation
            startNavigation();
        }
    }
    
    /**
     * Affiche le marqueur de cible sur la carte
     * @param {Object} mapCoordinates - Coordonnées {x, y} sur la carte
     */
    function showTargetMarker(mapCoordinates) {
        if (!targetMarker) return;
        
        // Positionner le marqueur
        targetMarker.style.left = `${mapCoordinates.x}px`;
        targetMarker.style.top = `${mapCoordinates.y}px`;
        targetMarker.style.display = 'block';
        
        // Arrêter le clignotement précédent si existant
        stopMarkerBlinking();
    }
    
    /**
     * Démarre le clignotement du marqueur
     */
    function startMarkerBlinking() {
        // Arrêter le clignotement précédent si existant
        stopMarkerBlinking();
        
        // Démarrer le nouvel intervalle de clignotement (2Hz = 500ms)
        let visible = true;
        blinkInterval = setInterval(function() {
            if (!targetMarker) return;
            
            visible = !visible;
            targetMarker.style.opacity = visible ? '1' : '0.3';
        }, 500);
    }
    
    /**
     * Arrête le clignotement du marqueur
     */
    function stopMarkerBlinking() {
        if (blinkInterval) {
            clearInterval(blinkInterval);
            blinkInterval = null;
            
            // Réinitialiser l'opacité
            if (targetMarker) {
                targetMarker.style.opacity = '1';
            }
        }
    }
    
    /**
     * Cache le marqueur de cible
     */
    function hideTargetMarker() {
        if (!targetMarker) return;
        
        // Arrêter le clignotement
        stopMarkerBlinking();
        
        // Cacher le marqueur
        targetMarker.style.display = 'none';
    }
    
    /**
     * Démarre la navigation vers la cible
     */
    function startNavigation() {
        // Vérifier qu'une cible est définie
        if (!targetPoint) return;
        
        // Changer l'état
        navigationState = 'MOVING';
        
        // Démarrer la vérification d'obstacles
        startObstacleCheck();
        
        // Démarrer le mouvement
        if (moveInterval) {
            clearInterval(moveInterval);
        }
        moveInterval = setInterval(moveToTarget, 100);
    }
    
    /**
     * Déplace le robot vers la cible
     */
    function moveToTarget() {
        // Vérifier l'état de navigation
        if (navigationState !== 'MOVING' && navigationState !== 'PAUSED') return;
        
        // Vérifier que la position du robot est disponible
        if (!window.PositionDisplay || !window.PositionDisplay.isAvailable()) {
            console.warn("Position du robot non disponible");
            return;
        }
        
        // Récupérer la position actuelle du robot
        const robotX = parseFloat(document.getElementById('position-x-value').textContent);
        const robotY = parseFloat(document.getElementById('position-y-value').textContent);
        const robotTheta = parseFloat(document.getElementById('position-theta-radians').textContent || '0');
        
        if (isNaN(robotX) || isNaN(robotY) || isNaN(robotTheta)) {
            console.warn("Coordonnées du robot invalides");
            return;
        }
        
        // Calculer la distance à la cible
        const dx = targetPoint.x - robotX;
        const dy = targetPoint.y - robotY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // Vérifier si la cible est atteinte
        if (distance < TARGET_THRESHOLD) {
            stopNavigation();
            return;
        }
        
        // Calculer l'angle vers la cible
        const targetAngle = Math.atan2(dy, dx);
        
        // Calculer la différence d'angle (tenir compte des angles cycliques)
        let angleDiff = targetAngle - robotTheta;
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
        
        // Déterminer si le robot est en phase de rotation
        const isRotating = Math.abs(angleDiff) > ANGLE_THRESHOLD;
        
        // Définir les vitesses
        let linearVel = 0;
        let angularVel = 0;
        
        // Si l'angle n'est pas correct, tourner d'abord
        if (isRotating) {
            // Contrôle proportionnel pour la rotation
            const kp = 1.0; // Gain proportionnel
            angularVel = Math.min(ANGULAR_SPEED, Math.max(-ANGULAR_SPEED, angleDiff * kp));
            
            // Toujours permettre la rotation, même en cas d'obstacle
            // La vitesse linéaire reste à 0 pendant la rotation
        } else {
            // L'angle est correct, on peut avancer
            // Sauf si on est en pause à cause d'un obstacle
            if (navigationState !== 'PAUSED') {
                // Contrôle proportionnel pour la vitesse linéaire
                const kpLinear = 1.0; // Gain proportionnel pour la vitesse linéaire
                linearVel = Math.min(LINEAR_SPEED, distance * kpLinear);
                
                // Petite correction d'angle pendant le mouvement
                const kp = 0.3; // Gain proportionnel plus faible pendant le mouvement
                angularVel = angleDiff * kp;
            }
        }
        
        // Envoyer la commande de vitesse en utilisant le même format que velocity-control.js
        if (cmdVelTopic && RobotApp.isConnected) {
            const twistStamped = new ROSLIB.Message({
                twist: {
                    linear: { x: linearVel, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: angularVel }
                }
            });
            
            cmdVelTopic.publish(twistStamped);
        }
    }


    /**
     * Démarre la vérification périodique des obstacles
     */
    function startObstacleCheck() {
        // Arrêter la vérification précédente si existante
        stopObstacleCheck();
        
        // Démarrer la nouvelle vérification (10Hz)
        obstacleCheckInterval = setInterval(checkObstacles, 100);
    }
    
    /**
     * Arrête la vérification périodique des obstacles
     */
    function stopObstacleCheck() {
        if (obstacleCheckInterval) {
            clearInterval(obstacleCheckInterval);
            obstacleCheckInterval = null;
        }
    }
    
    /**
     * Vérifie la présence d'obstacles dans le champ de vision actuel du robot
     */
    function checkObstacles() {
        // Vérifications préliminaires
        if (navigationState !== 'MOVING' && navigationState !== 'PAUSED') return;
        if (!lidarData?.length) return;

        // Configuration de la plage de détection (rotation de 90° vers la gauche)
        const PLAGE_TOTALE = Math.PI/2;  // 90 degrés
        const DEMI_PLAGE = PLAGE_TOTALE/2;
        const DECALAGE_ANGLE = -Math.PI/2; // Rotation de 90° vers la gauche
        const DIST_MIN = 0.2;
        const DIST_MAX = 0.5;

        let detection = false;
        let plusProche = Infinity;

        // Analyse des points Lidar avec rotation de plage
        lidarData.forEach(point => {
            // Calcul d'angle avec rotation de référence
            let angle = Math.atan2(point.y, point.x) + DECALAGE_ANGLE;
            
            // Normalisation angulaire
            while(angle > Math.PI) angle -= 2*Math.PI;
            while(angle < -Math.PI) angle += 2*Math.PI;

            // Vérification dans la plage frontale ajustée
            if(Math.abs(angle) <= DEMI_PLAGE) {
                const dist = Math.hypot(point.x, point.y);
                
                if(dist > DIST_MIN && dist <= DIST_MAX) {
                    detection = true;
                    plusProche = Math.min(plusProche, dist);
                }
            }
        });

        // Gestion d'état intégrée
        if(detection) {
            if(navigationState === 'MOVING') {
                // Mettre la navigation en pause
                navigationState = 'PAUSED';
                
                // Arrêt immédiat du robot
                if(cmdVelTopic?.isAdvertised) {
                    cmdVelTopic.publish(new ROSLIB.Message({
                        twist: { linear: {x:0}, angular: {z:0} }
                    }));
                }
                
                // Démarrer le clignotement du marqueur
                startMarkerBlinking();
            }
        } else {
            if(navigationState === 'PAUSED') {
                // Reprendre la navigation
                navigationState = 'MOVING';
                
                // Arrêter le clignotement du marqueur
                stopMarkerBlinking();
            }
        }
    }

    /**
     * Met en pause la navigation à cause d'un obstacle
     */
    function pauseNavigation() {
        // Changer l'état
        navigationState = 'PAUSED';
        
        // Arrêter le robot
        if (cmdVelTopic && RobotApp.isConnected) {
            const twistStamped = new ROSLIB.Message({
                twist: {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 }
                }
            });
            
            cmdVelTopic.publish(twistStamped);
        }
        
        // Démarrer le clignotement du marqueur
        startMarkerBlinking();
    }
    
    /**
     * Reprend la navigation après disparition de l'obstacle
     */
    function resumeNavigation() {
            // Changer l'état
            navigationState = 'MOVING';
            
            // Arrêter le clignotement du marqueur
            stopMarkerBlinking();
    }
        
    /**
     * Arrête complètement la navigation
     */
    function stopNavigation() {
            // Arrêter les intervalles
            if (moveInterval) {
                clearInterval(moveInterval);
                moveInterval = null;
            }
            
            // Arrêter la vérification d'obstacles
            stopObstacleCheck();

            // Arrêter le clignotement du marqueur
            stopMarkerBlinking();
            
            // Cacher le marqueur de cible
            hideTargetMarker();

            // Arrêter le robot
            if (cmdVelTopic && RobotApp.isConnected) {
                const twistStamped = new ROSLIB.Message({
                    twist: {
                        linear: { x: 0, y: 0, z: 0 },
                        angular: { x: 0, y: 0, z: 0 }
                    }
                });
                
                // S'assurer que la commande d'arrêt est envoyée
                cmdVelTopic.publish(twistStamped);
                
                // Envoyer une seconde commande d'arrêt après un court délai pour garantir l'arrêt complet
                setTimeout(() => {
                    if (RobotApp.isConnected) {
                        cmdVelTopic.publish(twistStamped);
                    }
                }, 50);
            }
            
            // Réinitialiser la cible et l'état
            targetPoint = null;
            navigationState = 'IDLE';
            
            // Informer les autres modules que la navigation est arrêtée
            document.dispatchEvent(new CustomEvent('semiauto-nav-stopped'));
        }

    /**
     * Gère les changements de mode d'affichage (lidar/map)
     * @param {CustomEvent} event - Événement avec le détail du mode
     */
    function handleDisplayModeChange(event) {
        const mode = event.detail.mode;
        
        if (mode !== 'map') {
            // Si on quitte le mode carte, on ne stoppe plus la navigation
            // On masque simplement le marqueur de cible temporairement
            if (targetMarker) {
                targetMarker.style.display = 'none';
            }
        } else {
            // Si on revient au mode carte et qu'une navigation est active
            if (navigationState !== 'IDLE' && targetPoint) {
                // Réafficher le marqueur de cible
                if (targetMarker) {
                    // Obtenir les coordonnées sur la carte (via le module MapModule)
                    if (window.MapModule && window.MapModule.isCalibrated() && 
                        typeof window.MapModule.worldToMapCoordinates === 'function') {
                        const mapCoordinates = window.MapModule.worldToMapCoordinates(targetPoint);
                        if (mapCoordinates) {
                            targetMarker.style.left = `${mapCoordinates.x}px`;
                            targetMarker.style.top = `${mapCoordinates.y}px`;
                            targetMarker.style.display = 'block';
                            
                            // Si la navigation est en pause, réactiver le clignotement
                            if (navigationState === 'PAUSED') {
                                startMarkerBlinking();
                            }
                        }
                    }
                }
            }
        }
    }

        
    /**
     * Gère les événements clavier
     * @param {KeyboardEvent} event - Événement clavier
     */
    function handleKeyDown(event) {
        // Si une touche de mouvement est pressée et la navigation est active, arrêter la navigation
        const key = event.key.toLowerCase();
        if ((key === 'j' || key === 'k' || key === 'l') && navigationState !== 'IDLE') {
            stopNavigation();
        }
    }
        
    /**
     * Nettoie les ressources du module
     */
    function cleanup() {
        // Arrêter la navigation
        stopNavigation();
            
        // Supprimer le marqueur de cible s'il existe
        if (targetMarker && targetMarker.parentNode) {
            targetMarker.parentNode.removeChild(targetMarker);
            targetMarker = null;
        }
            
        // Réinitialiser les variables
        cmdVelTopic = null;
        lidarData = [];
    }
        
        // Écouter l'événement d'initialisation de l'application
        document.addEventListener('app-init', init);
        
        // Exposer certaines fonctions pour les autres modules
        window.SemiAutoNav = {
            start: startNavigation,
            stop: stopNavigation,
            isActive: function() { return navigationState !== 'IDLE'; }
        };
    })();
    