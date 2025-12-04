/**
 * map.js
 * Module de gestion de la carte statique avec affichage de la position du robot
 */

(function() {
    // Variables locales du module
    let mapContainer;
    let isMapLoaded = false;
    let resizeTimeout;
    let robotMarker = null;
    let updatePositionInterval = null;
    
    // Variables pour la calibration
    let calibrationState = {
        isCalibrating: false,
        step: 0, // 0: pas de calibration, 1-3: étapes de calibration
        worldPoints: [], // Points dans le repère du robot
        mapPoints: [],   // Points correspondants sur la carte
        transformMatrix: null // Matrice de transformation calculée
    };
    
    // Éléments UI pour la calibration
    let calibrationButton;
    let calibrationMessage;
    let calibrationPoints = [];
    
    /**
     * Initialisation du module
     */
    function init() {
        // Récupération du conteneur de la carte
        mapContainer = document.querySelector('.map-container');
        
        if (!mapContainer) {
            console.error("Élément conteneur de carte non trouvé");
            return;
        }
        
        // Initialiser les éléments UI
        initializeUI();
        
        // Charger la carte
        loadMap();
        
        // Écoute des événements
        document.addEventListener('app-cleanup', cleanup);
        document.addEventListener('ros-disconnected', pausePositionUpdates);
        document.addEventListener('ros-connected', resumePositionUpdates);
        
        // Écouter les changements de mode d'affichage (lidar/map)
        document.addEventListener('display-mode-changed', handleDisplayModeChange);
        
        console.log("Module carte initialisé");
        
        // Restaurer la calibration précédente si disponible
        restoreCalibration();
    }
    
    /**
     * Initialise les éléments d'interface utilisateur
     */
    function initializeUI() {
        // Créer le marqueur de position du robot
        robotMarker = document.createElement('div');
        robotMarker.className = 'robot-position-marker';
        robotMarker.style.display = 'none'; // Caché jusqu'à la calibration
        mapContainer.appendChild(robotMarker);
        
        // Créer le bouton de calibration
        calibrationButton = document.createElement('button');
        calibrationButton.className = 'calibration-button';
        calibrationButton.textContent = 'Calibrer';
        // Empêcher la propagation de l'événement pour éviter qu'il soit capturé comme point de calibration
        calibrationButton.addEventListener('click', function(event) {
            event.stopPropagation(); // Empêcher la propagation vers le conteneur de carte
            startCalibration();
        });
        mapContainer.appendChild(calibrationButton);
        
        // Créer le message de calibration
        calibrationMessage = document.createElement('div');
        calibrationMessage.className = 'calibration-message';
        mapContainer.appendChild(calibrationMessage);
        
        // Créer les points de calibration (3 points)
        for (let i = 0; i < 3; i++) {
            const point = document.createElement('div');
            point.className = 'calibration-point';
            mapContainer.appendChild(point);
            calibrationPoints.push(point);
        }
        
        // Ajouter un gestionnaire d'événements pour les clics sur la carte
        mapContainer.addEventListener('click', handleMapClick);
    }
    
    /**
     * Charge la carte
     */
    function loadMap() {
        // Déterminer quelle carte utiliser
        const testMode = window.location.search.includes('test');
        const mapImage = document.getElementById('staticMap');
        
        // Définir la source de l'image en fonction du mode
        mapImage.src = testMode ? './map/my_map.png' : './map/SB.png';
        
        // Créer une image temporaire pour vérifier le chargement
        const tempImage = new Image();
        tempImage.onload = function() {
            isMapLoaded = true;
            console.log("Carte chargée avec succès");
            // Ajuster la taille après le chargement
            adjustMapSize();
        };
        tempImage.onerror = function() {
            console.error("Échec du chargement de la carte");
        };
        
        // Charger l'image depuis le même chemin que celui défini pour mapImage
        tempImage.src = mapImage.src;
        
        // Ajuster la taille initiale
        adjustMapSize();
        
        // Ajouter un écouteur pour le redimensionnement de la fenêtre avec debounce
        window.addEventListener('resize', function() {
            clearTimeout(resizeTimeout);
            resizeTimeout = setTimeout(adjustMapSize, 200);
        });
    }
    
    /**
     * Ajuste la taille de la carte en fonction de l'espace disponible
     */
    function adjustMapSize() {
        const availableWidth = window.innerWidth - 640 - 60;
        const maxWidth = Math.max(200, availableWidth);
        mapContainer.style.maxWidth = `${maxWidth}px`;
    }
    
    /**
     * Gère les clics sur la carte
     * @param {MouseEvent} event - L'événement de clic
     */
    function handleMapClick(event) {
        // Vérifier si on est en mode carte et si la carte est chargée
        if (!isMapLoaded || window.LidarModule.getCurrentMode() !== 'map') return;
        
        // Si on clique sur le bouton de calibration, ne pas traiter comme point de calibration
        if (event.target === calibrationButton) return;
        
        // Traiter uniquement si en mode calibration
        if (calibrationState.isCalibrating && calibrationState.step > 0) {
            // Empêcher tout comportement par défaut
            event.preventDefault();
            
            // Récupérer les coordonnées relatives à l'élément
            const rect = mapContainer.getBoundingClientRect();
            const x = event.clientX - rect.left;
            const y = event.clientY - rect.top;
            
            // Vérifier si nous avons les coordonnées du robot
            if (window.PositionDisplay && window.PositionDisplay.isAvailable()) {
                const robotX = parseFloat(document.getElementById('position-x-value').textContent);
                const robotY = parseFloat(document.getElementById('position-y-value').textContent);
                
                if (!isNaN(robotX) && !isNaN(robotY)) {
                    // Enregistrer le point du monde (robot)
                    calibrationState.worldPoints[calibrationState.step - 1] = { x: robotX, y: robotY };
                    
                    // Enregistrer le point de la carte
                    calibrationState.mapPoints[calibrationState.step - 1] = { x, y };
                    
                    // Afficher le point de calibration
                    const pointElement = calibrationPoints[calibrationState.step - 1];
                    pointElement.style.left = `${x}px`;
                    pointElement.style.top = `${y}px`;
                    pointElement.style.display = 'block';
                    
                    // Passer à l'étape suivante
                    calibrationState.step++;
                    
                    // Si c'est le dernier point, finaliser la calibration
                    if (calibrationState.step > 3) {
                        finalizeCalibration();
                    } else {
                        // Mettre à jour le message
                        updateCalibrationMessage();
                    }
                } else {
                    alert("Impossible de lire les coordonnées du robot. Vérifiez la connexion ROS.");
                }
            } else {
                alert("Module de position non disponible. Vérifiez la connexion ROS.");
            }
        }
        
        // Si nous ne sommes pas en mode calibration, traiter comme point de navigation
        if (!calibrationState.isCalibrating && window.PositionDisplay && window.PositionDisplay.isAvailable()) {
            // Empêcher tout comportement par défaut
            event.preventDefault();
            
            // Récupérer les coordonnées relatives à l'élément
            const rect = mapContainer.getBoundingClientRect();
            const x = event.clientX - rect.left;
            const y = event.clientY - rect.top;
            
            // Récupérer les coordonnées du robot
            const robotX = parseFloat(document.getElementById('position-x-value').textContent);
            const robotY = parseFloat(document.getElementById('position-y-value').textContent);
            
            if (isNaN(robotX) || isNaN(robotY)) {
                console.warn("Coordonnées du robot invalides");
                return;
            }
            
            // Convertir les coordonnées de la carte en coordonnées du monde
            // Nous avons besoin d'une fonction inverse de worldToMapCoordinates
            const worldCoordinates = mapToWorldCoordinates({ x, y });
            
            if (!worldCoordinates) {
                console.warn("Impossible de convertir les coordonnées");
                return;
            }
            
            // Déclencher un événement pour informer le module de navigation
            document.dispatchEvent(new CustomEvent('navigation-target-set', {
                detail: {
                    coordinates: worldCoordinates,
                    mapCoordinates: { x, y }
                }
            }));
        }
    }
    /**
     * Démarre le processus de calibration
     */
    function startCalibration() {

        localStorage.removeItem('mapCalibration');
        sessionStorage.removeItem('mapCalibration');

        // Réinitialiser l'état de calibration
        calibrationState.isCalibrating = true;
        calibrationState.step = 1;
        calibrationState.worldPoints = [];
        calibrationState.mapPoints = [];
        calibrationState.transformMatrix = null;
        
        // Cacher le marqueur de position pendant la calibration
        robotMarker.style.display = 'none';
        
        // Réinitialiser les points de calibration
        calibrationPoints.forEach(point => {
            point.style.display = 'none';
        });
        
        // Afficher le message de calibration
        updateCalibrationMessage();
        
        // Pause des mises à jour de position pendant la calibration
        pausePositionUpdates();
        
    }
    
    /**
     * Met à jour le message de calibration
     */
    function updateCalibrationMessage() {
        calibrationMessage.textContent = `Cliquez sur le point ${calibrationState.step}/3 sur la carte`;
        calibrationMessage.style.display = 'block';
    }
    
    /**
     * Finalise la calibration et calcule la matrice de transformation
     */
    function finalizeCalibration() {
        // Vérifier que nous avons tous les points nécessaires
        if (calibrationState.worldPoints.length < 3 || calibrationState.mapPoints.length < 3) {
            console.error("Données de calibration incomplètes");
            resetCalibration();
            return;
        }
        
        // Calculer la transformation à partir des trois points
        calculateTransformation();
        
        // Terminer la calibration
        calibrationState.isCalibrating = false;
        calibrationState.step = 0;
        
        // Cacher le message de calibration
        calibrationMessage.style.display = 'none';
        
        // Sauvegarder la calibration
        saveCalibration();
        
        // Afficher le marqueur de position
        robotMarker.style.display = 'block';
        
        // Reprendre les mises à jour de position
        resumePositionUpdates();
        
    }
    
    /**
     * Calcule la transformation à partir des points de calibration
     */
    function calculateTransformation() {
        // Nous utilisons trois points pour calculer une transformation affine
        const w1 = calibrationState.worldPoints[0];
        const w2 = calibrationState.worldPoints[1];
        const w3 = calibrationState.worldPoints[2];
        
        const m1 = calibrationState.mapPoints[0];
        const m2 = calibrationState.mapPoints[1];
        const m3 = calibrationState.mapPoints[2];
        
        // Calculer les coefficients de la transformation affine
        // Pour la transformation: mapX = a*worldX + b*worldY + c, mapY = d*worldX + e*worldY + f
        
        // Système d'équations pour x
        const matA = [
            [w1.x, w1.y, 1, 0, 0, 0],
            [w2.x, w2.y, 1, 0, 0, 0],
            [w3.x, w3.y, 1, 0, 0, 0],
            [0, 0, 0, w1.x, w1.y, 1],
            [0, 0, 0, w2.x, w2.y, 1],
            [0, 0, 0, w3.x, w3.y, 1]
        ];
        
        const vecB = [m1.x, m2.x, m3.x, m1.y, m2.y, m3.y];
        
        // Résoudre le système d'équations
        try {
            const coefficients = solveSystem(matA, vecB);
            
            // Stocker les coefficients dans la matrice de transformation
            calibrationState.transformMatrix = {
                a: coefficients[0],
                b: coefficients[1],
                c: coefficients[2],
                d: coefficients[3],
                e: coefficients[4],
                f: coefficients[5]
            };
            
        } catch (error) {
            console.error("Erreur lors du calcul de la transformation:", error);
            alert("Erreur de calibration. Veuillez réessayer avec des points plus éloignés.");
            resetCalibration();
        }
    }
    
    /**
     * Résout un système d'équations linéaires par élimination de Gauss
     * @param {Array} A - Matrice des coefficients
     * @param {Array} b - Vecteur des constantes
     * @returns {Array} - Solution du système
     */
    function solveSystem(A, b) {
        const n = b.length;
        const augmentedMatrix = A.map((row, i) => [...row, b[i]]);
        
        // Élimination de Gauss
        for (let i = 0; i < n; i++) {
            // Recherche du pivot
            let maxRow = i;
            for (let j = i + 1; j < n; j++) {
                if (Math.abs(augmentedMatrix[j][i]) > Math.abs(augmentedMatrix[maxRow][i])) {
                    maxRow = j;
                }
            }
            
            // Échange des lignes
            [augmentedMatrix[i], augmentedMatrix[maxRow]] = [augmentedMatrix[maxRow], augmentedMatrix[i]];
            
            // Élimination
            for (let j = i + 1; j < n; j++) {
                const factor = augmentedMatrix[j][i] / augmentedMatrix[i][i];
                for (let k = i; k <= n; k++) {
                    augmentedMatrix[j][k] -= factor * augmentedMatrix[i][k];
                }
            }
        }
        
        // Substitution arrière
        const x = new Array(n).fill(0);
        for (let i = n - 1; i >= 0; i--) {
            let sum = 0;
            for (let j = i + 1; j < n; j++) {
                sum += augmentedMatrix[i][j] * x[j];
            }
            x[i] = (augmentedMatrix[i][n] - sum) / augmentedMatrix[i][i];
        }
        
        return x;
    }
    
    /**
     * Réinitialise l'état de calibration
     */
    function resetCalibration() {
        calibrationState.isCalibrating = false;
        calibrationState.step = 0;
        calibrationState.worldPoints = [];
        calibrationState.mapPoints = [];
        
        // Cacher les points de calibration
        calibrationPoints.forEach(point => {
            point.style.display = 'none';
        });
        
        // Cacher le message
        calibrationMessage.style.display = 'none';
        
        // Si pas de transformation valide, cacher le marqueur
        if (!calibrationState.transformMatrix) {
            robotMarker.style.display = 'none';
        }
        
        // Reprendre les mises à jour si une transformation existe
        if (calibrationState.transformMatrix) {
            resumePositionUpdates();
        }
    }
    
    /**
     * Convertit les coordonnées du monde en coordonnées de la carte
     * @param {Object} worldPoint - Point dans le repère du robot {x, y}
     * @returns {Object} - Point dans le repère de la carte {x, y}
     */
    function worldToMapCoordinates(worldPoint) {
        if (!calibrationState.transformMatrix) return null;
        
        const { a, b, c, d, e, f } = calibrationState.transformMatrix;
        
        return {
            x: a * worldPoint.x + b * worldPoint.y + c,
            y: d * worldPoint.x + e * worldPoint.y + f
        };
    }
    
    /**
     * Met à jour la position du robot sur la carte
     */
    function updateRobotPosition() {
        // Vérifier si nous avons une calibration valide
        if (!calibrationState.transformMatrix) return;
        
        // Vérifier si le module de position est disponible
        if (!window.PositionDisplay || !window.PositionDisplay.isAvailable()) return;
        
        // Récupérer les coordonnées actuelles du robot
        const robotX = parseFloat(document.getElementById('position-x-value').textContent);
        const robotY = parseFloat(document.getElementById('position-y-value').textContent);
        
        if (isNaN(robotX) || isNaN(robotY)) return;
        
        // Convertir les coordonnées du monde en coordonnées de la carte
        const mapCoordinates = worldToMapCoordinates({ x: robotX, y: robotY });
        
        if (!mapCoordinates) return;
        
        // Mettre à jour la position du marqueur
        robotMarker.style.left = `${mapCoordinates.x}px`;
        robotMarker.style.top = `${mapCoordinates.y}px`;
        
        // S'assurer que le marqueur est visible
        robotMarker.style.display = 'block';
    }
    
    /**
     * Démarre les mises à jour périodiques de la position
     */
    function startPositionUpdates() {
        // Arrêter les mises à jour existantes
        stopPositionUpdates();
        
        // Démarrer les mises à jour périodiques
        updatePositionInterval = setInterval(updateRobotPosition, 500);
        
    }
    
    /**
     * Arrête les mises à jour périodiques de la position
     */
    function stopPositionUpdates() {
        if (updatePositionInterval) {
            clearInterval(updatePositionInterval);
            updatePositionInterval = null;
        }
    }
    
    /**
     * Met en pause les mises à jour de position
     */
    function pausePositionUpdates() {
        stopPositionUpdates();
    }
    
    /**
     * Reprend les mises à jour de position
     */
    function resumePositionUpdates() {
        // Ne reprendre que si nous avons une calibration valide
        if (calibrationState.transformMatrix) {
            startPositionUpdates();
        }
    }
    
    /**
     * Gère les changements de mode d'affichage (lidar/map)
     * @param {CustomEvent} event - Événement avec le détail du mode
     */
    function handleDisplayModeChange(event) {
        const mode = event.detail.mode;
        
        if (mode === 'map') {
            // Si on passe en mode carte
            if (calibrationState.isCalibrating) {
                // Si en cours de calibration, restaurer l'état de calibration
                updateCalibrationMessage();
                
                // Afficher les points de calibration déjà placés
                for (let i = 0; i < calibrationState.step - 1; i++) {
                    if (i < calibrationPoints.length) {
                        calibrationPoints[i].style.display = 'block';
                    }
                }
            } else if (calibrationState.transformMatrix) {
                // Si déjà calibré, reprendre les mises à jour
                resumePositionUpdates();
            }
        } else {
            // Si on passe en mode lidar
            pausePositionUpdates();
            
            // Cacher temporairement les éléments de calibration en mode lidar
            if (calibrationState.isCalibrating) {
                calibrationMessage.style.display = 'none';
            }
        }
    }
    
    /**
     * Sauvegarde l'état de calibration dans localStorage
     */
    function saveCalibration() {
        if (calibrationState.transformMatrix) {
            // Créer un objet avec les données de calibration et des métadonnées
            const calibrationData = {
                transformMatrix: calibrationState.transformMatrix,
                worldPoints: calibrationState.worldPoints,
                mapPoints: calibrationState.mapPoints,
                mapName: document.getElementById('staticMap').src.split('/').pop(), // Nom de la carte
                timestamp: new Date().toISOString() // Horodatage
            };
            
            // Sauvegarder dans localStorage et sessionStorage
            localStorage.setItem('mapCalibration', JSON.stringify(calibrationData));
            sessionStorage.setItem('mapCalibration', JSON.stringify(calibrationData));
            console.log("Calibration sauvegardée dans localStorage");
        }
    }
    
    /**
     * Restaure l'état de calibration depuis localStorage
     */
    function restoreCalibration() {
        try {
            // Essayer d'abord sessionStorage (pour la session courante)
            const sessionCalibration = sessionStorage.getItem('mapCalibration');
            if (sessionCalibration) {
                const parsedCalibration = JSON.parse(sessionCalibration);
                applyCalibration(parsedCalibration);
                console.log("Calibration restaurée depuis sessionStorage");
                return true;
            }
            
            // Si pas dans sessionStorage, essayer localStorage
            const savedCalibration = localStorage.getItem('mapCalibration');
            if (savedCalibration) {
                const parsedCalibration = JSON.parse(savedCalibration);
                
                // Vérifier si la calibration correspond à la carte actuelle
                const currentMapName = document.getElementById('staticMap').src.split('/').pop();
                if (parsedCalibration.mapName && parsedCalibration.mapName === currentMapName) {
                    applyCalibration(parsedCalibration);
                    
                    // Copier aussi dans sessionStorage pour un accès plus rapide
                    sessionStorage.setItem('mapCalibration', savedCalibration);
                    
                    console.log("Calibration restaurée depuis localStorage");
                    return true;
                } else {
                    console.log("La calibration sauvegardée correspond à une autre carte");
                }
            }
        } catch (error) {
            console.error("Erreur lors de la restauration de la calibration:", error);
        }
        
        return false;
    }

    /**
     * Applique les données de calibration
     * @param {Object} calibrationData - Données de calibration
     */
    function applyCalibration(calibrationData) {
        // Restaurer la matrice de transformation
        calibrationState.transformMatrix = calibrationData.transformMatrix;
        calibrationState.worldPoints = calibrationData.worldPoints || [];
        calibrationState.mapPoints = calibrationData.mapPoints || [];
        
        // Restaurer les points de calibration visuels
        if (calibrationState.mapPoints.length > 0) {
            calibrationState.mapPoints.forEach((point, index) => {
                if (index < calibrationPoints.length) {
                    const pointElement = calibrationPoints[index];
                    pointElement.style.left = `${point.x}px`;
                    pointElement.style.top = `${point.y}px`;
                    pointElement.style.display = 'block';
                }
            });
        }
        
        // Afficher le marqueur de position
        robotMarker.style.display = 'block';
        
        // Démarrer les mises à jour de position si en mode carte
        if (window.LidarModule && window.LidarModule.getCurrentMode() === 'map') {
            startPositionUpdates();
        }
    }

    /**
     * Convertit les coordonnées de la carte en coordonnées du monde
     * @param {Object} mapPoint - Point dans le repère de la carte {x, y}
     * @returns {Object} - Point dans le repère du robot {x, y}
     */
    function mapToWorldCoordinates(mapPoint) {
        if (!calibrationState.transformMatrix) return null;
        
        const { a, b, c, d, e, f } = calibrationState.transformMatrix;
        
        // Résoudre le système d'équations inverse:
        // mapX = a*worldX + b*worldY + c
        // mapY = d*worldX + e*worldY + f
        
        // Calculer le déterminant
        const det = a*e - b*d;
        
        if (Math.abs(det) < 1e-6) {
            console.error("Matrice de transformation non inversible");
            return null;
        }
        
        // Calculer les coordonnées du monde
        const worldX = (e*(mapPoint.x - c) - b*(mapPoint.y - f)) / det;
        const worldY = (a*(mapPoint.y - f) - d*(mapPoint.x - c)) / det;
        
        return { x: worldX, y: worldY };
    }
    
    /**
     * Nettoie les ressources du module
     */
    function cleanup() {
        // Arrêter les mises à jour de position
        stopPositionUpdates();
        
        // Supprimer les écouteurs d'événements
        if (mapContainer) {
            mapContainer.removeEventListener('click', handleMapClick);
        }
        
        if (calibrationButton) {
            calibrationButton.removeEventListener('click', startCalibration);
        }
        
        // Supprimer l'écouteur de redimensionnement
        window.removeEventListener('resize', function() {
            clearTimeout(resizeTimeout);
        });
        
        // Réinitialiser les variables
        clearTimeout(resizeTimeout);
        
        console.log("Nettoyage du module carte");
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Exposer certaines fonctions pour les autres modules
    window.MapModule = {
        isLoaded: function() { return isMapLoaded; },
        startCalibration: startCalibration,
        resetCalibration: resetCalibration,
        worldToMapCoordinates: worldToMapCoordinates,
        mapToWorldCoordinates: mapToWorldCoordinates,
        isCalibrated: function() { return !!calibrationState.transformMatrix; }
    };
})();
