/**
 * lidar.js
 * Module de gestion de l'affichage du lidar
 * Ce module gère l'affichage du point cloud du lidar et le basculement entre lidar et carte
 */

(function() {
    // Variables locales du module
    let canvas; // Élément canvas pour l'affichage du lidar
    let ctx; // Contexte 2D du canvas
    let isSubscribed = false; // État d'abonnement au topic
    let animationFrameId = null; // ID pour l'animation frame
    let displayMode = 'lidar'; // Mode d'affichage par défaut: 'lidar' ou 'map'
    let lidarContainer; // Conteneur pour la vue lidar/map
    
    // Variables pour les données lidar
    let lidarData = [];
    let maxRange = 4.0; // Portée maximale du lidar en mètres
    const pointSize = 1.5; //pix
    
    /**
     * Initialisation du module
     */
    function init() {
        // Récupération du canvas et de son contexte
        canvas = document.getElementById('lidarCanvas');
        if (!canvas) {
            console.error("Élément canvas 'lidarCanvas' non trouvé");
            return;
        }
        
        ctx = canvas.getContext('2d');
        
        // Récupération du conteneur lidar/map
        lidarContainer = document.querySelector('.lidar-map-container');
        if (!lidarContainer) {
            console.error("Conteneur lidar-map non trouvé");
            return;
        }
        
        // Configuration initiale en mode lidar
        setDisplayMode('lidar');
        
        // Écoute des événements ROS
        document.addEventListener('ros-connected', initializeLidar);
        document.addEventListener('ros-disconnected', cleanupLidar);
        document.addEventListener('ros-error', cleanupLidar);
        document.addEventListener('app-cleanup', cleanupLidar);
        
        // Écoute de l'événement clavier pour basculer entre lidar et carte
        document.addEventListener('keydown', handleKeyDown);
        
        console.log("Module lidar initialisé");
    }
    
    /**
     * Gère les événements clavier pour basculer entre lidar et carte
     * @param {KeyboardEvent} event - Événement clavier
     */
    function handleKeyDown(event) {
        // Vérifier si la touche M (majuscule ou minuscule) est pressée
        if (event.key.toLowerCase() === 'm') {
            // Basculer entre les modes
            toggleDisplayMode();
        }
        
    }
    
    /**
     * Bascule entre les modes d'affichage lidar et carte
     */
    function toggleDisplayMode() {
        const newMode = displayMode === 'lidar' ? 'map' : 'lidar';
        setDisplayMode(newMode);
    }
    
    /**
     * Définit le mode d'affichage
     * @param {string} mode - Mode d'affichage ('lidar' ou 'map')
     */
    function setDisplayMode(mode) {
        displayMode = mode;
        // Mettre à jour les classes CSS pour contrôler l'affichage
        if (mode === 'map') {
            lidarContainer.classList.add('show-map');
            pauseLidarRendering();
        } else {
            lidarContainer.classList.remove('show-map');
            resumeLidarRendering();
        }
        
        // Déclencher un événement pour informer les autres modules du changement de mode
        document.dispatchEvent(new CustomEvent('display-mode-changed', { 
            detail: { mode: mode } 
        }));
    }
    
    /**
     * Pause le rendu du lidar pour économiser les ressources
     */
    function pauseLidarRendering() {
        if (animationFrameId) {
            cancelAnimationFrame(animationFrameId);
            animationFrameId = null;
        }
    }
    
    /**
     * Reprend le rendu du lidar
     */
    function resumeLidarRendering() {
        if (!animationFrameId && isSubscribed) {
            renderLidar();
        }
    }
    
    /**
     * Initialiser la connexion au lidar après connexion ROS
     */
    function initializeLidar() {
        // Ne rien faire si déjà abonné
        if (isSubscribed) return;
        
        // Vérifier que ROS est connecté
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation du lidar sans connexion ROS");
            return;
        }
        
        // Créer le topic pour les données lidar
        RobotApp.topics.lidar = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/scan',
            messageType: 'sensor_msgs/msg/LaserScan'
        });
        
        // S'abonner aux messages lidar
        RobotApp.topics.lidar.subscribe(function(message) {
            // Traitement des données lidar
            processLidarData(message);
        });
        
        isSubscribed = true;
        
        if (displayMode === 'lidar') {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            drawReferenceCircles();
        }
        
        console.log("Abonnement au flux lidar réussi");
    }
    
    /**
     * Traite les données lidar reçues
     * @param {Object} message - Message ROS contenant les données lidar
     */
    function processLidarData(message) {
        // Extraire les données pertinentes du message
        const angleMin = message.angle_min;
        const angleIncrement = message.angle_increment;
        const ranges = message.ranges;
        
        // Réinitialiser les données
        lidarData = [];
        
        // Traiter chaque point
        for (let i = 0; i < ranges.length; i++) {
            const range = ranges[i];
            const angle = angleMin + (i * angleIncrement);
            const adjustedAngle = angle + Math.PI; 
            // Ignorer les valeurs invalides (inf, NaN, etc.)
            if (isFinite(range) && range > 0) {
                // Convertir en coordonnées cartésiennes
                const x = range * Math.cos(adjustedAngle);
                const y = range * Math.sin(adjustedAngle);
                
                // Ajouter le point aux données
                lidarData.push({ x, y, range });
            }
        }

        // Déclencher un événement pour partager les données lidar avec d'autres modules
        document.dispatchEvent(new CustomEvent('lidar-data-updated', {
            detail: {
                data: lidarData
            }
        }));

        if (displayMode === 'lidar') {
            renderLidar();// Mise à jour nuage des points
        }
    }
    
    /**
     * Rendu du lidar sur le canvas
     */
    function renderLidar() {
        // Effacer le canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Définir le style de fond
        ctx.fillStyle = 'black';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Dessiner les cercles de référence (échelle)
        drawReferenceCircles();
        
        // Dessiner les points lidar
        drawLidarPoints();
        
    }
    
    /**
     * Dessine les cercles de référence pour l'échelle
     */
    function drawReferenceCircles() {
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const pixelsPerMeter = canvas.width / (2 * maxRange);
        
        ctx.strokeStyle = '#333333';
        ctx.lineWidth = 1;
        
        // Dessiner les cercles à différentes distances
        for (let radius = 0.5; radius <= maxRange; radius += 0.5) {
            const pixelRadius = radius * pixelsPerMeter;
            
            ctx.beginPath();
            ctx.arc(centerX, centerY, pixelRadius, 0, 2 * Math.PI);
            ctx.stroke();
            
            // Ajouter l'étiquette de distance
            if (radius % 2 === 0) {
                ctx.fillStyle = '#888888';
                ctx.font = '10px Arial';
                ctx.fillText(`${radius}m`, centerX + 5, centerY - pixelRadius + 5);
            }
        }
        
        // Dessiner les axes
        ctx.beginPath();
        ctx.moveTo(0, centerY);
        ctx.lineTo(canvas.width, centerY);
        ctx.moveTo(centerX, 0);
        ctx.lineTo(centerX, canvas.height);
        ctx.stroke();
        
        // Arrow
        const arrowSize = 10;
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        // Arrow body
        ctx.moveTo(centerX, 10);
        // Arrow head
        ctx.moveTo(centerX, 10);
        ctx.lineTo(centerX - arrowSize/2, 20);
        ctx.moveTo(centerX, 10);
        ctx.lineTo(centerX + arrowSize/2, 20);
        ctx.stroke();
    }
    
    /**
     * Dessine les points du lidar
     */
    function drawLidarPoints() {
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const pixelsPerMeter = canvas.width / (2 * maxRange);
        
        ctx.fillStyle = 'rgba(0, 255, 0, 0.7)';

        lidarData.forEach(point => {
            const screenX = centerX + point.x * pixelsPerMeter;
            const screenY = centerY - point.y * pixelsPerMeter;
            
            ctx.beginPath();
            ctx.arc(screenX, screenY, pointSize, 0, 2 * Math.PI);
            ctx.fill();
        });
    }
    
    /**
     * Nettoyer les ressources du lidar
     */
    function cleanupLidar() {
        // Annuler l'animation
        if (animationFrameId) {
            cancelAnimationFrame(animationFrameId);
            animationFrameId = null;
        }
        
        // Se désabonner du topic lidar
        if (RobotApp.topics.lidar) {
            RobotApp.topics.lidar.unsubscribe();
            RobotApp.topics.lidar = null;
            isSubscribed = false;
            console.log("Désabonnement du flux lidar");
        }
        
        // Réinitialiser les données
        lidarData = [];
        
        // Effacer le canvas
        if (ctx && canvas) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = 'black';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
        }
    }
    
    // Écouter l'événement d'initialisation de l'application
    document.addEventListener('app-init', init);
    
    // Exposer certaines fonctions pour les autres modules
    window.LidarModule = {
        toggleDisplayMode: toggleDisplayMode,
        getCurrentMode: function() { return displayMode; }
    };
})();
