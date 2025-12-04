/**
 * camera.js
 * Module de gestion de la caméra et du flux vidéo
 * Ce module gère l'affichage des images de la caméra sur le canvas
 */

(function() {
    // Variables locales du module
    let canvas; // Élément canvas pour l'affichage
    let ctx; // Contexte 2D du canvas
    let cameraImg; // Objet Image pour le chargement des images
    let isProcessingImage = false; // Indicateur d'état de traitement de l'image
    let latestImageMessage = null; // Dernier message d'image reçu
    let isSubscribed = false; // État d'abonnement au topic

    /**
     * Initialisation du module
     */
    function init() {
        // Récupération du canvas et de son contexte
        canvas = document.getElementById('cameraFeed');
        if (!canvas) {
            console.error("Élément canvas 'cameraFeed' non trouvé");
            return;
        }
        
        ctx = canvas.getContext('2d');
        
        // Création d'un objet Image réutilisable
        cameraImg = new Image();
        
        // Écoute des événements ROS
        document.addEventListener('ros-connected', initializeCamera);
        document.addEventListener('ros-disconnected', cleanupCamera);
        document.addEventListener('ros-error', cleanupCamera);
        document.addEventListener('app-cleanup', cleanupCamera);
        
        console.log("Module caméra initialisé");
    }

    /**
     * Initialiser la connexion à la caméra après connexion ROS
     */
    function initializeCamera() {
        // Ne rien faire si déjà abonné
        if (isSubscribed) return;
        
        // Vérifier que ROS est connecté
        if (!RobotApp.ros || !RobotApp.isConnected) {
            console.warn("Tentative d'initialisation de la caméra sans connexion ROS");
            return;
        }
        
        // Créer le topic pour les images
        RobotApp.topics.image = new ROSLIB.Topic({
            ros: RobotApp.ros,
            name: '/oakd/rgb/preview/image_raw/compressed',
            messageType: 'sensor_msgs/msg/CompressedImage'
        });
        
        // S'abonner aux messages d'image
        RobotApp.topics.image.subscribe(function(message) {
            // Mettre à jour avec le dernier message
            latestImageMessage = message;
            
            // Si aucune image n'est en cours de traitement, traiter ce dernier message
            if (!isProcessingImage) {
                processLatestImage();
            }
        });
        
        isSubscribed = true;
        console.log("Abonnement au flux de la caméra réussi");
    }

    /**
     * Traiter le dernier message d'image reçu
     */
    function processLatestImage() {
        // S'il n'y a pas de message, quitter
        if (!latestImageMessage) return;
        
        // Marquer comme en cours de traitement
        isProcessingImage = true;
        
        // Obtenir le dernier message et le réinitialiser
        const messageToProcess = latestImageMessage;
        latestImageMessage = null;
        
        // Configurer les gestionnaires d'événements pour l'objet Image
        cameraImg.onload = function() {
            // Effacer le canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Calculer les dimensions pour maintenir le ratio d'aspect
            const imgRatio = cameraImg.width / cameraImg.height;
            const canvasRatio = canvas.width / canvas.height;
            
            let drawWidth, drawHeight, offsetX, offsetY;
            
            if (imgRatio > canvasRatio) {
                // L'image est plus large que le canvas
                drawWidth = canvas.width;
                drawHeight = canvas.width / imgRatio;
                offsetX = 0;
                offsetY = (canvas.height - drawHeight) / 2;
            } else {
                // L'image est plus haute que le canvas
                drawHeight = canvas.height;
                drawWidth = canvas.height * imgRatio;
                offsetX = (canvas.width - drawWidth) / 2;
                offsetY = 0;
            }
            
            // Remplir le fond en noir
            ctx.fillStyle = 'black';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Dessiner la nouvelle image en préservant le ratio d'aspect
            ctx.drawImage(cameraImg, offsetX, offsetY, drawWidth, drawHeight);
            
            // Traitement terminé
            isProcessingImage = false;
            
            // Si un nouveau message est reçu pendant le traitement, continuer
            if (latestImageMessage) {
                processLatestImage();
            }
        };
        
        cameraImg.onerror = function(e) {
            console.error("Échec du chargement de l'image", e);
            isProcessingImage = false;
            
            // Même en cas d'échec, essayer de traiter le message suivant
            if (latestImageMessage) {
                processLatestImage();
            }
        };
        
        // Définir la source de l'image (format base64)
        cameraImg.src = "data:image/jpeg;base64," + messageToProcess.data;
    }
    

    /**
     * Nettoyer les ressources de la caméra
     */
    function cleanupCamera() {
        // Se désabonner du topic d'image
        if (RobotApp.topics.image) {
            RobotApp.topics.image.unsubscribe();
            RobotApp.topics.image = null;
            isSubscribed = false;
            console.log("Désabonnement du flux de la caméra");
        }
        
        // Réinitialiser l'état de traitement
        isProcessingImage = false;
        latestImageMessage = null;
        
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
    window.CameraModule = {
        reinitialize: initializeCamera,
        cleanup: cleanupCamera
    };
})();
