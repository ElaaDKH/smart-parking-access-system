from flask import Flask, jsonify, Response, render_template_string
import cv2
import numpy as np
import easyocr
from datetime import datetime
import os
import firebase_admin
from firebase_admin import credentials, db

app = Flask(__name__)

# Initialize EasyOCR (une seule fois au démarrage)
print("Loading EasyOCR...")
reader = easyocr.Reader(['en'], gpu=False)
print("EasyOCR ready!")

# Initialize Firebase
print("Initializing Firebase...")
cred = credentials.Certificate("C:/Users/hp/Downloads/esp32-cam-platenumberdetection-firebase-adminsdk-fbsvc-3f5035183a.json")  
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://esp32-cam-platenumberdetection-default-rtdb.europe-west1.firebasedatabase.app/'
})
print("Firebase connected!")

#Dossier pour sauvegarder les images
IMAGES_DIR = 'captured_images'
if not os.path.exists(IMAGES_DIR):
    os.makedirs(IMAGES_DIR)

#Initialiser la webcam
print("Initializing webcam...")
cap = cv2.VideoCapture(0)  # 0 = webcam par défaut (changez si nécessaire)

if not cap.isOpened():
    print("ERROR: Cannot open webcam!")
else:
    print("Webcam ready!")

#Variable pour stocker le dernier résultat OCR
last_ocr_result = {"plate": "", "authorized": False, "timestamp": ""}

#Template HTML pour la page de preview
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Webcam Preview - Parking System</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .container {
            background: white;
            border-radius: 15px;
            padding: 30px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            max-width: 900px;
            width: 100%;
        }
        h1 {
            color: #2a5298;
            text-align: center;
            margin-bottom: 30px;
        }
        #video-feed {
            width: 100%;
            border-radius: 10px;
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        .controls {
            margin-top: 20px;
            display: flex;
            gap: 10px;
            justify-content: center;
        }
        button {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 12px 30px;
            font-size: 16px;
            border-radius: 8px;
            cursor: pointer;
            transition: transform 0.2s;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        .result {
            margin-top: 20px;
            padding: 20px;
            background: #f8f9fa;
            border-radius: 10px;
            border-left: 5px solid #667eea;
        }
        .result h3 {
            margin-top: 0;
            color: #2a5298;
        }
        .authorized {
            color: #28a745;
            font-weight: bold;
        }
        .denied {
            color: #dc3545;
            font-weight: bold;
        }
        #status {
            text-align: center;
            margin-top: 10px;
            font-style: italic;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚗 Parking System - Webcam Preview</h1>
        <img id="video-feed" src="{{ url_for('video_feed') }}" alt="Webcam Feed">
        
        <div class="controls">
            <button onclick="captureAndOCR()">📷 Capturer et Analyser</button>
            <button onclick="refreshFeed()">🔄 Rafraîchir</button>
        </div>
        
        <div id="status"></div>
        
        <div id="result" class="result" style="display:none;">
            <h3>Résultat:</h3>
            <p><strong>Plaque détectée:</strong> <span id="plate"></span></p>
            <p><strong>Statut:</strong> <span id="auth-status"></span></p>
            <p><small>Timestamp: <span id="timestamp"></span></small></p>
        </div>
    </div>
    
    <script>
        function captureAndOCR() {
            document.getElementById('status').textContent = '⏳ Capture et analyse en cours...';
            
            fetch('/capture')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').textContent = '✅ Analyse terminée!';
                    
                    // Afficher résultat
                    const resultDiv = document.getElementById('result');
                    resultDiv.style.display = 'block';
                    
                    document.getElementById('plate').textContent = data.plate || 'Aucune plaque détectée';
                    
                    const authSpan = document.getElementById('auth-status');
                    if (data.authorized) {
                        authSpan.textContent = 'ACCÈS AUTORISÉ ✓';
                        authSpan.className = 'authorized';
                    } else {
                        authSpan.textContent = 'ACCÈS REFUSÉ ✗';
                        authSpan.className = 'denied';
                    }
                    
                    document.getElementById('timestamp').textContent = new Date().toLocaleString();
                })
                .catch(error => {
                    document.getElementById('status').textContent = '❌ Erreur: ' + error;
                    console.error('Error:', error);
                });
        }
        
        function refreshFeed() {
            const img = document.getElementById('video-feed');
            img.src = img.src.split('?')[0] + '?t=' + new Date().getTime();
        }
        
        // Auto-refresh every 100ms pour un flux fluide
        setInterval(refreshFeed, 100);
    </script>
</body>
</html>
"""

def generate_frames():
    """Générateur pour le flux vidéo"""
    while True:
        success, frame = cap.read()
        if not success:
            break
        
        #Encoder l'image en JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        #Format pour streaming MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Page d'accueil avec preview webcam"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    """Route pour le flux vidéo"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture', methods=['GET'])
def capture_and_ocr():
    print("\n=== CAPTURE REQUEST ===")
    
    try:
        #Capturer une image de la webcam
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to capture from webcam")
            return jsonify({'error': 'Camera capture failed', 'authorized': False}), 500
        
        print(f"Image captured: {frame.shape}")
        
        #Sauvegarder l'image originale
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        original_path = f"{IMAGES_DIR}/original_{timestamp}.jpg"
        cv2.imwrite(original_path, frame)
        print(f"Saved: {original_path}")
        
        #Prétraitement de l'image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        #Redimensionner pour améliorer l'OCR
        big = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        
        #Améliorer le contraste
        enhanced = cv2.equalizeHist(big)
        
        #Sauvegarder l'image traitée
        processed_path = f"{IMAGES_DIR}/processed_{timestamp}.jpg"
        cv2.imwrite(processed_path, enhanced)
        print(f"Saved processed: {processed_path}")
        
        #Exécuter l'OCR avec EasyOCR
        print("Running OCR...")
        results = reader.readtext(enhanced)
        
        #Combiner tout le texte détecté
        all_text = []
        for (bbox, text, prob) in results:
            print(f"  Detected: '{text}' (confidence: {prob:.2f})")
            all_text.append(text)
        
        #Nettoyer et formater le texte
        final_text = ''.join(all_text)
        final_text = final_text.upper()
        final_text = ''.join(c for c in final_text if c.isalnum())
        
        print(f"✓ Final plate: '{final_text}'")
        
        #Vérifier dans Firebase
        authorized = False
        if final_text:
            authorized = check_plate_in_firebase(final_text)
            print(f"Authorization check: {authorized}")
        
        #Sauvegarder le résultat
        with open(f"{IMAGES_DIR}/result_{timestamp}.txt", 'w') as f:
            f.write(f"Plate: {final_text}\n")
            f.write(f"Authorized: {authorized}\n\n")
            f.write("Raw OCR results:\n")
            for i, (bbox, text, prob) in enumerate(results):
                f.write(f"{i+1}. {text} ({prob:.2f})\n")
        
        #Retourner la réponse JSON
        return jsonify({
            'plate': final_text,
            'authorized': authorized,
            'success': True
        })
        
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({'error': str(e), 'authorized': False}), 500

def check_plate_in_firebase(plate):
    """Vérifie si la plaque est autorisée dans Firebase"""
    try:
        ref = db.reference(f'/authorized_plates/{plate}')
        value = ref.get()
        
        if value is None:
            print(f"Plate '{plate}' not found in database")
            return False
        
        if isinstance(value, bool):
            return value
        
        return False
        
    except Exception as e:
        print(f"Firebase error: {e}")
        return False

@app.route('/test', methods=['GET'])
def test():
    """Endpoint de test pour vérifier que le serveur fonctionne"""
    return jsonify({
        'status': 'Server is running',
        'camera': cap.isOpened(),
        'timestamp': datetime.now().isoformat()
    })

if __name__ == '__main__':
    print("\n" + "="*50)
    print("  PARKING SYSTEM - Python OCR Server")
    print("="*50)
    print(f"Server starting on http://0.0.0.0:{5000}")
    print("Endpoints:")
    print("  - GET /           : Webcam preview page")
    print("  - GET /video_feed : Video stream")
    print("  - GET /capture    : Capture image and run OCR")
    print("  - GET /test       : Test if server is running")
    print("\n📷 Open your browser: http://localhost:5000")
    print("="*50 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        #Libérer la webcam à la fermeture
        if cap.isOpened():
            cap.release()
            print("Webcam released")
