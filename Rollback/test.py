import cv2
import numpy as np
import os
import signal
import time
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# --- CONFIGURATION ---
cap = cv2.VideoCapture(0)

# COLORS: Red is kept perfect. Black tightened by 15% (lower Value cap).
COLORS = {
    'Red': [
        (np.array([0, 150, 100]), np.array([4, 255, 255])),
        (np.array([176, 150, 100]), np.array([180, 255, 255]))
    ],
    'Green': [
        (np.array([55, 150, 60]), np.array([75, 255, 255]))
    ],
    'Black': [
        # Tightened Black: Value cap dropped to 30 to only see deepest blacks
        (np.array([0, 0, 0]), np.array([180, 255, 30]))
    ]
}

detection_memory = {
    'Red':   {'box': None, 'frames': 0, 'consecutive': 0, 'saved': False},
    'Green': {'box': None, 'frames': 0, 'consecutive': 0, 'saved': False},
    'Black': {'box': None, 'frames': 0, 'consecutive': 0, 'saved': False}
}

def is_center_blocked(frame):
    """Detects obstructions in the center 50% of the frame."""
    try:
        h, w = frame.shape[:2]
        s_h, e_h, s_w, e_w = h // 4, 3 * h // 4, w // 4, 3 * w // 4
        roi = frame[s_h:e_h, s_w:e_w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Laplacian variance: Lower score = less detail (blocked/covered)
        score = cv2.Laplacian(gray, cv2.CV_64F).var()
        return score < 25, (s_w, s_h, e_w, e_h)
    except:
        return False, (0,0,0,0)

HTML = """
<html>
    <head><title>Robot Vision Pro</title>
    <style>
        body { background: #000; color: #0f0; font-family: monospace; text-align: center; margin: 0; }
        .container { display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; }
        img { border: 2px solid #222; max-width: 90%; box-shadow: 0 0 20px #040; }
        .hud { margin-bottom: 5px; font-size: 11px; letter-spacing: 1px; }
    </style></head>
    <body>
        <div class="container">
            <div class="hud">SCANNER ACTIVE // TIGHT_BLACK_MODE // OBSTACLE_ROI: 50%</div>
            <img src="/video_feed">
            <br><a href="/stop_server" style="color:#600; text-decoration:none; margin-top:10px; display:block;">[ TERMINATE SESSION ]</a>
        </div>
    </body>
</html>
"""

def generate_frames():
    global detection_memory
    while True:
        success, frame = cap.read()
        if not success or frame is None:
            continue
        
        try:
            # 1. OBSTACLE BLOCKING DETECTION & VISUAL FRAME
            blocked, roi = is_center_blocked(frame)
            # Always draw the detection frame (Dim Blue) so you know where it's looking
            cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (80, 0, 0), 1)
            
            if blocked:
                # Highlight red and alert if blocked
                cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 0, 255), 3)
                cv2.putText(frame, "! PATH OBSTRUCTED !", (roi[0], roi[1]-10), 1, 1, (0, 0, 255), 2)
                for col in detection_memory: detection_memory[col]['consecutive'] = 0
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            for color_name, ranges in COLORS.items():
                mask = None
                for (low, high) in ranges:
                    m = cv2.inRange(hsv, low, high)
                    mask = m if mask is None else cv2.bitwise_or(mask, m)
                
                mask = cv2.medianBlur(mask, 3) 
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                found_now = False
                for c in cnts:
                    area = cv2.contourArea(c)
                    
                    # Custom Area: Black targets can be tiny (100px), others (250px)
                    min_area = 100 if color_name == 'Black' else 250
                    
                    if min_area < area < 50000:
                        x, y, w, h = cv2.boundingRect(c)
                        aspect_ratio = float(w)/h
                        
                        # Finger filter: Ignore long thin shapes
                        if 0.4 < aspect_ratio < 2.5:
                            detection_memory[color_name]['box'] = (x, y, w, h)
                            detection_memory[color_name]['frames'] = 3
                            detection_memory[color_name]['consecutive'] += 1
                            found_now = True
                            break 
                
                if not found_now:
                    detection_memory[color_name]['consecutive'] = 0
                    detection_memory[color_name]['saved'] = False

                mem = detection_memory[color_name]
                if mem['frames'] > 0:
                    bx, by, bw, bh = mem['box']
                    b_col = (0, 255, 0) if color_name == 'Green' else (0, 0, 255)
                    if color_name == 'Black': b_col = (255, 255, 255)
                    
                    is_locked = mem['consecutive'] >= 10
                    cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), b_col, 2 if is_locked else 1)
                    cv2.putText(frame, f"{color_name} {'[LOCKED]' if is_locked else '...'}", (bx, by-5), 1, 0.7, b_col, 1)
                    
                    if is_locked and not mem['saved']:
                        cv2.imwrite(f"/home/admin/detect_{color_name}_{int(time.time())}.jpg", frame)
                        mem['saved'] = True
                    
                    if not found_now:
                        mem['frames'] -= 1

            ret, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except:
            continue

@app.route('/')
def index(): return render_template_string(HTML)

@app.route('/stop_server')
def stop_server():
    cap.release()
    os.kill(os.getpid(), signal.SIGINT)
    return "Disconnected."

@app.route('/video_feed')
def video_feed(): return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)
