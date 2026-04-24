import cv2
import numpy as np
import time

def detect_brick(frame):
    # 1. DEFINE REGION OF INTEREST (ROI)
    # This ignores the wood frame and table edge on the left/top.
    height, width = frame.shape[:2]
    roi_y1, roi_y2 = int(height * 0.2), int(height * 0.8)
    roi_x1, roi_x2 = int(width * 0.3), int(width * 0.9)
    roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

    # 2. GRAYSCALE & THRESHOLD
    # Bricks are bright gray; Mat is dark black.
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # We use a simple threshold: anything brighter than '120' becomes WHITE.
    # Adjust this '120' up if the floor is too bright, or down if bricks are missing.
    _, mask = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)

    # 3. CLEANUP (Remove small dots)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 4. FIND CONTOURS
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        # Area filter (roughly the size of your brick in pixels)
        if 10000 < area < 50000:
            # Get the Rotated Bounding Box
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # Re-offset to the full frame
            box[:, 0] += roi_x1
            box[:, 1] += roi_y1
            
            # Draw the Green Box and the Angle
            angle = int(rect[2])
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)
            cv2.putText(frame, f"ANGLE: {angle}deg", (box[0][0], box[0][1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
    # Draw the Blue ROI box for visual confirmation
    cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)
    
    return frame, mask

# --- MAIN LOOP ---
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
time.sleep(2)

while True:
    ret, frame = cap.read()
    if not ret: break
    
    processed_frame, debug_mask = detect_brick(frame)
    
    cv2.imshow('BIM Simple Reset', processed_frame)
    cv2.imshow('Debug Mask', debug_mask)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()