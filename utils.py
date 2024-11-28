import time
from djitellopy import Tello
import cv2
import numpy as np
import mediapipe as mp

def initializeTello(host, vsport):
    drone = Tello(host=host, vsport=vsport)
    drone.connect()
    drone.for_back_velocity = 0
    drone.left_right_velocity = 0
    drone.up_down_velocity = 0
    drone.yaw_velocity = 0
    drone.speed = 0
    print(drone.get_battery())
    drone.streamoff()
    drone.streamon()
    return drone

def telloGetFrame(drone, w, h):
    frame_read = drone.get_frame_read()
    frame = frame_read.frame
    frame = cv2.resize(frame, (w, h))
    return frame

def findFace(img):
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.1, 4)
    myFaceListC = []
    myFaceListArea = []

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        myFaceListArea.append(area)
        myFaceListC.append([cx, cy])
    
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]
    

def trackFace(drone, info, w, h, pid, errors, target='center'):

    speeds = [0, 0, 0]  # [yaw_speed, up_down_speed, forward_speed]

    # Adjust yaw target based on the desired position
    if target == 'center':
        target_x = w // 2
        target_z = 4000
    elif target == 'left':
        target_x = w // 3  # One-third from the left
        target_z = 5000
    elif target == 'right':
        target_x = 2 * w // 3  # Two-thirds from the left
        target_z = 5000

    # Yaw adjustment (horizontal alignment)
    yaw_e = info[0][0] - target_x 
    speeds[0] = pid[0] * errors[0] + pid[1] * (yaw_e - errors[0])
    speeds[0] = int(np.clip(speeds[0], -100, 100))

    # Up/Down adjustment (vertical alignment)
    alt_e = h // 2 - info[0][1]
    speeds[1] = pid[0] * errors[1] + pid[1] * (alt_e - errors[1])
    speeds[1] = int(np.clip(speeds[1], -100, 100))

    # Forward/Backward adjustment (distance control)
    dist_e = target_z - info[1]  
    speeds[2] = pid[0] * errors[2] + pid[1] * (dist_e - errors[2])
    speeds[2] = int(np.clip(speeds[2], -20, 20))
    print(info[1])

    # If a face is detected, move the drone
    if info[0][0] != 0:
        drone.yaw_velocity = speeds[0]
        drone.up_down_velocity = speeds[1]
        drone.for_back_velocity = speeds[2]
        print(f"Speeds: {speeds}, Target: {target}")
    else:  # No face detected, hover
        drone.for_back_velocity = 0
        drone.left_right_velocity = 0
        drone.up_down_velocity = 0
        drone.yaw_velocity = 0
        errors = [0, 0, 0]

    if drone.send_rc_control:
        drone.send_rc_control(drone.left_right_velocity, 
                              drone.for_back_velocity, 
                              drone.up_down_velocity, 
                              drone.yaw_velocity)

    return [yaw_e, alt_e, dist_e]


def assign_primary(drones, frames, primary_drone, primary_found_times):

    # If primary is already assigned, skip role assignment
    if primary_drone is not None:
        return primary_drone, [i for i in range(len(drones)) if i != primary_drone]

    # Detect faces in all frames and track which drone sees a face
    detected_faces = []
    for i, frame in enumerate(frames):
        _, info = findFace(frame)
        if info[1] > 0:  # Face detected
            detected_faces.append((i, info))

    if detected_faces:
        # Sort drones by face area (largest area gets priority)
        detected_faces.sort(key=lambda x: x[1][1], reverse=True)
        
        for drone_index, _ in detected_faces:
            # Start or update the countdown for each drone
            if primary_found_times[drone_index] is None:
                primary_found_times[drone_index] = time.time()
            else:
                # If the drone holds the face for 1 second, assign it as primary
                if time.time() - primary_found_times[drone_index] >= 1:
                    primary_drone = drone_index
                    print(f"Primary drone assigned: Drone {primary_drone + 1}")
                    break
    else:
        # Reset the found times if no faces are detected
        primary_found_times = [None] * len(drones)

    # Return the primary and secondary roles
    return primary_drone, [i for i in range(len(drones)) if i != primary_drone], primary_found_times


mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

def findFaceMediaPipe(img):
    """Detect faces in the frame using MediaPipe and choose the one with the best confidence."""
    with mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5) as face_detection:
        # Convert the image to RGB as MediaPipe works with RGB images
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = face_detection.process(img_rgb)

        best_face = None
        best_confidence = 0

        if results.detections:
            # Loop through all detected faces
            for detection in results.detections:
                # Get the confidence score
                confidence = detection.score[0]

                if confidence > best_confidence:
                    # Update the best face if confidence is higher
                    best_confidence = confidence
                    best_face = detection

            if best_face:
                # Extract bounding box center and area for the best face
                bboxC = best_face.location_data.relative_bounding_box
                h, w, _ = img.shape
                cx = int(bboxC.xmin * w + bboxC.width * w / 2)
                cy = int(bboxC.ymin * h + bboxC.height * h / 2)
                area = int(bboxC.width * w * bboxC.height * h)
                cv2.rectangle(img, (int(bboxC.xmin * w), int(bboxC.ymin * h)), (int((bboxC.xmin + bboxC.width) * w), int((bboxC.ymin + bboxC.height) * h)), (255, 255, 0), 2)
                cv2.putText(img, f'Area: {area}', (cx - 50, cy - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                cv2.putText(img, f'Confidence: {best_confidence}', (cx - 50, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                return img, [[cx, cy], area]  # Return the face with the highest confidence

        # If no face is detected, return default values
        return img, [[0, 0], 0]
