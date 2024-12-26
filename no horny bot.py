
#track body and if hand detected track hand, each have their own centroid, hand still deploy taser

import cv2
import mediapipe as mp
import serial
import time

# Initialize Mediapipe Pose and Hands
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Initialize serial communication
try:
    arduino = serial.Serial(port='COM13', baudrate=9600, timeout=1)  # Replace COM13 with your Arduino port
    time.sleep(2)  # Allow time for the connection to establish
    print("Connected to Arduino")
except:
    arduino = None
    print("Arduino not connected. Running in simulation mode.")

# Video capture
cap = cv2.VideoCapture(0)


def pixel_to_angle(pixel, frame_width):    return int((pixel / frame_width) * 180)


def calculate_offset(pixel, frame_width):
    center = frame_width // 2
    threshold = 20  # Pixel threshold for "centered"
    if abs(pixel - center) <= threshold:
        return None  # Stop updating if centered
    return pixel_to_angle(pixel, frame_width)


# Variables for smoothing movement
current_angle = 90  # Initial angle (servo centered)
target_angle = 90
increment = 2
delay = 0.02  # Reduced delay for faster updates

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Adjust frame orientation
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.flip(frame, 1)

    h, w, _ = frame.shape
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process pose (body tracking) and hands
    pose_results = pose.process(frame_rgb)
    hand_results = hands.process(frame_rgb)

    body_detected = False
    hand_detected = False
    controlling_angle = None  # Track which is controlling the servo

    # Hand detection logic
    if hand_results.multi_hand_landmarks:
        hand_detected = True
        hand_landmarks = hand_results.multi_hand_landmarks[0]
        hand_center_x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * w)

        new_angle = calculate_offset(hand_center_x, w)
        if new_angle is not None:
            target_angle = new_angle

        # Draw the hand centroid
        cv2.circle(frame, (hand_center_x, h // 2), 5, (255, 0, 0), -1)
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        controlling_angle = "hand"

    # Body tracking logic
    if not hand_detected and pose_results.pose_landmarks:
        body_detected = True
        left_hip_x = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].x * w
        right_hip_x = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].x * w
        body_center_x = int((left_hip_x + right_hip_x) / 2)  # Midpoint of the hips

        new_angle = calculate_offset(body_center_x, w)
        if new_angle is not None:
            target_angle = new_angle

        # Draw the body centroid
        cv2.circle(frame, (body_center_x, h // 2), 5, (0, 0, 255), -1)
        mp_draw.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        controlling_angle = "body"

    # Gradually move current_angle toward target_angle
    if current_angle < target_angle:
        current_angle += increment
        if current_angle > target_angle:
            current_angle = target_angle
    elif current_angle > target_angle:
        current_angle -= increment
        if current_angle < target_angle:
            current_angle = target_angle

    # Send servo angle updates
    if controlling_angle == "hand":
        if arduino:
            arduino.write(f"H1\n".encode())  # Signal to activate hand-related components
            arduino.write(f"B{current_angle}\n".encode())
        print(f"Hand Detected: Current Angle: {current_angle}, Target Angle: {target_angle}")
    elif controlling_angle == "body":
        if arduino:
            arduino.write(f"H0\n".encode())  # Signal to deactivate hand-related components
            arduino.write(f"B{current_angle}\n".encode())
        print(f"Body Detected: Current Angle: {current_angle}, Target Angle: {target_angle}")
    else:
        # If neither body nor hand is detected, reset to center
        target_angle = 90
        if arduino:
            arduino.write(f"H0\n".encode())  # Deactivate hand-related components
            arduino.write(f"B{target_angle}\n".encode())
        print("No body or hand detected: Resetting body servo to 90°")

    time.sleep(delay)  # Reduced delay for faster updates

    cv2.imshow("Body and Hand Tracking with Priority", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # Press Esc to exit
        break

cap.release()
cv2.destroyAllWindows()

if arduino:
    arduino.close()
