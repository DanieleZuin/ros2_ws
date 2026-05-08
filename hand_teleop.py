import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np
import math

class AdvancedHandController(Node):
    """
    ROS 2 Node that captures a video stream, processes hand gestures using MediaPipe,
    and translates them into kinematic Twist commands for robot teleoperation.
    """
    def __init__(self):
        super().__init__('advanced_hand_controller')
        
        # Publisher for the robot's velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize MediaPipe Hands module
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
            
        # Connect to the video stream (Host Mac acting as video server)
        self.cap = cv2.VideoCapture("http://192.168.1.214:5001/video")
        
        # Timer to trigger the callback at ~30 FPS (0.033 seconds)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Node Started! Open Hand = Throttle. Tilt Left/Right to steer.")

    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            return
            
        # Flip the image horizontally for a natural (mirror) selfie-view display
        image = cv2.flip(image, 1)
        h, w, c = image.shape
        
        # Convert the BGR image to RGB as required by MediaPipe
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        
        # Initialize a Twist message (default is 0.0 for all velocities)
        msg = Twist()
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                
                # EXTRACT 3 KEY LANDMARKS: Wrist(0), Middle Finger MCP/Knuckle(9), Middle Finger Tip(12)
                x_wrist, y_wrist = int(hand_landmarks.landmark[0].x * w), int(hand_landmarks.landmark[0].y * h)
                x_mcp, y_mcp = int(hand_landmarks.landmark[9].x * w), int(hand_landmarks.landmark[9].y * h)
                x_tip, y_tip = int(hand_landmarks.landmark[12].x * w), int(hand_landmarks.landmark[12].y * h)
                
                # --- 1. THROTTLE LOGIC (Hand Open/Closed) ---
                # Calculate the Euclidean distance between the wrist and the tip of the middle finger
                openness = math.hypot(x_tip - x_wrist, y_tip - y_wrist)
                
                # Map the distance: < 80px (Closed Fist) = 0.0 m/s | > 180px (Open Hand) = 1.0 m/s
                speed = np.interp(openness, [80, 180], [0.0, 1.0])
                
                # --- 2. STEERING LOGIC (Wrist Tilt) ---
                # Calculate the horizontal displacement between the wrist and the middle knuckle
                tilt = x_mcp - x_wrist 
                
                # Map the tilt: Left tilt (-60px) turns left (+1.5 rad/s) | Right tilt (+60px) turns right (-1.5 rad/s)
                steering = np.interp(tilt, [-60, 60], [1.5, -1.5])
                
                # --- DEADZONE ---
                # If the hand is relatively straight (small tilt), do not steer (prevents jittering)
                if abs(tilt) < 15:
                    steering = 0.0
                    
                # If the hand is closed (speed < 0.1), apply emergency brake (stop steering too)
                if speed < 0.1:
                    speed = 0.0
                    steering = 0.0

                # Apply calculated values to the ROS 2 Twist message
                msg.linear.x = float(speed)
                msg.angular.z = float(steering)
                
                # --- DEBUG UI (ON-SCREEN DISPLAY) ---
                # Draw a line from wrist to finger tip
                cv2.line(image, (x_wrist, y_wrist), (x_tip, y_tip), (255, 0, 0), 2)
                # Draw points on wrist and knuckle
                cv2.circle(image, (x_wrist, y_wrist), 5, (0, 0, 255), -1)
                cv2.circle(image, (x_mcp, y_mcp), 5, (0, 255, 0), -1)
                
                # Display Speed and Steering text
                cv2.putText(image, f"THROTTLE: {speed:.1f} m/s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(image, f"STEER: {steering:.1f} rad/s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 165, 0), 2)
                
                # Visual Throttle Bar
                cv2.rectangle(image, (20, 100), (20 + int(speed*200), 120), (0, 255, 0), -1)

                # Draw the standard MediaPipe hand skeleton
                self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        else:
            # If no hand is detected, ensure the robot stops completely
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            cv2.putText(image, "HAND NOT DETECTED - STOP", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Publish the Twist message to ROS 2
        self.publisher_.publish(msg)
        
        # Show the final debug frame
        cv2.imshow('ROS 2 AI Cockpit', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedHandController()
    try:
        rclpy.spin(node) # Keep the node running and listening
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources before shutting down
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()