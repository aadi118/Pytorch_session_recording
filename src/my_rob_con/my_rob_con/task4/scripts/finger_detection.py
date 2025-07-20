import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import mediapipe as mp

class DualHandFingerCommandNode(Node):
    def __init__(self):
        super().__init__('dual_hand_finger_command_node')  # Fixed constructor
        self.left_pub = self.create_publisher(String, 'left_hand_action', 10)
        self.right_pub = self.create_publisher(String, 'right_hand_action', 10)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2, min_detection_confidence=0.7)
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.detect_gestures)

    def detect_gestures(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(
                results.multi_hand_landmarks, results.multi_handedness):

                landmarks = [(int(lm.x * w), int(lm.y * h))
                             for lm in hand_landmarks.landmark]
                label = handedness.classification[0].label  # "Left" or "Right"

                # Count fingers
                count = 0
                # Thumb detection
                if label == "Right":
                    if landmarks[4][0] > landmarks[3][0]:
                        count += 1
                else:
                    if landmarks[4][0] < landmarks[3][0]:
                        count += 1

                # Other 4 fingers
                for tip_id in [8, 12, 16, 20]:
                    if landmarks[tip_id][1] < landmarks[tip_id - 2][1]:
                        count += 1

                # Ignore 5 fingers
                if count == 5:
                    continue

                # Map finger count to action
                action = "unknown"
                if count == 0:
                    action = "stop"
                elif count == 1:
                    action = "move_forward"
                elif count == 2:
                    action = "turn_right"
                elif count == 3:
                    action = "turn_left"
                elif count == 4:
                    action = "move_backward"

                # Publish to corresponding topic
                msg = String()
                msg.data = action
                if label == "Left":
                    self.left_pub.publish(msg)
                    self.get_logger().info(f'Left hand: {action}')
                    cv2.putText(frame, f'Left: {action}', (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                else:
                    self.right_pub.publish(msg)
                    self.get_logger().info(f'Right hand: {action}')
                    cv2.putText(frame, f'Right: {action}', (350, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

                # Draw landmarks
                mp.solutions.drawing_utils.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Dual Hand Command Node", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # Check if the user pressed the Esc key
            self.cap.release()  # Release the camera
            cv2.destroyAllWindows()  # Close the OpenCV window
            rclpy.shutdown()  # Shutdown ROS

def main(args=None):
    rclpy.init(args=args)
    node = DualHandFingerCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
