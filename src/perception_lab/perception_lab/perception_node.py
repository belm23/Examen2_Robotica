import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import os
import math
import numpy as np

# MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

GESTURE_FORWARD = "ADELANTE"
GESTURE_BACKWARD = "RETROCEDER"
GESTURE_LEFT = "IZQUIERDA"
GESTURE_RIGHT = "DERECHA"
GESTURE_STOP = "ALTO"

def angle(ax, ay, bx, by, cx, cy):
    AB = (ax - bx, ay - by)
    CB = (cx - bx, cy - by)
    dot = AB[0]*CB[0] + AB[1]*CB[1]
    magAB = math.sqrt(AB[0]**2 + AB[1]**2)
    magCB = math.sqrt(CB[0]**2 + CB[1]**2)
    if magAB * magCB == 0:
        return 0
    ang = math.degrees(math.acos(max(min(dot / (magAB * magCB), 1), -1)))
    return ang

class PosePerceptionNode(Node):
    def __init__(self, use_camera=True):
        super().__init__('pose_perception_node')
        self.bridge = CvBridge()

        self.use_camera = use_camera

        self.publisher = self.create_publisher(String,'/gesture_command',10)

        self.pose = mp_pose.Pose(min_detection_confidence=0.5,min_tracking_confidence=0.5)

        self.current_gesture = GESTURE_STOP
        self.last_gesture_raw = GESTURE_STOP
        self.gesture_counter = 0
        self.STABLE_FRAMES = 8
        self.last_command = "NONE"

        # Valores de calibracion
        self.camera_matrix = np.array([[854.2431733156817, 0.0, 247.14339779690025],
                                       [0.0, 860.568593102158, 303.8783018958221],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeff = np.array([[-0.2351345774023858, 0.9547019296977259,
                                     0.01088010710128529, 0.00018455359154599085,
                                     -3.5799500027328435]])

        if self.use_camera:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Eror")

        if not self.use_camera:self.subscription = self.create_subscription(Image, '/kinect/image_raw', self.image_callback, 10)

    def process_frame(self, frame):
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeff, (w,h), 1, (w,h))
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeff, None, newcameramtx)

        image_rgb = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        frame_draw = frame.copy()

        if results.pose_landmarks:
            raw_gesture = self.classify_gesture_raw(results.pose_landmarks)
            if raw_gesture == self.last_gesture_raw:
                self.gesture_counter += 1
            else:
                self.gesture_counter = 0
            self.last_gesture_raw = raw_gesture

            if self.gesture_counter >= self.STABLE_FRAMES:
                self.current_gesture = raw_gesture
                if self.current_gesture != self.last_command:
                    msg_out = String()
                    msg_out.data = self.current_gesture
                    self.publisher.publish(msg_out)
                    self.last_command = self.current_gesture

            cv2.putText(frame_draw, f"{self.current_gesture}", (20,50),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 4)

        return frame_draw

    def classify_gesture_raw(self, lm):
        ls = lm.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
        rs = lm.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        le = lm.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
        re = lm.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
        lw = lm.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
        rw = lm.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]

        angle_left = angle(lw.x, lw.y, le.x, le.y, ls.x, ls.y)
        angle_right = angle(rw.x, rw.y, re.x, re.y, rs.x, rs.y)

        extended_left = angle_left > 150
        extended_right = angle_right > 150

        if lw.y < ls.y - 0.05 and rw.y < rs.y - 0.05:
            return GESTURE_FORWARD
        if lw.y > ls.y + 0.15 and rw.y > rs.y + 0.15:
            return GESTURE_BACKWARD
        if extended_left and not extended_right:
            return GESTURE_LEFT
        if extended_right and not extended_left:
            return GESTURE_RIGHT
        if extended_left and extended_right:
            if abs(lw.y - ls.y) < 0.15 and abs(rw.y - rs.y) < 0.15:
                return GESTURE_STOP

        return self.current_gesture

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_draw = self.process_frame(cv_image)
            if "DISPLAY" in os.environ:
                cv2.imshow("Pose Detection", frame_draw)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def spin_camera(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue
            frame_draw = self.process_frame(frame)
            if "DISPLAY" in os.environ:
                cv2.imshow("Pose Detection", frame_draw)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    # Cambia use_camera=True para usar la camara, False para usar el video
    node = PosePerceptionNode(use_camera=False)
    try:
        if node.use_camera:
            node.spin_camera()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pose.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
