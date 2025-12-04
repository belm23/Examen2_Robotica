import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthHeatmapNode(Node):
    def __init__(self):
        super().__init__('depth_heatmap_node')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image,'/kinect/depth/image_raw',self.depth_callback,10)
        self.pub_zones = self.create_publisher(Float32MultiArray, '/proximity_zones', 10)
        self.get_logger().info("Nodo de ejercico extra")

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_m = depth_image / 1000.0   
        depth_m[depth_m == 0] = np.nan   

        h, w = depth_m.shape
        left_zone = depth_m[:, :w//3]
        center_zone = depth_m[:, w//3: 2*w//3]
        right_zone = depth_m[:, 2*w//3:]

        left_dist = np.nanmean(left_zone)
        center_dist = np.nanmean(center_zone)
        right_dist = np.nanmean(right_zone)

        msg_out = Float32MultiArray()
        msg_out.data = [left_dist, center_dist, right_dist]
        self.pub_zones.publish(msg_out)

        heatmap = cv2.normalize(depth_m, None, 0, 255, cv2.NORM_MINMAX)
        heatmap = np.uint8(heatmap)
        heatmap_color = cv2.applyColorMap(255 - heatmap, cv2.COLORMAP_JET)
        cv2.line(heatmap_color, (w//3,0), (w//3,h), (255,255,255), 2)
        cv2.line(heatmap_color, (2*w//3,0), (2*w//3,h), (255,255,255), 2)
        cv2.imshow("Mapa de calor", heatmap_color)
        cv2.waitKey(1)

        self.get_logger().info(f"distancias promedio - Izq: {left_dist:.2f}, Centro: {center_dist:.2f}, Der: {right_dist:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthHeatmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
