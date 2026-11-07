import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2

class DroneCameraNode(Node):
    def __init__(self):
        super().__init__('drone_camera_node')
        
        # Utilisation du profil QoS standard pour les caméras sous ROS 2 (Best Effort)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.get_logger().info("Nœud Caméra ROS 2 initialisé avec succès ! En attente du flux...")

    def image_callback(self, msg):
        self.get_logger().info("Image reçue avec succès !")
        try:
            # Conversion du message ROS 2 en image OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Affichage de la fenêtre vidéo
            cv2.imshow("Flux Caméra Drone", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Erreur de conversion : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DroneCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()