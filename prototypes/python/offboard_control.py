#!/usr/bin/env python3

# Importation de la bibliothèque cliente ROS 2 pour Python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Importation des types de messages spécifiques à PX4
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint


class OffboardControl(Node):

    def __init__(self):
        # Initialisation du nœud sous le nom 'offboard_control'
        super().__init__('offboard_control')

        # 1. Configuration de la Qualité de Service (QoS)
        # PX4 exige une politique de fiabilité "Best Effort" et de durabilité "Transient Local"
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. Création du Publisher pour activer/maintenir le mode Offboard
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        # 3. Création du Publisher pour envoyer la consigne de position/vitesse
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        # 4. Timer réglé à 10 Hz (0.1 seconde) pour publier en continu
        timer_period = 0.1  # secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Compteur de boucles pour suivre l'initialisation
        self.offboard_setpoint_counter_ = 0

    def timer_callback(self):
        # 1. On attend d'avoir envoyé 10 consignes (1 seconde à 10 Hz) 
        # avant d'activer le mode Offboard et d'armer le drone
        if self.offboard_setpoint_counter_ == 10:
            self.get_logger().info("Activation du mode Offboard et Armement...")

        # 2. Publication continue du signal de vie / contrat de mode
        self.publish_offboard_control_mode()

        # 3. Publication continue de la consigne de position
        self.publish_trajectory_setpoint()

        # Incrémentation du compteur
        if self.offboard_setpoint_counter_ < 11:
            self.offboard_setpoint_counter_ += 1

    def publish_offboard_control_mode(self):
        """Publie le message de configuration / signal de vie."""
        msg = OffboardControlMode()
        # On définit l'horodatage en microsecondes (exigé par PX4)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Le CONTRAT : On indique à PX4 d'écouter la POSITION
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publie la consigne physique réelle (Position XYZ)."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Repère local NED (North-East-Down) :
        # X = 0.0 m (Nord)
        # Y = 0.0 m (Est)
        # Z = -2.0 m (2 mètres AU-DESSUS du sol)
        msg.position = [0.0, 0.0, -2.0]
        msg.yaw = 0.0  # Cap orienté vers le Nord (0 radian)

        self.trajectory_setpoint_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
