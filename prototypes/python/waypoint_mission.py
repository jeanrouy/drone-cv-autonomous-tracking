#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import des types de messages PX4
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class WaypointMission(Node):

    def __init__(self):
        super().__init__('waypoint_mission')

        # Configuration QoS conforme à PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. PUBLISHERS (Commandes vers PX4)
        self.offboard_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        # 2. SUBSCRIBER (Télémétrie en retour depuis PX4)
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            qos_profile
        )

        # 3. VARIABLES DE MISSION (Repère local NED)
        # Remarque : Z est négatif pour s'élever (ex: -2.0m = 2m de hauteur)
        self.waypoints = [
            [0.0, 0.0, -2.0],   # WP 1 : Décollage / Point initial (2m de hauteur)
            [5.0, 0.0, -2.0],   # WP 2 : Avancer de 5m vers le Nord
            [5.0, 5.0, -2.0],   # WP 3 : Aller à 5m à l'Est
            [0.0, 5.0, -2.0],   # WP 4 : Revenir vers le Sud
            [0.0, 0.0, -2.0]    # WP 5 : Retour au-dessus du point de départ
        ]

        self.current_wp_index = 0
        self.current_position = [0.0, 0.0, 0.0]
        self.acceptance_radius = 0.5  # Valide le waypoint si on est à moins de 50 cm
        self.offboard_setpoint_counter = 0
        self.mission_state = "INITIALIZING"

        # Timer d'exécution à 10 Hz (0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        """Met à jour les coordonnées temps réel du drone."""
        self.current_position = [msg.x, msg.y, msg.z]

    def timer_callback(self):
        # Étape A : Publier le signal de vie continu (exigé par PX4 pour garder l'Offboard active)
        self.publish_offboard_control_mode()

        # Étape B : Séquence d'activation automatique (Passe outre la console pxh)
        if self.offboard_setpoint_counter == 10:
            # 1. Demander à PX4 de basculer en mode Offboard
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            # 2. Envoyer l'ordre d'armement des moteurs
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.mission_state = "EXECUTING"
            self.get_logger().info("Mode Offboard activé et Drone Armé via ROS 2 !")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # Étape C : Gestion de la trajectoire
        if self.mission_state == "EXECUTING":
            # Si tous les waypoints sont franchis -> Déclencher l'atterrissage
            if self.current_wp_index >= len(self.waypoints):
                self.get_logger().info("Dernier waypoint atteint ! Ordre d'atterrissage...")
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.mission_state = "LANDING"
                return

            target_wp = self.waypoints[self.current_wp_index]

            # Calcul de la distance euclidienne 3D jusqu'à la cible
            dist = math.sqrt(
                (target_wp[0] - self.current_position[0])**2 +
                (target_wp[1] - self.current_position[1])**2 +
                (target_wp[2] - self.current_position[2])**2
            )

            # Passage au waypoint suivant dès qu'on entre dans le rayon d'acceptation
            if dist < self.acceptance_radius:
                self.get_logger().info(f"Waypoint {self.current_wp_index + 1} atteint !")
                self.current_wp_index += 1
            else:
                # Envoyer la consigne physique du waypoint actuel
                self.publish_trajectory_setpoint(target_wp)

    def publish_offboard_control_mode(self):
        """Contrat de mode : On indique qu'on régule la POSITION."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, waypoint):
        """Consigne physique [X, Y, Z] vers PX4."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        msg.yaw = 0.0
        # On informe PX4 de ne pas réguler la vitesse/accélération directe (remplissage avec NaN)
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yawspeed = float('nan')
        self.trajectory_pub.publish(msg) if hasattr(self, 'trajectory_pub') else self.trajectory_setpoint_publisher.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Méthode générique pour envoyer des ordres de haut niveau à PX4."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
