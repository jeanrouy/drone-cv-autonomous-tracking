#!/usr/bin/env python3

# Importation de la bibliothèque cliente ROS 2 pour Python (rclpy)
import rclpy
from rclpy.node import Node

# Importation du type de message standard texte (String)
from std_msgs.msg import String

# On crée une classe qui hérite de la classe 'Node' de ROS 2
class MinimalPublisher(Node):

    def __init__(self):
        # 1. On initialise le nœud en lui donnant un nom ("minimal_publisher")
        super().__init__('minimal_publisher')
        
        # 2. On crée un Publisher qui va publier des messages de type String 
        # sur le topic nommé 'chatter'. Le 10 est la taille de la file d'attente (QoS).
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # 3. On crée un timer qui va exécuter la fonction 'timer_callback' 
        # toutes les 1.0 secondes
        timer_period = 1.0  # en secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Compteur pour incrémenter nos messages
        self.i = 0

    def timer_callback(self):
        # Création de l'objet message
        msg = String()
        msg.data = f'Drone prêt pour la mission : message n°{self.i}'
        
        # Publication du message sur le topic 'chatter'
        self.publisher_.publish(msg)
        
        # Affichage d'un log propre dans le terminal
        self.get_logger().info(f'Publication : "{msg.data}"')
        
        # Incrémentation du compteur
        self.i += 1


def main(args=None):
    # Initialisation de la communication ROS 2
    rclpy.init(args=args)

    # Instanciation de notre nœud
    minimal_publisher = MinimalPublisher()

    # Maintient le nœud en exécution pour écouter les événements et exécuter les timers
    rclpy.spin(minimal_publisher)

    # Nettoyage et fermeture propre à l'arrêt du programme (Ctrl+C)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
