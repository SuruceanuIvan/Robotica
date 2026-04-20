#!/usr/bin/env python

# importam rospy pentru a putea folosi ROS in python
import rospy
# avem nevoie de math pentru functiile isnan si isinf
import math
# tipul de mesaj primit de la senzorul laser
from sensor_msgs.msg import LaserScan


# clasa care se ocupa cu citirea si afisarea datelor de la laser
class LaserMonitor:
    def __init__(sp):
        # pornim nodul ROS cu numele simple_subscriber
        rospy.init_node('simple_subscriber')

        # ne abonam la topicul /scan unde laserul publica datele
        sp.sub = rospy.Subscriber('/scan', LaserScan, sp.laser_callback)

        # initializam distantele cu 10 metri ca valoare default
        sp.front_distance = 10.0  # distanta masurata in fata
        sp.right_distance = 10.0  # distanta masurata in dreapta
        sp.left_distance = 10.0   # distanta masurata in stanga

    # functie ajutatoare care extrage distanta la un unghi specific din mesajul laser
    def get_range_at_angle(sp, ranges, angle, angle_min, angle_increment):
        # calculam pozitia in lista de distante care corespunde unghiului dorit
        sp.index = int((angle - angle_min) / angle_increment)
        # ne asiguram ca nu depasim limitele listei
        sp.index = max(0, min(len(ranges) - 1, sp.index))
        sp.val = ranges[sp.index]
        # valorile nan, infinit sau zero inseamna ca senzorul nu a citit nimic valid
        if math.isnan(sp.val) or math.isinf(sp.val) or sp.val == 0.0:
            return 10.0
        return sp.val

    # callback apelat automat la fiecare mesaj nou de la laser
    def laser_callback(sp, msg):
        # extragem lista de distante si parametrii de unghi
        sp.ranges = msg.ranges
        sp.angle_min = msg.angle_min
        sp.angle_increment = msg.angle_increment

        # calculam distantele pentru cele 3 directii importante
        sp.front_distance = sp.get_range_at_angle(sp.ranges, 0.0, sp.angle_min, sp.angle_increment)
        sp.right_distance = sp.get_range_at_angle(sp.ranges, -math.pi / 2, sp.angle_min, sp.angle_increment)
        sp.left_distance = sp.get_range_at_angle(sp.ranges, math.pi / 2, sp.angle_min, sp.angle_increment)

        # afisam valorile in consola ca sa vedem ce citeste laserul
        rospy.loginfo(
            "Fata: %.2fm | Dreapta: %.2fm | Stanga: %.2fm",
            sp.front_distance,
            sp.right_distance,
            sp.left_distance,
        )


# instantiem clasa si o numim sp asa cum cere cerintele
sp = LaserMonitor()
# mentinem nodul activ ca sa primim in continuare date
rospy.spin()
