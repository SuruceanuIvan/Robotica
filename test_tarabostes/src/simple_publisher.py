#!/usr/bin/env python

# importam libraria rospy ca sa putem face noduri in ROS
import rospy
# avem nevoie de math pentru pi si pentru verificarea valorilor invalide
import math
# mesajul de la senzorul laser
from sensor_msgs.msg import LaserScan
# mesajul pentru a trimite comenzi de viteza la robot
from geometry_msgs.msg import Twist


# clasa principala care controleaza robotul in functie de ce citeste laserul
class RobotController:
    def __init__(sp):
        # initializam nodul ROS, il numim simple_publisher
        rospy.init_node('simple_publisher')

        # cream publisherul care trimite comenzi de miscare pe topicul /cmd_vel
        sp.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # cream subscriberul care asculta datele de la laser pe topicul /scan
        sp.sub = rospy.Subscriber('/scan', LaserScan, sp.laser_callback)

        # distanta initiala fata de obstacole - punem 10 metri ca sa nu se opreasca la start
        sp.front_distance = 10.0  # distanta in fata
        sp.right_distance = 10.0  # distanta in dreapta
        sp.left_distance = 10.0   # distanta in stanga

        # vitezele robotului, le-am ales asa ca sa nu mearga prea repede
        sp.linear_speed = 0.3   # viteza de mers inainte in m/s
        sp.angular_speed = 0.5  # viteza de rotatie in rad/s

    # functie care returneaza distanta citita de laser la un unghi dat
    # calculam indexul din mesaj in functie de unghi ca sa fie universal
    def get_range_at_angle(sp, ranges, angle, angle_min, angle_increment):
        # calculam indexul corespunzator unghiului dorit
        sp.index = int((angle - angle_min) / angle_increment)
        # ne asiguram ca indexul nu iese din lista
        sp.index = max(0, min(len(ranges) - 1, sp.index))
        sp.val = ranges[sp.index]
        # daca valoarea e invalida (nan, infinit sau zero) returnam 10 ca sa nu se blocheze
        if math.isnan(sp.val) or math.isinf(sp.val) or sp.val == 0.0:
            return 10.0
        return sp.val

    # aceasta functie este apelata automat de fiecare data cand vine un mesaj nou de la laser
    def laser_callback(sp, msg):
        # luam toate distantele din mesaj
        sp.ranges = msg.ranges
        # unghiul de start al laserului
        sp.angle_min = msg.angle_min
        # cat de mare e pasul intre doua masuratori consecutive
        sp.angle_increment = msg.angle_increment

        # citim distanta din fata (unghi 0 grade)
        sp.front_distance = sp.get_range_at_angle(sp.ranges, 0.0, sp.angle_min, sp.angle_increment)
        # citim distanta din dreapta (unghi -90 grade adica -pi/2)
        sp.right_distance = sp.get_range_at_angle(sp.ranges, -math.pi / 2, sp.angle_min, sp.angle_increment)
        # citim distanta din stanga (unghi +90 grade adica +pi/2)
        sp.left_distance = sp.get_range_at_angle(sp.ranges, math.pi / 2, sp.angle_min, sp.angle_increment)

        # dupa ce am citit datele, decidem cum se misca robotul
        sp.move()

    # functia care decide miscarea robotului in functie de distantele citite
    def move(sp):
        # cream un mesaj nou de viteza
        sp.twist = Twist()

        # daca nu e obstacol in fata (mai mult de 1 metru liber)
        if sp.front_distance > 1.0:
            # verificam si partile laterale
            if sp.right_distance < 1.0:
                # obstacol in dreapta, viram la stanga
                sp.twist.linear.x = 0.0
                sp.twist.angular.z = sp.angular_speed
            elif sp.left_distance < 1.0:
                # obstacol in stanga, viram la dreapta
                sp.twist.linear.x = 0.0
                sp.twist.angular.z = -sp.angular_speed
            else:
                # niciun obstacol, mergem inainte
                sp.twist.linear.x = sp.linear_speed
                sp.twist.angular.z = 0.0
        else:
            # obstacol in fata, viram la stanga (are prioritate fata de orice altceva)
            sp.twist.linear.x = 0.0
            sp.twist.angular.z = sp.angular_speed

        # trimitem comanda de miscare la robot
        sp.pub.publish(sp.twist)


# cream obiectul principal, il numim sp cum a cerut cerintea
sp = RobotController()
# asteptam sa primim mesaje, fara asta nodul se inchide imediat
rospy.spin()
