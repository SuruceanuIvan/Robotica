#!/usr/bin/env python3

import rospy
from math import sqrt, atan2
# Am importat niste mesaje care ne vor ajuta sa comunicam cu husky
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Aici am importat mesajul nostru propriu din folder srv
from tarabostes_test2.srv import Talc, TalcResponse

# functia care ne ajuta sa calculam distanta
def tarabostes_calculeaza_distanta(tarabostes_poz_initiala):
    
    tarabostes_odom_acum = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=5)
    tarabostes_poz_acum = tarabostes_odom_acum.pose.pose.position
    
    # ne folosim de teoria lui pitagora
    tarabostes_dist = sqrt(
        (tarabostes_poz_acum.x - tarabostes_poz_initiala.x) ** 2 +
        (tarabostes_poz_acum.y - tarabostes_poz_initiala.y) ** 2
    )

    return tarabostes_dist

# o functie care ajuta robotul sa mearga inainte si atat
def tarabostes_mergi_inainte(tarabostes_cat_sa_merg):
    tarabostes_odom_start = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=5)
    tarabostes_poz_start = tarabostes_odom_start.pose.pose.position

    tarabostes_viteza = Twist()
    tarabostes_viteza.linear.x = 0.7

    while tarabostes_calculeaza_distanta(tarabostes_poz_start) < tarabostes_cat_sa_merg:
        tarabostes_pub.publish(tarabostes_viteza)
        rospy.sleep(0.1)

    tarabostes_viteza.linear.x = 0.0
    tarabostes_pub.publish(tarabostes_viteza)
    
    rospy.sleep(0.5)

# citeste unghiul curent al robotului din odometrie
def tarabostes_get_yaw():
    tarabostes_odom_yaw = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=5)
    tarabostes_q = tarabostes_odom_yaw.pose.pose.orientation
    tarabostes_yaw = atan2(2*(tarabostes_q.w*tarabostes_q.z + tarabostes_q.x*tarabostes_q.y),
                           1 - 2*(tarabostes_q.y*tarabostes_q.y + tarabostes_q.z*tarabostes_q.z))
    return tarabostes_yaw

# aici robotul se roteste exact 120 de grade folosind odometria
def tarabostes_intoarce_120():
    global tarabostes_pub

    # am incercat sa fac ceva ca sa faca si a treia latura
    tarabostes_unghi_tinta = 2 * 3.14159 / 3
    tarabostes_unghi_acumulat = 0.0
    tarabostes_yaw_anterior = tarabostes_get_yaw()

    tarabostes_viteza_rotire = Twist()
    tarabostes_viteza_rotire.angular.z = 0.5

    while tarabostes_unghi_acumulat < tarabostes_unghi_tinta:
        tarabostes_pub.publish(tarabostes_viteza_rotire)
        rospy.sleep(0.1)

        tarabostes_yaw_acum = tarabostes_get_yaw()
        tarabostes_delta = tarabostes_yaw_acum - tarabostes_yaw_anterior

        if tarabostes_delta < -3.14159:
            tarabostes_delta += 2 * 3.14159
        elif tarabostes_delta > 3.14159:
            tarabostes_delta -= 2 * 3.14159

        tarabostes_unghi_acumulat += tarabostes_delta
        tarabostes_yaw_anterior = tarabostes_yaw_acum

    tarabostes_viteza_rotire.angular.z = 0.0
    tarabostes_pub.publish(tarabostes_viteza_rotire)
    rospy.sleep(0.5)

# o fct cu ajutarul caroia robotul descriu triunghi
def tarabostes_executa_triunghi(tarabostes_latura_triunghi):
    print("Robotul incepe sa faca un triunghi cu latura de " + str(tarabostes_latura_triunghi) + " metri")
    
    # am trei laturi deci trebuie sa fac un anumit set de miscari
    # de trei ori ca triunghi e format din 3 laturu (logic)
    # dupa ce am parcurs o latura robotul trebuie sa schimbe unghi pentru a 
    # incepe o latura noua
    for tarabostes_nr_latura in range(3):
        print("Latura nr " + str(tarabostes_nr_latura + 1))
        tarabostes_mergi_inainte(tarabostes_latura_triunghi)
        print("Virez cu 120 de grade")
        tarabostes_intoarce_120()
    
    print("Triunghiul e terminat!")

# functia propriu zisa care misca robot
# aceasta functie o sa dau serviciului /triunghi_tarabostes
# dupa clientul va putea folosi referinta la aceasta functie
def tarabostes_callback(tarabostes_cerere):
    tarabostes_raspuns = TalcResponse()

    # preiau latura si de cate ori trebuie sa fac triunghi
    tarabostes_latura_primita = tarabostes_cerere.latura
    tarabostes_repetitii_primite = tarabostes_cerere.nr_repetitii

    # daca liniilke nu au dat eroare inseamna ca am primit cererea de la client
    print("Am primit cererea de la client")
    print("Latura este " + str(tarabostes_latura_primita) + " si trebuie sa fac " + str(tarabostes_repetitii_primite) + " triunghiuri")

    # acest for se va executa atatea ori cate repetari a trimis client
    for tarabostes_contor in range(tarabostes_repetitii_primite):
        print("Repetitia " + str(tarabostes_contor + 1) + " din " + str(tarabostes_repetitii_primite))
        tarabostes_executa_triunghi(tarabostes_latura_primita)

    print("Gata am facut toate triunghiurile boss!")

    # daca nu a dat eroare inseamna ca robotul chiar a facut triunghi de nr ori
    tarabostes_raspuns.success = True
    
    # returnez raspuns
    return tarabostes_raspuns

# initializes nodul de server
rospy.init_node('server_tarabostes')

# fac un publisher ca pentru cmd_vel
# adica am nevoie de fluxul continuu
tarabostes_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
tarabostes_serviciu = rospy.Service('/triunghi_tarabostes', Talc, tarabostes_callback)

# dau mesaj sa fiu linistit ca serverul merge
print("Serverul tarabostes e pornit si asteapta")

rospy.spin()
