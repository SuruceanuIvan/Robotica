#!/usr/bin/env python3

import sys
import rospy
# importam mesajul nostru
from tarabostes_test2.srv import Talc

# initializam nodul de client
rospy.init_node('client_tarabostes')

# preluam latura
tarabostes_latura_dorita = float(sys.argv[1])
tarabostes_nr_repetitii_dorit = int(sys.argv[2])

# Dau un mesaj ca sa mi fie clar ca clientul functioneaza
print("Trimit cererea la server: latura=" + str(tarabostes_latura_dorita) + " repetitii=" + str(tarabostes_nr_repetitii_dorit))

# astept raspunsul de la service
rospy.wait_for_service('/triunghi_tarabostes')

# preiau functia de la serviciu
tarabostes_apel = rospy.ServiceProxy('/triunghi_tarabostes', Talc)

# folosesc referinta la functie ca sa folosesc functia creata in service
tarabostes_rezultat = tarabostes_apel(tarabostes_latura_dorita, tarabostes_nr_repetitii_dorit)

# Dau niste mesaje pentru a clarifica situatia
# o fac pentru mine ceva de genu debugging
if tarabostes_rezultat.success:
    print("Robotul a terminat toate triunghiurile cu succes!")
else:
    print("Ceva nu a mers bine la robot...")
