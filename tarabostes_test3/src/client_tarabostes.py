#!/usr/bin/env python
import rospy
import actionlib
from tarabostes_test3.msg import TalcAction, TalcGoal


def tarabostes_callback_feedback(tarabostes_feedback):
    # afisam in terminal ce face drona in momentul asta
    rospy.loginfo('Feedback primit: %s', tarabostes_feedback.stare_curenta)


if __name__ == '__main__':
    rospy.init_node('tarabostes_client_nod')

    # cream clientul si asteptam ca serverul sa fie gata
    tarabostes_client = actionlib.SimpleActionClient('talc', TalcAction)
    rospy.loginfo('Asteptam serverul de actiune')
    tarabostes_client.wait_for_server()

    # construim goal-ul de decolare si il trimitem
    tarabostes_goal_decolare = TalcGoal()
    tarabostes_goal_decolare.comanda = 'TAKEOFF'
    rospy.loginfo('TAKEOFF')
    tarabostes_client.send_goal(tarabostes_goal_decolare, feedback_cb=tarabostes_callback_feedback)

    # drona ramane in aer 5 secunde
    rospy.loginfo('Drona este in aer asteptam 5 secunde')
    rospy.sleep(5)

    # trimitem comanda de aterizare aceasta opreste actiunea de decolare
    tarabostes_goal_aterizare = TalcGoal()
    tarabostes_goal_aterizare.comanda = 'LAND'
    rospy.loginfo('LAND')
    tarabostes_client.send_goal(tarabostes_goal_aterizare, feedback_cb=tarabostes_callback_feedback)

    # asteptam sa se termine aterizarea
    tarabostes_client.wait_for_result()
    rospy.loginfo('Actiunea s-a terminat drona a aterizat')
