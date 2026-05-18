#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Empty
from tarabostes_test3.msg import TalcAction, TalcFeedback, TalcResult


# cream clasa pentru server
class ServerTarabostes(object):

    # cream obiectele pentru feedback si rezultat
    tarabostes_feedback = TalcFeedback()
    tarabostes_rezultat = TalcResult()

    def __init__(self):
        # publicam pe topicurile de decolare si aterizare ale dronei
        self.tarabostes_pub_decolare = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.tarabostes_pub_aterizare = rospy.Publisher('/ardrone/land', Empty, queue_size=1)

        # cream serverul de actiune si il pornim
        self.tarabostes_server = actionlib.SimpleActionServer(
            'talc', TalcAction, self.tarabostes_callback_goal, False)
        self.tarabostes_server.start()

    def tarabostes_callback_goal(self, tarabostes_goal):
        # citim comanda primita de la client
        tarabostes_comanda = tarabostes_goal.comanda
        tarabostes_rata = rospy.Rate(1)

        if tarabostes_comanda == 'TAKEOFF':
            # trimitem comanda de decolare catre drona
            self.tarabostes_pub_decolare.publish(Empty())
            rospy.loginfo('Drona decoleza')

            # stam in bucla si trimitem feedback pana vine comanda de aterizare
            while not rospy.is_shutdown():
                if self.tarabostes_server.is_preempt_requested():
                    # a venit o noua comanda deci oprim bucla
                    self.tarabostes_server.set_preempted()
                    return

                self.tarabostes_feedback.stare_curenta = 'decolare in desfasurare'
                self.tarabostes_server.publish_feedback(self.tarabostes_feedback)
                tarabostes_rata.sleep()

        elif tarabostes_comanda == 'LAND':
            # trimitem comanda de aterizare catre drona
            self.tarabostes_pub_aterizare.publish(Empty())
            rospy.loginfo('Drona aterizeaza')

            # trimitem feedback de cateva ori cat dureaza aterizarea
            for tarabostes_i in range(3):
                if self.tarabostes_server.is_preempt_requested():
                    self.tarabostes_server.set_preempted()
                    return

                self.tarabostes_feedback.stare_curenta = 'aterizare in desfasurare'
                self.tarabostes_server.publish_feedback(self.tarabostes_feedback)
                tarabostes_rata.sleep()

        else:
            rospy.logwarn('Comanda necunoscuta: %s', tarabostes_comanda)

        # actiunea s-a terminat rezultatul este gol
        self.tarabostes_server.set_succeeded(self.tarabostes_rezultat)


if __name__ == '__main__':
    rospy.init_node('tarabostes_server_nod')
    ServerTarabostes()
    rospy.spin()
