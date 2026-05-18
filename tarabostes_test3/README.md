terminal 1: roslaunch ardrone_gazebo single_ardrone.launch <br> <hr>
terminal 2: roslaunch tarabostes_test3 tarabostes_test3.launch <br> <hr> 
terminal 3: rostopic pub /talc/goal tarabostes_test3/TalcActionGoal "goal: {comanda: 'LAND'}" --once sau rostopic pub /talc/goal tarabostes_test3/TalcActionGoal "goal: {comanda: 'TAKEOFF'}" --once
<br><hr>rostopic echo /ardrone/takeoff si rostopic echo /ardrone/land
