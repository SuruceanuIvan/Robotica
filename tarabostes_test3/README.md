terminal 1: roslaunch ardrone_gazebo single_ardrone.launch 
terminal 2: roslaunch tarabostes_test3 tarabostes_test3.launch 
terminal 3: rostopic pub /talc/goal tarabostes_test3/TalcActionGoal "goal: {comanda: 'LAND'}" --once sau rostopic pub /talc/goal tarabostes_test3/TalcActionGoal "goal: {comanda: 'TAKEOFF'}" --once
