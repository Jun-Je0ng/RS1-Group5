INSTRUCTIONS FOR MOVE_BOT NODE

1. make sure you have src folder that contains controller, skidsteer and main
2. run no.3 for running the world with main spawn point
3. ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=large_demo rviz:=true robot_x:=8.0 robot_y:=62.0 robot_z:=0.5
4. open new terminal and go to 41068_ignition_bringup path
5. run no.6 and no.7 in path
6. colcon build --packages-select 41068_ignition_bringup
7. source install/setup.bash
8. run no.9 to start the move_bot node
9. ros2 run 41068_ignition_bringup move_bot
10. this will run the default parameters however, you can adjust manually
11. open new terminal and run:
12. x movement: ros2 param set /move_bot lin_x 0.0
13. z movement: ros2 param set /move_bot ang_z 0.0
14. values must be a double value you can do negative value for going the opposite direction
