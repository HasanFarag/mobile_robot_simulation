# mobile_robot_simulation

killall -9 gzserver

killall -9 gzclient

killall -9 rosmaster

 roslaunch sjtu_drone assault.launch
 
 rosrun ball_chaser drone_solar.py

  <!-- Node for publishing the transform from the world to the Pick-It frame -->
  
  <node name="static_tf" type="static_transform_publisher"
      args="0 0 0 3.14 0 0 /world /fixed_base" pkg="tf2_ros" />
      
      
       <node pkg="tf" type="static_transform_publisher" name="$(anon tf)" args="0 0 0 0 0 0 /base /world 30" />
       
       

       


