# mobile_robot_simulation



  <!-- Node for publishing the transform from the world to the Pick-It frame -->
  
  <node name="static_tf" type="static_transform_publisher"
      args="0 0 0 3.14 0 0 /world /fixed_base" pkg="tf2_ros" />
      
      
       <node pkg="tf" type="static_transform_publisher" name="$(anon tf)" args="0 0 0 0 0 0 /base /world 30" />


sudo apt-get install python-genmsg

sudo find / -name "libopencv_core.so.3.2*"

Write /opt/ros/kinetic/lib/x86_64-linux-gnu/ at /etc/ld.so.conf.d/opencv.conf

sudo ldconfig -v
       
sudo apt-get install ros-kinetic-manipulation-msgs        

       


