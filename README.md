# mobile_robot_simulation

  <!-- Node for publishing the transform from the world to the Pick-It frame -->
  <node name="static_tf" type="static_transform_publisher"
      args="0 0 0 3.14 0 0 /world /fixed_base" pkg="tf2_ros" />
