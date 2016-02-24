# Cornell-Research

How to get octomap to work in rViz:
  rosrun octomap_server octomap_server_node <FILENAME.bt>
  rosrun rviz rviz
  then, add MarkerArray topic to occupied_cells_viz_array
  
To build octomap from live kinect data
  roslaunch openni_launch openni.launch
  make edits in octomap_mapping.launch to set map frame name to: "frame_id" with value "camera_depth_frame"
  roslaunch octomap_server octomap_mapping.launch
