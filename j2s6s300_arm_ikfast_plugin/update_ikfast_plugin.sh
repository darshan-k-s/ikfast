search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=j2s6s300.srdf
robot_name_in_srdf=j2s6s300
moveit_config_pkg=j2s6s300_moveit_config
robot_name=j2s6s300
planning_group_name=arm
ikfast_plugin_pkg=j2s6s300_arm_ikfast_plugin
base_link_name=root
eef_link_name=j2s6s300_end_effector
ikfast_output_path=/home/darshan/iisc/kinova/src/kinova_description/urdf/j2s6s300_arm_ikfast_plugin/src/j2s6s300_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
