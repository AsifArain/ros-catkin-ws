
        #--------------------------------------------------------------------------#
        #                                                                          #
        #                                                                          #
        #             EXPLORATION STRATEGIES FOR GAS EMISSION MONITORING           #
        #                                                                          #
        #                                                                          #        
        #                       @author: Asif Arain                                #
        #                       @date:   02-Mar-2018                               #
        #                                                                          #
        #                                                                          #        
        #--------------------------------------------------------------------------#
        



# IP Address
#-------------------------
export HUSKY_IP="10.178.100.201" # TP-Link 
#export HUSKY_IP="10.178.100.202" # D-Link DWA-182
#export HUSKY_IP="10.178.100.205"  # D-Link DWA-160
export THIS_IP="10.178.100.203"  # Asif Dell Laptop
        
#----------------------------
#           NETWORK
#----------------------------
husky_net()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@$HUSKY_IP
  #sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X asif@$HUSKY_IP
}



#=====================================
#       EXPLORATION STRATEGIES
#=====================================

#-- human expert
#----------------------------
human()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s; rosrun rviz rviz & 
  sleep 5s; roscd exploration_launch; roslaunch main_human_expert_exploration.launch
}

#-- one step exploration
#----------------------------
one_step()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s; rosrun rviz rviz & 
  sleep 5s; roscd exploration_launch; roslaunch main_one_step_exploration.launch
}

#-- two step exploration -- gas detectiion
#-------------------------------------------
two_step_detection()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s; rosrun rviz rviz & 
  sleep 5s; roscd exploration_launch; roslaunch main_two_step_exploration_detection.launch
}

#-- two step exploration -- gas tomography with adaptive replanning
#--------------------------------------------------------------------
two_step_adaptive_tomography()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s; rosrun rviz rviz & 
  sleep 5s; roscd exploration_launch; roslaunch main_two_step_exploration_tomography_adaptive.launch
}

#-- two step exploration -- gas tomography with initial plan
#-------------------------------------------------------------
two_step_fixed_tomography()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s; rosrun rviz rviz & 
  sleep 5s; roscd exploration_launch; roslaunch main_two_step_exploration_tomography_fixed.launch
}

#-- matlab
#----------------------------
matlab_exploration()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s; 17b
}

#-- dummy exploration
#-------------------------------------------------------------
dummy()
{
  roslaunch exploration_launch dummy_exploration.launch
}

#=====================================
#       ROSBAG RECORD
#=====================================
rosbag_record_selected()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s
  rosbag record --lz4 --split --size=1024 -e "((/amtec(.*))|(/encoder)|(/env_map)|(/gdm_exploration(.*))|(/joint_states)|(/ndt_mcl)|(/planned(.*))|(/ptu_control(.*))|(/rmld(.*))|(/executed(.*))|(/tf(.*))|(/placement_msg(.*)))" -O `date +%Y-%m-%d-%H-%M-%S`    
}

rosbag_record_except()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s
  rosbag record -a --lz4 --split --size=1024 -x "((/velodyne/scan)|(/lidar/scan)|(velodyne_point(.*)))" -O `date +%Y-%m-%d-%H-%M-%S`
}


#=====================================
#       ROS PARAMETERS RECORD
#=====================================
param_record()
{
  export ROS_MASTER_URI=http://$HUSKY_IP:11311
  export ROS_IP=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  sleep 1s
  rosparam dump `date +%Y-%m-%d-%H-%M-%S`.yaml
}


ExperimentLogDir() 
{
  #do things with parameters like $1 such as
  #mv "$1" "$1.bak"
  #cp "$2" "$1"
  #alias planner='cd /media/a/wrk/planners/spp-tomography/arain-spp-rem-jfr2017'
  #cd /media/a/wrk/planners/spp-tomography/arain-spp-rem-jfr2017/
  #cd /media/a/wrk/planners/spp-tomography/arain-spp-rem-jfr2017/ExperimentLogs/prismaforum5-"$1"/
  cd /media/a/wrk/planners/spp-tomography/arain-spp-rem-jfr2017/ExperimentLogs/prismaforum5-"$1"/
}

#=====================================
#       VIDEO RECORD
#=====================================
video_record()
{
  streamer -t 0:4:00 -c /dev/video0 -f rgb24 -r 15 -o arain_cam_`date +%Y-%m-%d-%H-%M-%S`.avi
}

#printf "some text here \n"
#printf "HIV test negative \n"
#printf "Pregrancy test negative \n"
#printf "| HIV test negative | Pregrancy test negative |\n"
#printf "|- Pregrancy test negative -|\n"
#echo "Dil ka un sey lagan wabal-e-jaan thehra"
