
      #--------------------------------------------------------------------------#
      #                                                                          #
      #                          ROS SETTINGS                                    #
      #                                                                          #
      #--------------------------------------------------------------------------#
        





#--------------------------------------
#
#     WORKSPACE
#
#--------------------------------------

#export octomap_DIR=/opt/ros/indigo/share/octomap

# short link for catkin path
export ROS_CATKIN_WS="/media/a/sft/ros-catkin-ws"

# short link for ros dataset path
export ROS_DATA="/media/a/sft/ROS_DATASET"


ros_general()
{
	source /opt/ros/kinetic/setup.bash 
  source $ROS_CATKIN_WS/general/devel/setup.bash 
	export ROS_PACKAGE_PATH=$ROS_CATKIN_WS/general/src:$ROS_PACKAGE_PATH 
	export ROS_WORKSPACE=$ROS_CATKIN_WS/general
	#printenv | grep ROS
}

ros_olfaction()
{
	source /opt/ros/kinetic/setup.bash 
  source $ROS_CATKIN_WS/olfaction-demo/devel/setup.bash 
	export ROS_PACKAGE_PATH=$ROS_CATKIN_WS/olfaction-demo/src:$ROS_PACKAGE_PATH 
	export ROS_WORKSPACE=$ROS_CATKIN_WS/olfaction-demo
	#printenv | grep ROS
}

ros_disp()
{
	source /opt/ros/kinetic/setup.bash 
  source $ROS_CATKIN_WS/gas-disp/devel/setup.bash 
	export ROS_PACKAGE_PATH=$ROS_CATKIN_WS/gas-disp/src:$ROS_PACKAGE_PATH 
	export ROS_WORKSPACE=$ROS_CATKIN_WS/gas-disp
	#printenv | grep ROS
}

ros_jfr()
{
	source /opt/ros/kinetic/setup.bash 
  source $ROS_CATKIN_WS/jfr-experiments/devel/setup.bash 
	export ROS_PACKAGE_PATH=$ROS_CATKIN_WS/jfr-experiments/src:$ROS_PACKAGE_PATH 
	export ROS_WORKSPACE=$ROS_CATKIN_WS/jfr-experiments
	#printenv | grep ROS
}

ros_localization()
{
	source /opt/ros/kinetic/setup.bash 
  source $ROS_CATKIN_WS/localization-test/devel/setup.bash 
	export ROS_PACKAGE_PATH=$ROS_CATKIN_WS/localization-test/src:$ROS_PACKAGE_PATH 
	export ROS_WORKSPACE=$ROS_CATKIN_WS/localization-test
	#printenv | grep ROS
}

ros_smokebot()
{
	source /opt/ros/kinetic/setup.bash 
  source $ROS_CATKIN_WS/smokebot/devel/setup.bash 
	export ROS_PACKAGE_PATH=$ROS_CATKIN_WS/smokebot/src:$ROS_PACKAGE_PATH 
	export ROS_WORKSPACE=$ROS_CATKIN_WS/smokebot
	#printenv | grep ROS
}

ros_smokebot




#--- WORKSPACE
#-------------------------

if [ -f /media/a/sft/ros-catkin-ws/jfr_env.bash ]; then
  . /media/a/sft/ros-catkin-ws/jfr_env.bash
fi

if [ -f /media/a/sft/ros-catkin-ws/smokebot_env.bash ]; then
  . /media/a/sft/ros-catkin-ws/smokebot_env.bash
fi



#---- NETWORKS
#----------------------------
net_gasbot4()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@192.168.200.4 #XXX old wifi adapter
}
net_gasbot5()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@192.168.200.5 #XXX new wifi adapter
}

net_aass12()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@192.168.222.12 #XXX old wifi adapter
}
net_aass24()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@192.168.222.24 #XXX new wifi adapter
}

net_guestnet_husky201()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@10.178.100.201
}
net_guestnet_husky202()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@10.178.100.202
}
net_guestnet_asif203()
{
  sshpass -p "samina" ssh -o StrictHostKeyChecking=no -X a@10.178.100.203
}
net_guestnet_expert204()
{
  sshpass -p "expert" ssh -o StrictHostKeyChecking=no -X asif@10.178.100.204
}
net_guestnet_husky205()
{
  sshpass -p "clearpath" ssh -o StrictHostKeyChecking=no -X husky@10.178.100.205
}


# -------------------------------------------------------------------------------
alias myros="printenv | grep ROS"



#---- ROS MASTER/SLAVE
#----------------------------
gasbot4_ros_slave()
{
  export ROS_MASTER_URI=http://192.168.200.4:11311 
  export ROS_IP=192.168.200.6
  echo $ROS_MASTER_URI 
  echo $ROS_IP
}
gasbot5_ros_slave()
{
  export ROS_MASTER_URI=http://192.168.200.5:11311 
  export ROS_IP=192.168.200.6
  echo $ROS_MASTER_URI 
  echo $ROS_IP
}
aass12_ros_slave()
{
  export ROS_MASTER_URI=http://192.168.222.12:11311
  export ROS_IP=192.168.222.25 #136
  echo $ROS_MASTER_URI 
  echo $ROS_IP
}
aass24_ros_slave()
{
  export ROS_MASTER_URI=http://192.168.222.24:11311
  export ROS_IP=192.168.222.25 #136
  echo $ROS_MASTER_URI 
  echo $ROS_IP
}
guestnet201_ros_slave()
{
  export ROS_MASTER_URI=http://10.178.100.201:11311
  export ROS_IP=10.178.100.203
  export ROS_HOSTNAME=10.178.100.203
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  echo $ROS_HOSTNAME
}
guestnet202_ros_slave()
{
  export ROS_MASTER_URI=http://10.178.100.202:11311
  export ROS_IP=10.178.100.203
  export ROS_HOSTNAME=10.178.100.203
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  echo $ROS_HOSTNAME
}
guestnet205_ros_slave()
{
  export ROS_MASTER_URI=http://10.178.100.205:11311
  export ROS_IP=10.178.100.203
  export ROS_HOSTNAME=10.178.100.203
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  echo $ROS_HOSTNAME
}



#------ GAZEBO
export GAZEBO_MODEL_PATH=/home/a/.gazebo/models    
export GAZEBO_PLUGIN_PATH=/media/a/sft/ros-catkin-ws/smokebot/devel/lib/:$GAZEBO_PLUGING_PATH


