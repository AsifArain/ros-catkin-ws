
        #--------------------------------------------------------------------------#
        #                                                                          #
        #                                                                          #
        #                              SMOKEBOT                                    #
        #                                                                          #
        #                                                                          #        
        #                       @author: Asif Arain                                #
        #                       @date:   26-Apr-2018                               #
        #                                                                          #
        #                                                                          #        
        #--------------------------------------------------------------------------#
        



# IP Address
#-------------------------
#export TAUROB_IP="10.0.0.3" #gustav_net 2.4
#export TAUROB_IP="11.0.0.3" # gustav_net 5
#export TAUROB_IP="10.178.100.206" # GuestNet
#export TAUROB_IP="10.178.100.201" # GuestNet
export TAUROB_IP="10.0.0.11" # GuestNet

#export THIS_IP="10.0.0.199"  # Asif Dell Laptop #gustav_net 2.4
#export THIS_IP="11.0.0.199"  # Asif Dell Laptop #gustav_net 5
#export THIS_IP="10.178.100.203"  # Asif Dell Laptop # GuestNet
export THIS_IP="10.0.0.200"  # Asif Dell Laptop # Hannover
        
#----------------------------
#           NETWORK
#----------------------------
gustav_net()
{
    #sshpass -p "gustav3" ssh -o StrictHostKeyChecking=no -X gustav@$TAUROB_IP
    sshpass -p "gustav" ssh -o StrictHostKeyChecking=no -X gustav@$TAUROB_IP
}

hannover_net()
{    
    sshpass -p "1234" ssh -o StrictHostKeyChecking=no -X user@$TAUROB_IP
}


#=====================================
#       ROS MASTER SLAVE
#=====================================
gustav_slave()
{
  export ROS_MASTER_URI=http://$TAUROB_IP:11311
  export ROS_IP=$THIS_IP
  export ROS_HOSTNAME=$THIS_IP
  #echo $ROS_MASTER_URI 
  #echo $ROS_IP
  #echo $ROS_HOSTNAME
  echo 'ROS Master: '$ROS_MASTER_URI', ROS IP: ' $ROS_IP', ROS Hostname: ' $ROS_HOSTNAME
}

hannover_slave()
{
  export ROS_MASTER_URI=http://$TAUROB_IP:11311
  export ROS_IP=$THIS_IP
  export ROS_HOSTNAME=$THIS_IP
  #echo $ROS_MASTER_URI 
  #echo $ROS_IP
  #echo $ROS_HOSTNAME
  echo 'ROS Master: '$ROS_MASTER_URI', ROS IP: ' $ROS_IP', ROS Hostname: ' $ROS_HOSTNAME
}
#hannover_slave

#-------------------------------------
#   GUSTAV RVIZ
#-------------------------------------
gustav_rviz()
{
  export ROS_MASTER_URI=http://$TAUROB_IP:11311
  export ROS_IP=$THIS_IP
  export ROS_HOSTNAME=$THIS_IP
  echo $ROS_MASTER_URI 
  echo $ROS_IP
  echo $ROS_HOSTNAME    
  sleep 1s; roslaunch gustav_launch gustav_rviz.launch
}




#--------------------------------------
#
# Cancel movebase goal
#
#--------------------------------------
cancel_goal()
{
    rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}
}

#--------------------------------------
#  watchdog
#--------------------------------------
watchdog()
{
    rosrun taurob_watchdog_client taurob_watchdog_client 10.0.0.11
}

#--------------------------------------
#  stop radar motor
#--------------------------------------
stop_radar_motor()
{
    rosrun mpr_multi mpr_multi -s
}



