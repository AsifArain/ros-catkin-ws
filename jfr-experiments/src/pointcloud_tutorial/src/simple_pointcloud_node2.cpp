#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


int main(int argc, char** argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);


    /*
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "/map";
    msg->height = msg->width = 1;
    msg->points.push_back (pcl::PointXYZ(0.0, 0.0, 0.0));
    msg->points.push_back (pcl::PointXYZ(1.0, 1.0, 1.0));
    //msg->points.push_back (pcl::PointXYZ(3.0, 3.0, 3.0));
    */
    
    PointCloud combined;
    
    double py1 = 1.0;
    double px1 = 1.0;
    double pZ1 = 1.0;
    pcl::PointXYZRGB toPush;
    toPush.x = px1; toPush.y = py1; toPush.z = pZ1;
    toPush.r = 255; toPush.g = 0.3f; toPush.b = 0.3f;
    combined.points.push_back(toPush);
    
    double py2 = 0.0;
    double px2 = 0.0;
    double pZ2 = 0.0;
    //pcl::PointXYZ toPush;
    toPush.x = px2; toPush.y = py2; toPush.z = pZ2;
    toPush.r = 0.7; toPush.g = 0.7; toPush.b = 0.7;
    combined.points.push_back(toPush);
    
    
    combined.height = 1;
    combined.width = combined.points.size();
    combined.header.frame_id = "/map";
    
    
    ros::Rate loop_rate(1);
        while (nh.ok()){
        //msg->header.stamp = ros::Time::now().toNSec();    
        //pub.publish (msg);
        
        
        //combined.header.stamp = ros::Time::now(); 
        //combined.header.stamp = ros::Time::now();
        
        
        ros::Time time_st = ros::Time::now ();
        //msg->header.stamp = time_st.toNSec()/1e3;
        combined.header.stamp = time_st.toNSec()/1e3;
        
        
        pub.publish (combined.makeShared());
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    
    
    
    
    
    
}




    
    
    
