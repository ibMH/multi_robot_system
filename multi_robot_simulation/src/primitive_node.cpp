

#include <multi_robot_simulation/primitive_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <multi_robot_msgs/PathVelocities.h>
#include <multi_robot_simulation/planner.h>
#include <tf/tf.h>
int main(int argc, char** argv)
{
    ros::init(argc,argv,"primitive_node"); 
    ros::NodeHandle nh;
    ros::Publisher pub_path=nh.advertise<nav_msgs::Path>("/path",1);
    ros::Publisher pub_path_vel=nh.advertise<multi_robot_msgs::PathVelocities>("/path_vel",1);

    nav_msgs::Path path;
    multi_robot_msgs::PathVelocities path_velocities;
    
    PrimitivePlanner::CirclePrimitive primitive(0.2,10.0,3,1.57);
    primitive.start_point_=tf::Transform(tf::createQuaternionFromYaw(0),tf::Vector3(0.0,0.0,0.0));
    primitive.start_vel_=0.0;
    if(!primitive.interpolate(0.1))
    {
        ROS_WARN("Error due interpolation");
    }
    
    for(auto point:primitive.getPosition())
    {
        geometry_msgs::PoseStamped pose;
        tf::poseTFToMsg(tf::Pose(point),pose.pose);
        path.poses.push_back(pose);
    }
    std::vector<tf::Vector3>velocities=primitive.getVelocity();
    std::vector<double>angular_velocities=primitive.getAngularVelocity();
    for(int i=0;i<velocities.size();i++)
    {
        geometry_msgs::Twist twist;
        tf::vector3TFToMsg(velocities.at(i),twist.linear);
        tf::vector3TFToMsg(tf::Vector3(0.0,0.0,angular_velocities.at(i)),twist.angular);
        path_velocities.velocities.push_back(twist);
    }

    PrimitivePlanner planner(nh);
    planner.start();    
    ros::spin();
    ros::Rate rate(10);
    while(ros::ok())
    {
        path.header.frame_id="/map";
        path_velocities.header.frame_id="/map";
        pub_path.publish(path);
        pub_path_vel.publish(path_velocities);
        rate.sleep();
    }


   
}