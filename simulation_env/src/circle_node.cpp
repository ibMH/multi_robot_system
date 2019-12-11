#include <simulation_env/planner.h>
#include <ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"CirclePlanner");
    ros::NodeHandle nh("~");
    
    tf::Pose ref;
    ref.setOrigin(tf::Vector3(0,0,0));
    
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    ref.setRotation(quat);

    CirclePlanner circle(nh);    
    circle.set_parameter(5.0,0.2);
    circle.set_start_pose(ref);
    if(argc==1)
    {
        circle.load();
        circle.start();
    }
    ros::spin(); 
}