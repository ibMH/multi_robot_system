#include <Controller/Controller.h>

Controller::Controller(ros::NodeHandle &nh):nh(nh)
{

    set_name("my_slave");
 
    this->output=this->nh.advertise<geometry_msgs::Twist>("out",10);

    this->input=this->nh.subscribe("in",10,&Controller::input_velocities_callback,this);
    this->odom=this->nh.subscribe("mobile_base_controller/odom",10,&Controller::input_odom_callback,this);

   
} 

void Controller::set_reference(double x,double y,double z)
{
    this->reference.position.x=x;
    this->reference.position.y=y;
    this->reference.position.z=z;
}

void Controller::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);
}

//################################################################################################
//Linking important topics/transformations

void Controller::link_input_velocity(std::string topic_name)
{
    this->input.shutdown();
    ROS_INFO("Linking input %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->input=this->nh.subscribe(topic_name,10,&Controller::input_velocities_callback,this);
}

void Controller::link_input_odom(std::string topic_name)
{
    this->odom.shutdown();
    ROS_INFO("Linking odom %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->odom=this->nh.subscribe(topic_name,10,&Controller::input_odom_callback,this);
}

void Controller::link_output_velocity(std::string topic_name)
{
    this->output.shutdown();
    ROS_INFO("Linking output %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->output=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}




//################################################################################################
//callback methods

void Controller::input_velocities_callback(geometry_msgs::Twist msg)
{
    this->msg_velocities_in=msg;
}

void Controller::input_odom_callback(nav_msgs::Odometry msg)
{   
    this->msg_odom=msg;
    this->msg_odom.pose.pose.position.x+=this->reference.position.x;
    this->msg_odom.pose.pose.position.y+=this->reference.position.y;
    this->msg_odom.pose.pose.position.z+=this->reference.position.z;
}



void Controller::optimal_control()
{   

    //Calcualte ideal velocities
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    
    linear=this->msg_velocities_in.linear;
    angular=this->msg_velocities_in.angular;

    this->msg_velocities_ideal.angular.x=angular.x;
    this->msg_velocities_ideal.angular.y=angular.y;
    this->msg_velocities_ideal.angular.z=angular.z;

    this->msg_velocities_ideal.linear.x=linear.x-angular.z*this->reference.position.y;
    this->msg_velocities_ideal.linear.y=linear.y+angular.z*this->reference.position.x;
    this->msg_velocities_ideal.linear.z=linear.z;
    

    //Calculate controlvector
    this->msg_velocities_out.linear.x=cos(this->current_pose.orientation.z)*msg_velocities_ideal.linear.x +sin(this->current_pose.orientation.z)*this->msg_velocities_ideal.linear.y;
    this->msg_velocities_out.angular.z=angular.z;
}



void Controller::scope()
{
    //publish
    geometry_msgs::Twist msg;
    this->optimal_control();

    this->output.publish(this->msg_velocities_out);

    ros::spinOnce();
}