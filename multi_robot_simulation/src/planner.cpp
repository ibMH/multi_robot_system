#include <multi_robot_simulation/planner.h>

//Implementation of a generic planner class########################################################################################
//#################################################################################################################################
Planner::Planner(ros::NodeHandle &nh):nh(nh)
{
    this->frame_name="/map";

    this->tim_sampling=this->nh.createTimer(ros::Duration(0.05),&Planner::plan,this);

    this->pub_current_odometry=nh.advertise<nav_msgs::Odometry>("/trajectory_odom",10);
    this->set_start_service=nh.advertiseService("start_planner",&Planner::srv_start,this);
    this->set_stop_service=nh.advertiseService("stop_planner",&Planner::srv_stop,this);

    this->paused=ros::Duration(0,0);
    this->start_reference.setIdentity();
    this->is_planning=false;
    this->iterations=1;
    this->iterations_counter=0;
    
    this->start_pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
    this->vel=tf::Vector3(0,0,0);
    this->pos=tf::Vector3(0,0,0);
    this->ang_vel=0.0;
    this->orientation=tf::createQuaternionFromYaw(0);
}


void Planner::plan(const ros::TimerEvent& event)
{
    if(this->iterations_counter==this->iterations)
    {
        stop();
    }
    if(this->is_planning)
    {
        ros::Duration local_time;
        local_time=ros::Time::now()-this->start_time-this->paused;        
        
        this->vel=this->get_velocity(local_time);
        this->pos=this->get_position(local_time);
        this->orientation=this->get_orientation(local_time);
        this->ang_vel=this->get_angular_velocity(local_time);
        this->check_period(local_time);
        this->transformValues(this->start_reference);
        this->publish();
    }
    
}


void Planner::start()
{
    this->start_time=ros::Time::now();
    this->is_planning=true;
    this->start_reference=this->getTransform(this->start_pose);
}

void Planner::pause()
{
    this->is_planning=false;
    this->is_paused=true;
}

bool Planner::srv_start(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if(this->is_planning)
    {this->pause();}
    else
    {this->start();}
}

void Planner::stop()
{
    ROS_INFO("Shutting down node: %s",ros::this_node::getName().c_str());
    ros::shutdown();
}
bool Planner::srv_stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->stop();
    return true;
}

void Planner::setStartPose(tf::Pose pose)
{
    this->start_pose=pose;
}


tf::Transform Planner::getTransform(tf::Pose pose)
{   
    tf::Pose planner_start( this->get_orientation(ros::Duration(0)),
                            this->get_position(ros::Duration(0)));

    ROS_INFO(   "Planner starts at: x-%lf y-%lf theta-%lf",
                planner_start.getOrigin().x(),
                planner_start.getOrigin().y(),
                tf::getYaw(planner_start.getRotation()));
    
    tf::Transform trafo(pose*planner_start.inverse());

    ROS_INFO(   "Trafo applied on planner values: x-%lf y-%lf z-%lf theta-%lf ",
                trafo.getOrigin().x(),
                trafo.getOrigin().y(),
                trafo.getOrigin().z(),               
                tf::getYaw(trafo.getRotation()));
    return trafo;
}


void Planner::transformValues(tf::Transform trafo)
{
    tf::Transform rotate(trafo.getRotation());
    this->vel=rotate*this->vel;
    this->pos=trafo*this->pos;
    this->orientation=rotate*this->orientation;
}


void Planner::publish()
{    
    nav_msgs::Odometry msg2;
    msg2.header.frame_id=this->frame_name;
    msg2.header.stamp=ros::Time::now();
    tf::Pose pose(this->orientation,this->pos);
    tf::poseTFToMsg(pose,msg2.pose.pose);
    tf::vector3TFToMsg(this->vel,msg2.twist.twist.linear);
    msg2.twist.twist.angular.z=this->ang_vel;    
    this->pub_current_odometry.publish(msg2);
}

void Planner::load()
{
    if(!this->nh.getParam(PARAM_ITERATION,this->iterations))
    {
        ROS_WARN("Could not load %s",this->nh.resolveName(PARAM_ITERATION).c_str());
    }
    std::vector<float> ref_rpy;
    if(!this->nh.getParam(PARAM_REFERENCE,ref_rpy))
    {
        ROS_WARN("Could not load %s",this->nh.resolveName(PARAM_REFERENCE).c_str());
    }
    else
    {
        this->setStartPose(tf::Pose(tf::createQuaternionFromRPY(ref_rpy[3],ref_rpy[4],ref_rpy[5]),tf::Vector3(ref_rpy[0],ref_rpy[1],ref_rpy[2])));
    }    
    this->loadChild();
}








//Implementation of a Planner that gives a circle########################################################################################
//#######################################################################################################################################
CirclePlanner::CirclePlanner(ros::NodeHandle &nh):Planner(nh)
{
 
}
void CirclePlanner::set_parameter(double r, double omega)
{
    this->plan.r=r;
    this->plan.omega=omega;
}


tf::Vector3 CirclePlanner::get_position(ros::Duration time)
{
    double t=time.toSec();    
    tf::Vector3 pos(sin(this->plan.omega*t)*this->plan.r,
                    -cos(this->plan.omega*t)*this->plan.r,
                    0);
    return pos;

}

tf::Quaternion CirclePlanner::get_orientation(ros::Duration time)
{
    return tf::createQuaternionFromYaw(this->plan.omega*time.toSec());
}

tf::Vector3 CirclePlanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 vel(cos(this->plan.omega*t)*this->plan.r*this->plan.omega,
                    sin(this->plan.omega*t)*this->plan.r*this->plan.omega,
                    0);
    return vel;
}
 tf::Vector3 CirclePlanner::get_acceleration(ros::Duration time)
 {
     return tf::Vector3();
 }
double CirclePlanner::get_angular_velocity(ros::Duration time)
{
    return this->plan.omega;    
}

void CirclePlanner::loadChild()
{
    try{
        this->nh.getParam(PARAM_ITERATION,this->iterations);
        this->nh.getParam(PARAM_R,this->plan.r);
        this->nh.getParam(PARAM_OMEGA,this->plan.omega);
        ROS_INFO("Loaded %s : Radius: %f Omega: %f",    ros::this_node::getName().c_str(),
                                                        this->plan.r,
                                                        this->plan.omega);
    }
    catch(...)
    {
        ROS_INFO("Error due loading paramter of: %s",ros::this_node::getName().c_str());
    }
   
}

void CirclePlanner::check_period(ros::Duration time)
{
    if(this->plan.omega*time.toSec()>(iterations_counter+1)*2*M_PI)
    {
        this->iterations_counter++;
    }
}




//Implementation of a Planner that gives a Lissajous figures###################################################################################
//#############################################################################################################################################
LissajousPlanner::LissajousPlanner(ros::NodeHandle &nh):Planner(nh)
{
}


tf::Vector3 LissajousPlanner::get_position(ros::Duration time)
{
    double t;
    t=time.toSec();
    tf::Vector3 r(0,0,0);
    double x;
    x=this->plan.Ax*sin(this->plan.omegax*t);
    double y;
    y=this->plan.Ay*sin(this->plan.omegax*this->plan.ratio*t+this->plan.dphi);

    return tf::Vector3(x,y,0);
}

tf::Vector3 LissajousPlanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    double dx=this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax;
    double dy=this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio;
    tf::Vector3 vel(dx,
                    dy,
                    0);
    return vel;
}

 tf::Vector3 LissajousPlanner::get_acceleration(ros::Duration time)
 {

    double t=time.toSec();
    double dx=-this->plan.Ax*sin(this->plan.omegax*t)*std::pow(this->plan.omegax,2.0);
    double dy=-this->plan.Ay*sin(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*std::pow(this->plan.omegax*this->plan.ratio,2.0);
    tf::Vector3 acc(dx,
                    dy,
                    0);
    return acc;
 }

tf::Quaternion LissajousPlanner::get_orientation(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 tangent;
    tangent.setX(this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax);
    tangent.setY(this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio);
    return tf::createQuaternionFromYaw(atan2(tangent.y(),tangent.x()));

}

double LissajousPlanner::get_angular_velocity(ros::Duration time)
{
    tf::Vector3 acc(this->get_acceleration(time));
    tf::Vector3 vel(this->get_velocity(time));
    return (vel.x()*acc.y()-vel.y()*acc.x())/vel.length2();
}


void LissajousPlanner::set_parameter(float omegax,float dphi,int ratio,float Ax,float Ay)
{
    this->plan.Ax=Ax;
    this->plan.Ay=Ay;
    this->plan.omegax=omegax;
    this->plan.dphi=dphi;
    this->plan.ratio=ratio;
}

void LissajousPlanner::loadChild()
{
    this->nh.getParam(PARAM_ITERATION,this->iterations);
    this->nh.getParam(PARAM_AMP_X,this->plan.Ax);
    this->nh.getParam(PARAM_AMP_Y,this->plan.Ay);
    this->nh.getParam(PARAM_RATIO,this->plan.ratio);
    this->nh.getParam(PARAM_PHASE,this->plan.dphi);
    this->nh.getParam(PARAM_OMEGA,this->plan.omegax);
    ROS_INFO("Loaded %s : AmplifierX %lf AmplifierY %lf Ratio %i PhaseShift %lf Omega: %lf ",   
                                                    ros::this_node::getName().c_str(),
                                                    this->plan.Ax,
                                                    this->plan.Ay,
                                                    this->plan.ratio,
                                                    this->plan.dphi,
                                                    this->plan.omegax);
}

void LissajousPlanner::check_period(ros::Duration time)
{
    if(this->plan.omegax*time.toSec()>(this->iterations_counter+1)*2*M_PI)
    {
        this->iterations_counter++;
    }
}


// ################################################################################################################################################

// ################################################################################################################################################

// ###############################################################################################################################################

ClickedPosePlanner::ClickedPosePlanner(ros::NodeHandle &nh,std::string topic_name):Planner(nh)
{
    this->sub_=this->nh.subscribe<geometry_msgs::PoseStamped>(topic_name,10,boost::bind(&ClickedPosePlanner::clickedCallback,this,_1));
    this->pose_=this->start_reference;
    this->start_reference=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
}


tf::Vector3 ClickedPosePlanner::get_position(ros::Duration time)
{
    return this->pose_.getOrigin();
}

tf::Vector3 ClickedPosePlanner::get_velocity(ros::Duration time)
{
    if(this->direction_.length()>0.01)
    {
        return this->direction_.normalized();
    }
    else
    {
        return tf::Vector3(0,0,0);
    }
    
}    
 tf::Vector3 ClickedPosePlanner::get_acceleration(ros::Duration time)
 {
     return tf::Vector3();
 }

tf::Quaternion ClickedPosePlanner::get_orientation(ros::Duration time)
{
  return this->pose_.getRotation();
}

double ClickedPosePlanner::get_angular_velocity(ros::Duration time)
{
    return 0.0;
}

void ClickedPosePlanner::check_period(ros::Duration time)
{
    return;
}

void ClickedPosePlanner::clickedCallback(const geometry_msgs::PoseStampedConstPtr msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose,pose);
    this->direction_=pose.getOrigin()-this->pose_.getOrigin();
    this->pose_=pose;
    
}
void ClickedPosePlanner::loadChild()
{
    return;
}



//Implementation of a Planner that gives a spiral########################################################################################
//#######################################################################################################################################
Spiralplanner::Spiralplanner(ros::NodeHandle &nh):Planner(nh)
{
 
}
void Spiralplanner::set_parameter(double r_offset,double r_growth, double omega)
{
    this->plan.r_offset=r_offset;
    this->plan.r_growth=r_growth;
    this->plan.omega=omega;
}


tf::Vector3 Spiralplanner::get_position(ros::Duration time)
{
    double t=time.toSec();    
    tf::Vector3 pos(sin(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t),
                    -cos(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t),
                    0);
    return pos;

}

tf::Quaternion Spiralplanner::get_orientation(ros::Duration time)
{
    return tf::createQuaternionFromYaw(this->plan.omega*time.toSec());
}

tf::Vector3 Spiralplanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 vel(cos(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t)+sin(this->plan.omega*t)*this->plan.r_growth*this->plan.omega,
                    sin(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t)-cos(this->plan.omega*t)*this->plan.r_growth*this->plan.omega,
                    0.0);
    return vel;
}
 tf::Vector3 Spiralplanner::get_acceleration(ros::Duration time)
 {
     return tf::Vector3();
 }

double Spiralplanner::get_angular_velocity(ros::Duration time)
{
    return this->plan.omega;    
}

void Spiralplanner::loadChild()
{
    try{
        this->nh.getParam(PARAM_ITERATION,this->iterations);
        this->nh.getParam(PARAM_GROWTH,this->plan.r_growth);
        this->nh.getParam(PARAM_OMEGA,this->plan.omega);
        this->nh.getParam(PARAM_R0,this->plan.r_offset);
        ROS_INFO("Loaded %s : R_OFfset: %f  r_growth %f Omega: %f",    
                                                        ros::this_node::getName().c_str(),
                                                        this->plan.r_offset,
                                                        this->plan.r_growth,
                                                        this->plan.omega);
    }
    catch(...)
    {
        ROS_INFO("Error due loading paramter of: %s",ros::this_node::getName().c_str());
    }   
}

void Spiralplanner::check_period(ros::Duration time)
{
    if(this->plan.omega*time.toSec()>2*M_PI)
    {
        this->iterations_counter++;
    }
}


//Implementation of a StepResponse planner that gives a spiral########################################################################################
//#######################################################################################################################################
StepResposePlanner::StepResposePlanner(ros::NodeHandle &nh):Planner(nh)
{ 
    this->pos_old_=this->start_reference.getOrigin();
    this->ori_old_=this->start_reference.getRotation();
    this->iterations_counter=1;
}

tf::Vector3 StepResposePlanner::get_position(ros::Duration time)
{
    int descide=iterations_counter%3;
    ROS_INFO("%i",descide);
    switch(descide)
    {
        case 0:  this->pos_new_=this->pos_old_+tf::Vector3(this->plan.step_sizes[0],0.0,0.0);break;
        case 2:  this->pos_new_=this->pos_old_+tf::Vector3(0.0,this->plan.step_sizes[1],0.0);break;
        default:  this->pos_new_=this->pos_old_;
    }
    return this->pos_new_;
}

tf::Quaternion StepResposePlanner::get_orientation(ros::Duration time)
{
    int descide=iterations_counter%3;
    switch(descide)
    {
        case 1  :  this->ori_new_=tf::createQuaternionFromRPY(0.0,0.0,this->plan.step_sizes[2]*M_PI)*this->ori_old_;break;
        default:   this->ori_new_=this->ori_old_;
    }
    return this->ori_new_;
}

tf::Vector3 StepResposePlanner::get_velocity(ros::Duration time)
{
    return tf::Vector3(0.0,0.0,0.0);
}
 tf::Vector3 StepResposePlanner::get_acceleration(ros::Duration time)
 {
    return tf::Vector3(0.0,0.0,0.0);
 }

double StepResposePlanner::get_angular_velocity(ros::Duration time)
{
    return 0.0;    
}

void StepResposePlanner::loadChild()
{
    std::vector<float> steps;
    if(!this->nh.getParam(PARAM_STEP_SIZES,steps))
    {
        ROS_WARN("Could not load %s",this->nh.resolveName(PARAM_STEP_SIZES).c_str());
    }

    float delay;
    if(!this->nh.getParam(PARAM_DELAY,delay))   
    {
        ROS_WARN("Could not load %s",this->nh.resolveName(PARAM_DELAY).c_str());
    }
    this->plan=StepPlan(delay,steps);
}

void StepResposePlanner::check_period(ros::Duration time)
{
    float t=time.toSec();
    if(t>this->iterations_counter*this->plan.delay_time)
    {
        this->iterations_counter++;
        this->pos_old_=this->pos_new_;
        this->ori_old_=this->ori_new_;
    }   
}
