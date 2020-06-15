#include <ros/ros.h>
#include <multi_robot_msgs/SetInitialPose.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <multi_robot_simulation/planner.h>
#include <multi_robot_simulation/primitive_planner.h>

Planner* planner;
tf::Pose reference;


std::vector<std::string> getRobotNames(ros::NodeHandle nh)
{
    std::vector<std::string> names;   
    if(!nh.getParam ( "/formation/names", names))
    {
        ROS_WARN("Could not load %s",nh.resolveName( "/formation/names").c_str());
        throw(std::invalid_argument(std::string("Could not load")+nh.resolveName( "/formation/names")));
    }
    
    return names;
}


void setReferences(ros::NodeHandle nh)
{   
    std::vector<std::string> names=getRobotNames(nh);

    for(auto name:names)
    {
        tf::Pose world_pose,reference_frame,reference_master;
       
   
       
        std::vector<float> world_pose_rpy;
        if(!nh.getParam(name+"/controller/reference",world_pose_rpy))
        {
            ROS_WARN("Could not load: %s",nh.resolveName(name+"controller/reference").c_str());
        }
        else
        {
            //Get the robots world pose
            if(world_pose_rpy.size()!=6)
            {
                throw(std::invalid_argument("Wrong number of parameter for rpy pose! 6 Expected!"));
            }
            world_pose=tf::Pose(tf::createQuaternionFromRPY(world_pose_rpy[3],world_pose_rpy[4],world_pose_rpy[5]),
                            tf::Vector3(world_pose_rpy[0],world_pose_rpy[1],world_pose_rpy[2]));


             //Call the service for setting the controller pose
            ros::ServiceClient set_reference_frame=nh.serviceClient<multi_robot_msgs::SetInitialPose>(name+"/controller/set_reference_frame");
            set_reference_frame.waitForExistence();
            multi_robot_msgs::SetInitialPose call_reference_frame;
            tf::poseTFToMsg(world_pose,call_reference_frame.request.initial_pose);
            set_reference_frame.call(call_reference_frame);

            //Same with gazebo pose/
            ros::ServiceClient set_model_state=nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
            set_model_state.waitForExistence();
            gazebo_msgs::SetModelState state;
            state.request.model_state.model_name=name;
            state.request.model_state.reference_frame="/map";
            tf::poseTFToMsg(world_pose, state.request.model_state.pose);
            set_model_state.call(state);        

            std::string master_name;
            if(nh.getParam(name+"/controller/master",master_name)) //There is a master specification
            {
                if(!master_name.empty())    //There is a none empty master name
                {
                    
                    //Get the master pose
                    std::vector<float> master_pose_rpy;
                    if(!nh.getParam(master_name+"/controller/reference",master_pose_rpy))
                    {
                        ROS_WARN("Could not load: %s",nh.resolveName(name+"/controller/reference").c_str());
                    }
                    if(master_pose_rpy.size()!=6)
                    {
                        throw(std::invalid_argument("Wrong number of parameter for rpy pose! 6 Expected!"));
                    }
                    tf::Pose master_pose(   tf::createQuaternionFromRPY(master_pose_rpy[3],master_pose_rpy[4],master_pose_rpy[5]),
                                            tf::Vector3(master_pose_rpy[0],master_pose_rpy[1],master_pose_rpy[2]));

                    //Call the service for setting the slave pose relative to the master
                    ros::ServiceClient set_master_reference=nh.serviceClient<multi_robot_msgs::SetInitialPose>(name+"/controller/set__master_reference");
                    set_master_reference.waitForExistence();

                    multi_robot_msgs::SetInitialPose call_master_reference;                    
                    tf::poseTFToMsg(master_pose.inverseTimes(world_pose),call_master_reference.request.initial_pose);
                    set_master_reference.call(call_master_reference);
                }
            }

        }

        
        
    }
}

void startPlanner(ros::NodeHandle nh)
{
    //Choose planner
    ros::NodeHandle nh_planner("planner");
    
    int type;    
    nh.getParam("/planner/type",type);
    ROS_INFO("Got parameter /planner/type: %i",type);
    switch(type)
    {
        case 0:
        {
            ROS_INFO("Starting LissajousPlanner!"); 
            planner=new LissajousPlanner(nh_planner);
            break;
        }
        case 1:
        {
            ROS_INFO("Starting Circle Planner!");  
            planner=new CirclePlanner(nh_planner);
            break;
        }
        case 2:
        {
            ROS_INFO("Starting Clicked Pose Planner!");
            planner=new ClickedPosePlanner(nh_planner,"/move_base_simple/goal");
            break;
        }
        case 3:
        {
            ROS_INFO("Starting Spiral Planner");
            planner=new Spiralplanner(nh_planner);
            break;
        }
        case 4:
        {
            ROS_INFO("Starting StepResponse Planner");
            planner=new StepResposePlanner(nh_planner);
            break;
        }
        case 5:
        {
            ROS_INFO("Starting Primitive Planner");
            planner=new PrimitivePlanner(nh_planner);
            break;
        }

        default:
        {
            ROS_ERROR("Wrong planner type choosen!");
        }
      

    }


    try
    {
        planner->setStartPose(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0)));
        planner->load();
        
        ros::Duration(2).sleep();
        bool pause;
        nh.getParam("planner/pause",pause);
        ROS_INFO("Got Parameter /planner/pause: %d",pause);
        if(!pause)
        {
            planner->start();
        }
    }
    catch(std::exception &e)
    {
        ROS_WARN("%s",e.what());
    }
}

void linkObject(ros::NodeHandle nh)
{
    std::vector<std::string> names=getRobotNames(nh);
    ros::ServiceClient attach_client=nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attach_client.waitForExistence();

    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1=names[0];
    attach.request.link_name_1=names[0]+"/base_footprint";

    attach.request.model_name_2="object";
    attach.request.link_name_2="plate_link";

    attach.request.joint_type="fixed";

    attach_client.call(attach);     
}

void linkCamera(ros::NodeHandle nh)
{
    std::vector<std::string> names=getRobotNames(nh);
    ros::ServiceClient attach_client=nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attach_client.waitForExistence();

    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1=names[0];
    attach.request.link_name_1=names[0]+"/base_footprint";

    attach.request.model_name_2="camera5";
    attach.request.link_name_2="link";

    attach.request.joint_type="fixed";

    attach_client.call(attach);

    attach.request.model_name_1=names[0];
    attach.request.link_name_1=names[0]+"/base_footprint";

    attach.request.model_name_2="camera6";
    attach.request.link_name_2="link";

    attach.request.joint_type="fixed";

    attach_client.call(attach);


}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"SystemHandler");
    ros::NodeHandle nh;

    if(argc==1)
    {
        ROS_INFO_STREAM("Specifie arguments like: "<<std::endl
                        <<"-link"<<std::endl
                        <<"-plan"<<std::endl
                        <<"-camera"<<std::endl
                        <<"-reference"<<std::endl);
        return 0;
    }

    bool linkage=false;
    bool plan=false;
    bool set_references=false;
    bool link_camera=false;

    std::vector<std::string> args(argv, argv+argc);
    for (size_t i = 1; i < args.size(); ++i) 
    {
        if (args[i] == "-link") 
        {
            linkage=true;

        }
        else if(args[i]=="-plan")
        {
            plan=true;
        }
        else if(args[i]=="-reference")
        {
            set_references=true;
        }
        else if(args[i]=="-camera")
        {
            link_camera=true;
        }
    }

    if(set_references)
    {
        ROS_INFO("Setting system references!");
        setReferences(nh);
        ROS_INFO("Setting system references done!");
    }
    if(linkage)
    {
       
        ROS_INFO("Linking object to system!");
        linkObject(nh);
         ROS_INFO("Linking object to system done!");
    }
    if(link_camera)
    {
        ROS_INFO("Linking camera to system!");
        linkCamera(nh);
    }
    if(plan)
    {
        ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
        std_srvs::Empty empty;
        pauseGazebo.call(empty); 
        ROS_INFO("Starting system planner!");
        startPlanner(nh); 
       
    }

    ros::spin();
    delete planner;

  
}