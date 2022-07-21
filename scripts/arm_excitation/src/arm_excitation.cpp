// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

// Our Action interface type for moving H1's head, provided as a typedef for convenience
typedef actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create a ROS action client to move H1's arm
void createArmClient(arm_control_client_Ptr& actionClient, std::string &AC)
{
    ROS_INFO("Creating action client to arm controller ...");


    actionClient.reset( new arm_control_client(AC) );

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
        ROS_DEBUG("Waiting for the right_arm_controller_action server to come up");
        ++iterations;
    }

    if ( iterations == max_iterations )
        throw std::runtime_error("Error in createArmClient: right arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move H1's arm
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal,
                        std::vector<std::string> &jNames,
                        std::vector<Eigen::VectorXd> &waypoints,
                        std::vector<Eigen::VectorXd> &waypointVels,
                        double t,
                        double tIni)
{
    // The joint names, which apply to all waypoints
    for(int i=0; i<jNames.size() ; i++)
        goal.trajectory.joint_names.push_back(jNames.at(i));

    goal.trajectory.points.resize(waypoints.size());
    for(int i=0; i<waypoints.size() ; i++ )
    {
        goal.trajectory.points[i].positions.resize(7);
        goal.trajectory.points[i].velocities.resize(7);
        for(int j=0; j<7 ; j++ )
        {
            goal.trajectory.points[i].positions[j] = waypoints.at(i)(j);
            goal.trajectory.points[i].velocities[j] = waypointVels.at(i)(j);
        }
        goal.trajectory.points[i].time_from_start = ros::Duration(t*i + t);
    }
    for(int j=0; j<7 ; j++ )
    {
        goal.trajectory.points[0].velocities[j] = 0.0;
    }
}


// Entry point
int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "run_traj_control");

    ROS_INFO("Starting run_traj_control application ...");

    // Precondition: Valid clock
    ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///     Get parameters
    ////////////////////////////////////////////////////////////////////////////////

    ROS_WARN_STREAM("ns: " << nh.getNamespace() );

    // For points
    XmlRpc::XmlRpcValue xml_waypoints;
    std::vector<Eigen::VectorXd> Vqd;

    // For velocities
    XmlRpc::XmlRpcValue xml_waypointVels;
    std::vector<Eigen::VectorXd> Vqdp;

    std::vector<std::string> jointNames;
    std::string action_name;
    Eigen::VectorXd aux;
    aux.resize(7);
    aux.setZero();
    double t=3.0;

    double t1=3.0;
    double tFactor = 1.0;


    if( nh.hasParam("action_name") )
        nh.getParam("action_name", action_name);
    else
        ROS_ERROR_STREAM("No action_name parameter.");

    if( nh.hasParam("trans_time") )
        nh.getParam("trans_time", t);
    else
        ROS_ERROR_STREAM("No trans_time parameter.");

    if( nh.hasParam("time_slowing_factor") )
        nh.getParam("time_slowing_factor", tFactor);
    else
        ROS_ERROR_STREAM("No time_slowing_factor parameter.");

    t = t*(1.0/tFactor);

    if( nh.hasParam("initial_spline_time") )
        nh.getParam("initial_spline_time", t1);
    else
        ROS_ERROR_STREAM("No initial_spline_time parameter. Defaul value 3.0");

    ////////////////////////////////// Points

    if( nh.hasParam("way_points") )
        nh.getParam("way_points", xml_waypoints);
    else
        ROS_ERROR_STREAM("No way_points parameter.");

    for( int i=0; i< xml_waypoints.size(); i++ )
    {
        if( xml_waypoints[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
            ROS_ERROR("waypoint[%d] is not a list", i);
        else
            if( xml_waypoints[i].size() != 7 )
                ROS_ERROR("waypoint[%d] has no 7 elements", i);
            else
                for(int j=0 ; j<xml_waypoints[i].size() ; j++ )
                    aux(j) = xml_waypoints[i][j];
        Vqd.push_back(aux);
    }

    //    for( int i=0; i< Vqd.size(); i++ )
    //        ROS_INFO_STREAM("waypoint " << i << ": " << Vqd.at(i).transpose() );

    ////////////////////////////////// Velocities

    if( nh.hasParam("way_point_velocities") )
        nh.getParam("way_point_velocities", xml_waypointVels);
    else
        ROS_ERROR_STREAM("No way_point_velocities parameter.");

    for( int i=0; i< xml_waypointVels.size(); i++ )
    {
        if( xml_waypointVels[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
            ROS_ERROR("waypoint[%d] is not a list", i);
        else
            if( xml_waypointVels[i].size() != 7 )
                ROS_ERROR("waypoint[%d] has no 7 elements", i);
            else
                for(int j=0 ; j<xml_waypointVels[i].size() ; j++ )
                {
                    aux(j) = xml_waypointVels[i][j];
                    aux(j) *= tFactor;
                }
        Vqdp.push_back(aux);
    }

    //    for( int i=0; i< Vqdp.size(); i++ )
    //        ROS_INFO_STREAM("waypoint velocities " << i << ": " << Vqdp.at(i).transpose() );

    if(Vqdp.size() != Vqd.size())
    {
        ROS_ERROR_STREAM("Number of waypoints is different than number of velocities");
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("\n\n  Waypoints: \n\n");

    for( int i=0; i< Vqdp.size(); i++ )
        ROS_INFO_STREAM("q: " << Vqd.at(i).transpose() << "\t qp: " << Vqdp.at(i).transpose() );


    ////////////////////////////////// Joint Names
    if( nh.hasParam("joints") )
        nh.getParam("joints", jointNames);
    else
        ROS_ERROR_STREAM("No joints parameter.");

    // for( int i=0; i< jointNames.size(); i++ )
    //     ROS_INFO_STREAM("joint " << i << ": " << jointNames.at(i) );

    ////////////////////////////////////////////////////////////////////////////////
    ///     Execute motions
    ////////////////////////////////////////////////////////////////////////////////


    // Create an arm controller action client to move the H1's arm
    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient,action_name);


    // Generates the goal for the H1's arm
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    waypoints_arm_goal(arm_goal,jointNames,Vqd, Vqdp,t, t1);

    // Sends the command to start the given trajectory 1s from now
    arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

    ArmClient->sendGoal(arm_goal);
    // Wait for trajectory execution
    int counter = 0;

    while(ros::ok()) 
    {
        ROS_INFO_STREAM("counter + 1");
        //if !(ArmClient->getState().isDone())
        ROS_INFO_STREAM("counter + 1");
        counter = counter + 1;
        if (counter > 3){
            ROS_INFO_STREAM("Interruption & cancelling the goal ...");
            break; 
            ArmClient->cancelGoalsAtAndBeforeTime(ros::Time::now());
            ros::Duration(10).sleep();
        }
        // ros::Duration(0.1).sleep(); // sleep for four seconds
    }

    // while(!(ArmClient->getState().isDone()) && ros::ok())
    // {
    //     ROS_INFO_STREAM("counter + 1");
    //     counter = counter + 1;
    //     if (counter > 3){
    //         ROS_INFO_STREAM("Interruption & cancelling the goal ...");
    //         //ArmClient->cancelGoal();
    //     }
    //     // ros::Duration(0.1).sleep(); // sleep for four seconds
    // }

    return EXIT_SUCCESS;
}
