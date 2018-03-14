//
// Created by y3rs tr00ly and Pete Blacker on 7/1/18.
// Revised on: 25/1/18
//

#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
using namespace std;
#include "collisionObject.h"
#include "gripperControls.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

moveit::planning_interface::MoveGroupInterface *move_group_ptr;
//moveit_visual_tools::MoveItVisualTools *visualToolsPtr;
moveit::planning_interface::MoveGroupInterface::Plan grabPlan;
bool grabPlanned = false;
tf::TransformListener *tfListener;
geometry_msgs::PoseStamped preparePose, grabPose;

void planCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& nothing)
//callback to preview plan
{
    ROS_INFO("Previewing plan to target");
    try{//basically create a target_pose with some desired conditions
        geometry_msgs::PoseStamped target_pose;// Create a target_pose
        target_pose.header.frame_id="box_prepare2grab_frame";
        target_pose.header.stamp=ros::Time(0);
        target_pose.pose.orientation.x = 0;
        target_pose.pose.orientation.y = 0;
        target_pose.pose.orientation.z = 0;
        target_pose.pose.orientation.w = 1;
        target_pose.pose.position.x = 0;
        target_pose.pose.position.y = 0;
        target_pose.pose.position.z = 0;

	tfListener->transformPose("base_link", target_pose, preparePose);
        target_pose.header.frame_id="box_grab_frame";
	tfListener->transformPose("base_link", target_pose, grabPose);

        move_group_ptr->setMaxVelocityScalingFactor(.25);//velocityControl
        move_group_ptr->setPoseTarget(preparePose);//assigns goal to planner
        bool result = (move_group_ptr->plan(grabPlan)).val;//Equivalent to the 'plan' button in rviz
	if (result)
	ROS_INFO("SADSADSDADSADDASDDSASDSADSADSD");
	
	else
	ROS_ERROR("Something is better than nothing.");
        grabPlanned = true;
    }
    catch (tf2::TransformException &ex) {
	ROS_ERROR("Transform error %s", ex.what());
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& markerPose)
//callback for executing the plan on real robot
{
    if(grabPlanned)
    {
        ROS_INFO("Moving...");
        move_group_ptr->execute(grabPlan);//Equivalent to execute button in RViz's moveit plugin
        move_group_ptr->setMaxVelocityScalingFactor(.25);//velocityControl
        move_group_ptr->setPoseTarget(grabPose);//assigns goal to planner
        move_group_ptr->plan(grabPlan);//Equivalent to the 'plan' button in rviz
        move_group_ptr->execute(grabPlan);
        sendGripperMsg(255);//Close the gripper
        move_group_ptr->setPoseTarget(preparePose);//assigns goal to planner
        move_group_ptr->plan(grabPlan);//Equivalent to the 'plan' button in rviz
        move_group_ptr->execute(grabPlan);
    }
}

int main(int argc, char **argv) {
//Initialize ROS
// ^^^^^^^^^^^^^
    ros::init(argc, argv,"pickAndPlaceNode");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    tfListener = new tf::TransformListener(ros::Duration(10));
// Setup a Publisher for gripper commands
    ros::Publisher gripperPub = node_handle.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 20);
    gripperPubPtr = &gripperPub;
    initializeGripperMsg();
    sendGripperMsg(0);

    // Publish debugger pose which is offset by 10cm from the markerPose.
    //ros::Publisher debug_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("debug_pose", 1000);
    //debug_pose_pub_ptr = &debug_pose_pub;

// Setup 2 Subscribers
// ^^^^^^^^^^^^^^^^^^^
    // Subscribe to the desired pose i.e. pose of QR code from visp.
    ros::Subscriber init_sub = node_handle.subscribe("/initialpose", 1000, planCallback);
    ros::Subscriber goal_sub = node_handle.subscribe("/goal", 1000, goalCallback);


// Moveit
// ^^^^^^
// Raw pointers are frequently used to refer to the planning group for improved performance.
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group_ptr = &move_group;

//    moveit_visual_tools::MoveItVisualTools visualTools("odom_combined");
//    visualToolsPtr = &visualTools;

    ROS_INFO("Planning reference frame: %s", move_group.getPlanningFrame().c_str());
    setupCollisionObject();


    // Visualization
    // ^^^^^^^^^^^^^
    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in Rviz
//    visual_tools.loadRemoteControl();

    // open gripper
    sendGripperMsg(0);

    ros::Rate r(1); // 1 hz
    while(node_handle.ok())
    {
      r.sleep();
      ros::spinOnce();
    }
   return 0;
}
