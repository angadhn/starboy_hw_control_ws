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
geometry_msgs::PoseStamped preparePose, grabPose, placePose;

class Action
{
public:
	static int GRIP;
	static int MOTION;

	Action(int _type, int _gripperPosition, geometry_msgs::PoseStamped _EEPose)
	{
		type = _type;
		gripperPosition = _gripperPosition;
		EEPose = _EEPose;
	}

	int type;
	int gripperPosition;
	geometry_msgs::PoseStamped EEPose;
};

int Action::GRIP = 0;
int Action::MOTION = 1;

int moveItErrorSuccess = 1;

std::string moveItErrorCodeString(int val)
{
	switch(val)
	{
		case 1   : return "Success";
		case -1  : return "Planning Failed";
		case -2  : return "Invalid motion plan";
		case -3  : return "Motion plan invalidated by environment change";
		case -4  : return "Control failed";
		case -5  : return "Unable to aquire sensor data";
		case -6  : return "Timed out";
		case -7  : return "Preempted";

		case -10 : return "Start state in collision";
		case -11 : return "Start state violates path constrains";
		case -12 : return "Goal in collision";
		case -13 : return "Goal violates path constraints";
		case -14 : return "Goal constraints violated";
		case -15 : return "Invalid group name";
		case -16 : return "Invalid goal constraints";
		case -17 : return "Invalid robot state";
		case -18 : return "Invalid link name";
		case -19 : return "Invalid object name";

		case -21 : return "Frame transform failure";
		case -22 : return "Collision checking unavailable";
		case -23 : return "Robot state stale";
		case -24 : return "Sensor info stale";

		case -31 : return "No inverse kinematic solution";

		default:   return "Non specific failure";
	}
}

std::vector< Action > taskList;
moveit::planning_interface::MoveGroupInterface::Plan currentPlan;
bool currentPlannedOkay = false;
int currentTask = 0;

void planCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& nothing)
//callback to preview plan
{
    ROS_INFO("Previewing plan to target");
    try{//basically create a target_pose with some desired conditions
	
	taskList.clear();

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
	
	// move to the prepare to grab pose
	tfListener->transformPose("base_link", target_pose, preparePose);
	taskList.push_back( Action(Action::MOTION, 0, preparePose) );

	// move to the grab pose and grip the box
        target_pose.header.frame_id="box_grab_frame";
	tfListener->transformPose("base_link", target_pose, grabPose);
	taskList.push_back( Action(Action::MOTION, 0, grabPose) );
	taskList.push_back( Action(Action::GRIP, 255, grabPose) );

	// lift the box off the surface
	taskList.push_back( Action(Action::MOTION, 0, preparePose) );

	// move to the place pose and release the box
        target_pose.header.frame_id="place_frame";
	tfListener->transformPose("base_link", target_pose, placePose);
	taskList.push_back( Action(Action::MOTION, 0, placePose) );
	taskList.push_back( Action(Action::GRIP, 0, grabPose) );

	// final motion back to the arms original position
	taskList.push_back( Action(Action::MOTION, 0, move_group_ptr->getCurrentPose()) );

        move_group_ptr->setMaxVelocityScalingFactor(.25);//velocityControl
        move_group_ptr->setPoseTarget(preparePose);//assigns goal to planner
        int result = (move_group_ptr->plan(currentPlan)).val;//Equivalent to the 'plan' button in rviz
	if (result == moveItErrorSuccess)
	{
		ROS_INFO("First move planned Okay");
		currentPlannedOkay = true;
		currentTask = 0;
	}
	else
	{
		ROS_ERROR("Failed to plan first move [%s]", moveItErrorCodeString(result).c_str());
		currentPlannedOkay = false;
	}
    }
    catch (tf2::TransformException &ex) {
	ROS_ERROR("Transform error %s", ex.what());
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& markerPose)
//callback for executing the plan on real robot
{
	if (currentPlannedOkay)
	{
		// execute the current task and plan
		if (taskList[currentTask].type == Action::MOTION)
		{
			ROS_INFO("task [%d] Executing motion action.", currentTask);
			int result = move_group_ptr->execute(currentPlan).val;
			if (result == moveItErrorSuccess)
				ROS_INFO("task [%d] Executed okay.", currentTask);
			else
			{
				ROS_ERROR("task [%d] Failed to execute [%s]",
					currentTask,
					moveItErrorCodeString(result).c_str());
				return;
			}
		}

		if (taskList[currentTask].type == Action::GRIP)
		{
			ROS_INFO("task [%d] Executing gripper action.", currentTask);
			sendGripperMsg(taskList[currentTask].gripperPosition);
			ros::Duration(1.0).sleep();
		}

		// if that's the end of the task list
		if (currentTask+1 == taskList.size())
		{
			ROS_INFO("task [%d] Completed execution of all tasks", currentTask);
			currentPlannedOkay = false;
			return;
		}

		// attempt to plan next move
		++currentTask;
		if (taskList[currentTask].type == Action::MOTION)
		{
			move_group_ptr->setMaxVelocityScalingFactor(.25);//velocityControl
        		move_group_ptr->setPoseTarget(taskList[currentTask].EEPose);//assigns goal to planner
        		int result = move_group_ptr->plan(currentPlan).val;
			if (result == moveItErrorSuccess)
			{
				ROS_INFO("task [%d] Next motion task planned okay.", currentTask);
				currentPlannedOkay = true;
			}
			else
			{
				ROS_ERROR("task [%d] Failed to plan next motion task [%s]",
					currentTask,
					moveItErrorCodeString(result).c_str());
				currentPlannedOkay = false;
			}
		}

		if (taskList[currentTask].type == Action::GRIP)
		{
			currentPlannedOkay = true;
			ROS_INFO("task [%d] planned next grippper task.", currentTask);
		}

	}
	else
		ROS_WARN("Current move not planned, cannot execute it!");

    /*if(grabPlanned)
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
        move_group_ptr->setPoseTarget(placePose);//assigns goal to planner
        move_group_ptr->plan(grabPlan);//Equivalent to the 'plan' button in rviz
        move_group_ptr->execute(grabPlan);

    }*/
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
