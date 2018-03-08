//
// Created by y3rs tr00ly and Pete Blacker on 13/12/17.
//

#include <ros/ros.h>/* ros/ros.h is a convenience include that
		       includes all the headers necessary to use
		       the most common public pieces of the ROS system.
		       In this code, it is used to create the pose publisher.*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

ros::Publisher *pose_pub_ptr;
ros::Publisher *debug_pose_pub_ptr;
ros::Subscriber *pose_sub_ptr;
moveit::planning_interface::MoveGroupInterface *move_group_ptr;

void desPosCallback(const geometry_msgs::PoseStamped::ConstPtr& markerPose){
    //markerPose.pose.position.x += 0.1;
    debug_pose_pub_ptr->publish(markerPose);
}

void moveArm(){
    // Getting basic information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_ERROR("Planning reference frame: %s", move_group_ptr->getPlanningFrame().c_str());
    ROS_ERROR("End effector reference frame: %s", move_group_ptr->getEndEffectorLink().c_str());
    
    //Planning to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::PoseStamped target_pose;/* Initializing a variable to provide the
					      final pose for the EE.*/


    target_pose.header.frame_id="base_link";
    target_pose.header.stamp=ros::Time::now();// + ros::Duration(2.1);
    target_pose.pose.orientation.w = 1.;
    target_pose.pose.position.x = .50;
    target_pose.pose.position.y = .0;
    target_pose.pose.position.z = .3;
    move_group_ptr->setMaxVelocityScalingFactor(.025);
    move_group_ptr->setPoseTarget(target_pose);// setting the target for move group

    pose_pub_ptr->publish(target_pose);



//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//    bool success = move_group.plan(my_plan);

//    ROS_INFO("plan: %s", move_group.plan(my_plan)? "":"FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in Rviz.
    //ROS_INFO("Visualizing plan 1 as trajectory line");

//    visual_tools.publishAxisLabeled(target_pose, "pose");
    //visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    //visual_tools.trigger();
    //visual_tools.prompt("next step");

    //if(success) {
    //    ROS_INFO_NAMED("Moving...");
    move_group_ptr->move();
    //}
    //sleep(5);
    //ros::shutdown();
}

int main(int argc, char **argv) {
/*
 * Initialize ROS. This allows ROS to do name remapping through the
 * command line -- not important for now. This is also where we specify 
 * the name of our node. Node names must be unique in a running system.
 * */
    ros::init(argc, argv,"move_group_interface_tutorial");
 /* 
  * Create a handle to this process' node. The first NodeHandle created will
  *  actually do the initialization of the node, and the last one destructed
  *   will cleanup any resources the node was using.
  *   */
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup a Publisher
    // ^^^^^^^^^^^^^^^^^
    // Publish the desired pose. This can be added as a topic in rviz.
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("desired_pose", 1000);
    pose_pub_ptr = &pose_pub;

    // Publish debugger pose which is offset by 10cm from the markerPose.
    ros::Publisher debug_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("debug_pose", 1000);
    debug_pose_pub_ptr = &debug_pose_pub;

    // Subscribe to the desired pose i.e. pose of QR code from visp.
    // As opposed to a publisher, a callback function is used.
    ros::Subscriber pose_sub = node_handle.subscribe("/visp_auto_tracker/object_position", 1000, desPosCallback);
    pose_sub_ptr = &pose_sub;

    // Moveit
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group_ptr = &move_group;

//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
    //namespace rvt = rviz_visual_tools;
    //moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    //visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in Rviz
//    visual_tools.loadRemoteControl();

    // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //text_pose.translation().z() = 1.75; // above head of PR2
//    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
//    visual_tools.trigger();
    ros::Rate r(1); // 1 hz
    while (ros::ok())
    {
      moveArm();
      r.sleep();
      ros::spinOnce();
    }
   return 0;
}
