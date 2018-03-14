#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
void setupCollisionObject()//Setup for a cuboid to serve as the table.
{
    // setup planning scene diffs publisher
    ros::NodeHandle node_handle;
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::AttachedCollisionObject collision_object;
    collision_object.object.header.frame_id = "base_link";


    /* The id of the object is used to identify it. */
    collision_object.object.id = "table";

    /* Define a box to add to the world. */

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.53;
    primitive.dimensions[1] = 0.76;
    primitive.dimensions[2] = 0.92;


    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0.21;
    box_pose.position.z = 0-(0.92/2);

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(box_pose);
    collision_object.object.operation = collision_object.object.ADD;

    moveit_msgs::PlanningScene planningScene;
    planningScene.world.collision_objects.push_back(collision_object.object);
    planningScene.is_diff = true;
    planning_scene_diff_publisher.publish(planningScene);

    //std::vector<moveit_msgs::CollisionObject> collision_objects;
    //collision_objects.push_back(collision_object);

    ROS_INFO("Add an object into the world");

    //planning_scene_interface.addCollisionObjects(collision_objects);

   /* Sleep so we have time to see the object in RViz */
   sleep(2.0);
}

