#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_listener.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> graspActionClient;
graspActionClient *gripper_;
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction> moveActionClient;
moveActionClient *move_;

static const std::string PLANNING_GROUP = "panda_arm";
namespace rvt = rviz_visual_tools;

void openGripper()
{
  franka_gripper::MoveGoal goal;
  if(!move_->waitForServer(ros::Duration(1.0))) {
    ROS_ERROR_STREAM("Could not find graspgripper");
  }
  goal.speed = 0.1;
  goal.width = 0.08;
  move_->sendGoal(goal);
  move_->waitForResult(ros::Duration(10));
  if(move_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Open Griper");
  }

}

void closedGripper()
{
  if(!gripper_->waitForServer(ros::Duration(1.0))) {
    ROS_ERROR_STREAM("Could not find graspgripper");
  }
  franka_gripper::GraspGoal goal;
  goal.force = 60;
  goal.speed = 0.1;
  goal.width = 0.01;
//  gripper->sendGoalAndWait(goal,ros::Duration(10));
  gripper_->sendGoal(goal);
  gripper_->waitForResult(ros::Duration(10));
//  gripper->waitForResult(ros::Duration(10));
  if(gripper_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Close Griper");
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  gripper_ = new graspActionClient("/franka_gripper/grasp",true);
//  closedGripper();
  move_ = new moveActionClient("/franka_gripper/move",true);
//  openGripper();

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setMaxVelocityScalingFactor(0.2);
  tf::TransformListener listener;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  ROS_INFO_NAMED("move_group", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("move_group", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  tf::StampedTransform transform;
    try{
      listener.lookupTransform("/panda_link0", "/object",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  printf("x = %lf\n",transform.getOrigin().x());
  printf("Y = %lf\n",transform.getOrigin().y());
  printf("Z = %lf\n",transform.getOrigin().z());
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  geometry_msgs::Pose target_pose1;
  tf2::Quaternion orientation;
  orientation.setRPY(-1.5707 , -0.62, 0);
  target_pose1.orientation = tf2::toMsg(orientation);
  target_pose1.position.x = transform.getOrigin().x();
  target_pose1.position.y = transform.getOrigin().y()-0.15;
  target_pose1.position.z =transform.getOrigin().z();
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("move_group", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("move_group", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //MOVE
  move_group.move();
  openGripper();

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation = target_pose1.orientation;
  target_pose2.position.x = transform.getOrigin().x();
  target_pose2.position.y = transform.getOrigin().y() - 0.073;
  target_pose2.position.z = transform.getOrigin().z();
  move_group.setPoseTarget(target_pose2);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  success = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);

  move_group.move();
  
  closedGripper();

  geometry_msgs::Pose target_pose3;
  tf2::Quaternion orientation1;
  orientation1.setRPY(3.1415 , 0 , 0);
  target_pose3.orientation = tf2::toMsg(orientation1);
  target_pose3.position.x = 0.35;
  target_pose3.position.y = 0;
  target_pose3.position.z = 0.40;
  move_group.setPoseTarget(target_pose3);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

  success = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.publishAxisLabeled(target_pose3, "pose3");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan3.trajectory_, joint_model_group);

  move_group.move();

  exit(0);
 
}
