#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace
{

void sendGoal(move_base_msgs::MoveBaseGoal goal, MoveBaseClient& ac,
              const std::string& start_message, const std::string& end_message)
{
    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

     // Send the goal position and orientation for the robot to reach
    ROS_INFO("%s\n", start_message.c_str());
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("%s\n", end_message.c_str());
    }
    else
    {
        ROS_INFO("Robot failed to move for some reason");
    }
}

}  // namespace

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal1;
    move_base_msgs::MoveBaseGoal goal2;

    // Define a position and orientation for the robot to reach
    goal1.target_pose.pose.position.x = -2.0;
    goal1.target_pose.pose.orientation.w = 1.0;

    goal2.target_pose.pose.position.x = 0.0;
    goal2.target_pose.pose.orientation.w = 1.0;

    sendGoal(goal1, ac, "Robot is travelling to the pick-up zone", "Robot picked up the virtual object");
    ros::Duration(5).sleep();
    sendGoal(goal2, ac, "Robot is travelling to the drop-off zone", "Robot dropped the virtual object");

    return 0;
}
