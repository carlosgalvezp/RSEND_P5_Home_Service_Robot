#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

namespace
{

constexpr float kEpsilon = 0.2F;

nav_msgs::Odometry current_odometry{};

void odomCallback(const nav_msgs::Odometry& msg)
{
    current_odometry = msg;
}

void publishMarker(const geometry_msgs::Point& position, const int32_t action,
                   ros::Publisher& publisher)
{
    // Set our initial shape type to be a cube
    constexpr uint32_t shape = visualization_msgs::Marker::CUBE;

    // Create marker
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;

    // Set the marker action
    marker.action = action;

    // Set the pose of the marker.
    marker.pose.position = position;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    publisher.publish(marker);
}

void waitForRobotToArrive(const geometry_msgs::Point& target_position)
{
    while (std::fabs(target_position.x - current_odometry.pose.pose.position.x) > kEpsilon ||
           std::fabs(target_position.y - current_odometry.pose.pose.position.y) > kEpsilon)
    {
        ros::spinOnce();  // To update current_odometry
        ros::Duration(0.01).sleep();
    }
}

}  // namespace

int main( int argc, char** argv )
{
    // Create node and publisher
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber marker_sub = n.subscribe("/odom", 1, odomCallback);

    // Wait until we have a subscriber
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN("Please create a subscriber to /visualization_marker");
        ros::Duration(0.1).sleep();
    }

    // Set pickup position and publish marker
    geometry_msgs::Point target_position{};
    target_position.x = -8.F;
    target_position.y = -5.F;
    publishMarker(target_position, visualization_msgs::Marker::ADD, marker_pub);

    // Wait for robot to arrive
    waitForRobotToArrive(target_position);

    // Hide the marker
    publishMarker(target_position, visualization_msgs::Marker::DELETE, marker_pub);

    // Sleep 5 seconds to simulate a pickup
    ros::Duration(5).sleep();

    // Set drop-off position
    target_position.x = 1.F;
    target_position.y = 4.F;

    // Wait for robot to arrive at dropoff location
    waitForRobotToArrive(target_position);

    // Display marker again
    publishMarker(target_position, visualization_msgs::Marker::ADD, marker_pub);

    // End here
    ros::spin();
}
