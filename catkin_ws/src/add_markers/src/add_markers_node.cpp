#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace
{

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
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    publisher.publish(marker);
}

}  // namespace

int main( int argc, char** argv )
{
    // Create node and publisher
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Wait until we have a subscriber
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN("Please create a subscribe to /visualization_marker");
        ros::Duration(1).sleep();
    }

    // Set pickup position
    geometry_msgs::Point pickup_position{};
    pickup_position.x = -2.F;
    pickup_position.y = 0.F;
    pickup_position.z = 0.F;

    // Set drop-off position
    geometry_msgs::Point dropoff_position{};
    dropoff_position.x = 0.F;
    dropoff_position.y = 0.F;
    dropoff_position.z = 0.F;

    publishMarker(pickup_position, visualization_msgs::Marker::ADD, marker_pub);
    ros::Duration(5).sleep();
    publishMarker(pickup_position, visualization_msgs::Marker::DELETE, marker_pub);
    ros::Duration(5).sleep();
    publishMarker(dropoff_position, visualization_msgs::Marker::ADD, marker_pub);
}
