#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class GpsToPath
{
public:
    GpsToPath()
    {
        // Subscribe to the GPS fix topic
        gps_sub = nh.subscribe("/gps/fix", 10, &GpsToPath::gpsCallback, this);
        
        // Subscribe to the initial 2D pose (e.g., from RViz)
        initial_pose_sub = nh.subscribe("/initial_2d", 1, &GpsToPath::setInitialPose, this);

        // Publish path messages on the "gps_path" topic
        path_pub = nh.advertise<nav_msgs::Path>("gps", 10);

        initial_pose_received = false;  // Flag for initial pose
    }

    void setInitialPose(const geometry_msgs::PoseStamped &pose_msg)
    {
        // Set the initial position and orientation from the pose message
        initial_position_x = pose_msg.pose.position.x;
        initial_position_y = pose_msg.pose.position.y;
        initial_orientation_yaw = tf::getYaw(pose_msg.pose.orientation);

        initial_pose_received = true;

        ROS_INFO("Initial pose set: X: %f, Y: %f, Yaw: %f", initial_position_x, initial_position_y, initial_orientation_yaw);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // Ensure initial pose is set before processing GPS data
        if (!initial_pose_received)
        {
            ROS_WARN("Waiting for initial pose...");
            return;
        }

        // Check if GPS has a valid fix
        if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
        {
            ROS_WARN("No GPS fix.");
            return;
        }

        // Convert GPS to UTM coordinates
        geographic_msgs::GeoPoint geo_point;
        geo_point.latitude = msg->latitude;
        geo_point.longitude = msg->longitude;
        geo_point.altitude = msg->altitude;

        geodesy::UTMPoint utm_point(geo_point);

        // Create a PoseStamped message for the path
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom"; // Set the frame id to odom
        pose_stamped.pose.position.x = utm_point.easting + initial_position_x; // Adjust for initial position
        pose_stamped.pose.position.y = utm_point.northing + initial_position_y; // Adjust for initial position
        pose_stamped.pose.position.z = msg->altitude;

        // Set orientation to the initial yaw (you can update it if needed)
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(initial_orientation_yaw);

        // Add the new pose to the path
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "odom"; // Set the frame id for the path
        path_msg.poses.push_back(pose_stamped); // Add the new pose to the path

        // Publish the path message
        path_pub.publish(path_msg);

        // Log GPS data for debugging
        ROS_INFO("Published Path Point: X: %f, Y: %f, Z: %f",
                 pose_stamped.pose.position.x,
                 pose_stamped.pose.position.y,
                 pose_stamped.pose.position.z);
    }

private:
    ros::NodeHandle nh;                   // Node handle
    ros::Subscriber gps_sub;              // Subscriber for GPS data
    ros::Subscriber initial_pose_sub;     // Subscriber for initial pose
    ros::Publisher path_pub;              // Publisher for path data

    bool initial_pose_received;            // Flag to track if initial pose has been received
    double initial_position_x;             // Initial position X
    double initial_position_y;             // Initial position Y
    double initial_orientation_yaw;        // Initial orientation (yaw)

    nav_msgs::Path path_msg;               // Path message to store the GPS points
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "gps_to_path_node");
    GpsToPath gps_to_path; // Create instance of GpsToPath class

    // Spin to process callbacks
    ros::spin();
    return 0;
}


