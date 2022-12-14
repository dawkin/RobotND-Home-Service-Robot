#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>


// Pickup location
float pickup[3] =  {1.5, 0.0, 1.0};
// Dropoff location
float dropoff[3] =  {-7.7, -2.6, 1.0};
// Thershold
float thresh[2] = {0.3, 0.05};

uint8_t goal_state = 0;

void goalCallback(const std_msgs::UInt8::ConstPtr& msg)
{   
  goal_state = msg->data;
  return;
}

int main( int argc, char** argv )
{
  ROS_INFO("Main");
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/goal_state", 1, goalCallback);
  // Boolean values
  bool done = false;
  bool pickup_displayed = false;
  bool pickup_done = false;
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    ros::spinOnce();
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickup[0];
    marker.pose.position.y = pickup[1];
    marker.pose.position.z = 0;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pickup[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
   if (goal_state == 0 && !pickup_displayed)
   {
     marker_pub.publish(marker);
     ROS_INFO("Pick-up marker displayed");
     pickup_displayed = true;
   }
   else if (goal_state == 1 && ! pickup_done)
   {
     marker.action = visualization_msgs::Marker::DELETE;
     sleep(5);
     marker_pub.publish(marker);
     ROS_INFO("Pick-up marker removed");
     pickup_done = true ;
   }
   else if(goal_state == 2)
   {
    marker.pose.position.x = dropoff[0];
    marker.pose.position.y = dropoff[1];
    marker.pose.orientation.w = dropoff[2];;
    marker.action = visualization_msgs::Marker::ADD;
    // Wait for 5 seconds before publishing the marker
    sleep(5);
    marker_pub.publish(marker);
    ROS_INFO("Drop off marker displayed");
    done = true;
   }
   if (done)
   {  
      sleep(10);
      return 0;
   }  
    
  }
  return 0;
}