#include <vector>

#include <ros/ros.h>
#include <swarm_bridge/swarm_bridge.hpp>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_bridge_node");
    ros::NodeHandle nh("~");

    // initialize
    SwarmBridge::Ptr swarm_bridge;
    swarm_bridge.reset(new SwarmBridge(nh));

    // add a subscriber, 
    // which callback function implemented by lambda a function.
    // topic name using default.
    ros::Publisher test_pub = nh.advertise<nav_msgs::Odometry>("test", 10);
    swarm_bridge->subscribe<nav_msgs::Odometry>([&](nav_msgs::Odometry msg){test_pub.publish(msg);});

    // add a subscriber with a customized topic name, 
    // it will only subscribe to publishers with same topic name.
    ros::Publisher test2_pub = nh.advertise<visualization_msgs::Marker>("test2", 10);
    swarm_bridge->subscribe<visualization_msgs::Marker>("vis", [&](visualization_msgs::Marker msg){test2_pub.publish(msg);});

    // add a ros subscriber and publish the message to swarm bridge in its callback.
    // try publish a nav_msgs::Odometry message to test it.
    ros::Subscriber test3_pub = nh.subscribe<nav_msgs::Odometry>("/test3", 10, [&swarm_bridge](nav_msgs::OdometryConstPtr msg)
    {
        ROS_INFO("Received message from test3 topic from ros and publish it to swarm bridge.");
        swarm_bridge->publish<nav_msgs::Odometry>("test3", *msg);
    });

    swarm_bridge->subscribe<nav_msgs::Odometry>("test3", [&](nav_msgs::Odometry msg)
    {
        ROS_INFO("Received message from test topic from swarm bridge.");
    });

    ros::Rate r(100);
    while (ros::ok())
    {
        // publish data using the default topic name.
        nav_msgs::Odometry msg;
        swarm_bridge->publish<nav_msgs::Odometry>(msg);

        // publish data using customized topic names.
        visualization_msgs::Marker msg2;
        swarm_bridge->publish<visualization_msgs::Marker>("vis", msg2);

        // publish data to a group of agents, whose id is 0, 1, 2.
        // visualization_msgs::Marker msg3;
        // swarm_bridge->publish<visualization_msgs::Marker>("vis", msg2, {0, 1, 2});
    
        r.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
