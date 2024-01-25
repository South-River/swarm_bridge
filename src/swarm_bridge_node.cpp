#include <ros/ros.h>
#include <swarm_bridge/swarm_bridge.hpp>

#include <nav_msgs/Odometry.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_bridge_node");
    ros::NodeHandle nh("~");

    SwarmBridge::Ptr swarm_bridge;
    swarm_bridge.reset(new SwarmBridge(nh));

    ros::Publisher test_pub = nh.advertise<nav_msgs::Odometry>("test", 10);
    swarm_bridge->registerCallFunc<nav_msgs::Odometry>([&](nav_msgs::Odometry msg){test_pub.publish(msg);});

    ros::Rate r(100000);
    while (ros::ok())
    {
        nav_msgs::Odometry msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        msg.child_frame_id = "0";
        swarm_bridge->sendMsg<nav_msgs::Odometry>(msg);
    
        // r.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}