#include <ros/ros.h>
#include <swarm_bridge/swarm_bridge.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_bridge_node");

    SwarmBridge::Ptr swarm_bridge;
    swarm_bridge.reset(new SwarmBridge());

    ros::spin();
    return 0;
}