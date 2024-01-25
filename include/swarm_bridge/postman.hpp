#ifndef _POSTMAN_HPP
#define _POSTMAN_HPP

#include <ros/ros.h>

#include "swarm_bridge/swarm_bridge.hpp"

template <typename T>
class PostMan
{
public:
    PostMan()=delete;
    PostMan(const TCPBridge::Ptr tcp_bridge) : tcp_bridge_(tcp_bridge);
    ~PostMan()
    {
        ROS_ERROR("[SwarmBridge] PostMan(%s) stopped", typeid(T).name());
    }

public:
    void sendMsg(const T &msg);
    void registerCallFunc(std::function<void(T)> func);

private:
    TCPBridge::Ptr tcp_bridge_;
};

template <typename T>
void PostMan<T>::sendMsg(const T &msg)
{
    tcp_bridge_->sendMsg();
}

template <typename T>
void PostMan<T>::registerCallFunc(std::function<void(T)> func)
{
    tcp_bridge_->registerOdomCallFunc(func);
}

#endif