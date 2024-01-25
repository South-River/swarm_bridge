#ifndef _SWARM_BRIDGE_HPP
#define _SWARM_BRIDGE_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "swarm_bridge/udp_bridge.hpp"
#include "swarm_bridge/tcp_bridge.hpp"

#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

class SwarmBridge
{
public:
  SwarmBridge(const ros::NodeHandle &nh);
  SwarmBridge(const SwarmBridge &rhs) = delete;
  SwarmBridge &operator=(const SwarmBridge &rhs) = delete;
  ~SwarmBridge();

  enum class State
  {
    Init,
    Running,
    Stop
  };

  typedef std::shared_ptr<SwarmBridge> Ptr;

  template <typename T>
  void sendMsg(const T &msg);

  template <typename T>
  void registerCallFunc(std::function<void(T)> func);

  State getState() const;
  std::map<int32_t, std::string> getIDIP() const;

private:
  ros::NodeHandle nh_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;

  UDPBridge::Ptr udp_bridge_;
  TCPBridge::Ptr tcp_bridge_;

  int self_id_;
  State state_;

  std::thread swarm_bridge_thread_;
  void swarmBridgeThread();
};

SwarmBridge::SwarmBridge(const ros::NodeHandle &nh) : spinner_(1, &callback_queue_)
{
  nh_ = nh;
  nh_.setCallbackQueue(&callback_queue_);

  udp_bridge_.reset(new UDPBridge());
  tcp_bridge_.reset(new TCPBridge());

  {
    std::string net_mode = "auto";
    nh_.getParam("net_mode", net_mode);

    std::vector<int> id_list;
    nh_.getParam("id_list", id_list);

    std::vector<std::string> ip_list;
    nh_.getParam("ip_list", ip_list);

    double udp_timeout = 10;
    nh_.getParam("udp_timeout", udp_timeout);

    udp_bridge_->setNetMode(net_mode, id_list, ip_list);
    udp_bridge_->setTimeOut(udp_timeout);
  }

  state_ = State::Init;
  self_id_ = -1;
  swarm_bridge_thread_ = std::thread([this]
                                     { swarmBridgeThread(); });
}

SwarmBridge::~SwarmBridge()
{
  ROS_ERROR("[SwarmBridge] stopping");
  spinner_.stop();
  state_ = State::Stop;
  if (swarm_bridge_thread_.joinable())
  {
    swarm_bridge_thread_.join();
  }
  ROS_ERROR("[SwarmBridge] stopped");
}

template <typename T>
void SwarmBridge::sendMsg(const T &msg)
{
  if (state_ == State::Stop)
    return;
  
  tcp_bridge_->sendMsg(msg);
}

template <typename T>
void SwarmBridge::registerCallFunc(std::function<void(T)> func)
{
  tcp_bridge_->registerCallFunc(func);
}

SwarmBridge::State SwarmBridge::getState() const
{
  return state_;
}

std::map<int32_t, std::string> SwarmBridge::getIDIP() const
{
  return udp_bridge_->getIDIP();
}

void SwarmBridge::swarmBridgeThread()
{
  // spinner_.start();

  ros::Rate rate(10);
  while (ros::ok())
  {
    switch (state_)
    {
    case State::Init:
    {
      nh_.getParam("self_id", self_id_);
      if (self_id_ != -1)
      {
        udp_bridge_->setSelfID(self_id_);
        tcp_bridge_->setSelfID(self_id_);

        state_ = State::Running;
      }
      break;
    }
    case State::Running:
    {
      tcp_bridge_->updateIDIP(udp_bridge_->getIDIP());
      break;
    }
    case State::Stop:
      return;
    default:
      break;
    }
    rate.sleep();
  }
}

#endif