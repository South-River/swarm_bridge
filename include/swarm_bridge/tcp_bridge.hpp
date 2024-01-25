#ifndef _TCP_BRIDGE_HPP
#define _TCP_BRIDGE_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <any>
#include <shared_mutex>

// #include <sensor_msgs/Imu.h>

// #include <crepes_msgs/imu.h>
// #include <crepes_msgs/cloud_d.h>
// #include <crepes_msgs/cloud_xyz.h>

#include "reliable_bridge.hpp"

class TCPBridge
{
public:
  TCPBridge(){
      // bridge_.reset(new ReliableBridge(self_id_, 100000));
  };
  TCPBridge(const TCPBridge &rhs) = delete;
  TCPBridge &operator=(const TCPBridge &rhs) = delete;
  ~TCPBridge()
  {
    ROS_ERROR("[SwarmBridge] [TCPBridge] stopping");
    ROS_ERROR("[SwarmBridge] [TCPBridge] stopped");
  };

  void setSelfID(int id)
  {
    std::unique_lock<std::shared_mutex> lock1(id_mutex_);
    std::unique_lock<std::shared_mutex> lock2(bridge_mutex_);

    if (self_id_ == id)
      return;
    self_id_ = id;

    // bridge_->StopALL();
    bridge_.reset(new ReliableBridge(self_id_, 100000));
  };

  // void registerImuCallFunc(std::function<void(crepes_msgs::imu)> func)
  // {
  //   imuCallFunc_ = func;
  // };

  // void registerUwbCallFunc(std::function<void(crepes_msgs::cloud_d)> func)
  // {
  //   uwbCallFunc_ = func;
  // };

  // void registerCamCallFunc(std::function<void(crepes_msgs::cloud_xyz)> func)
  // {
  //   camCallFunc_ = func;
  // };

  void updateIDIP(std::map<int32_t, std::string> map)
  {
    std::shared_lock<std::shared_mutex> lock1(id_mutex_);
    std::shared_lock<std::shared_mutex> lock2(map_mutex_);
    std::unique_lock<std::shared_mutex> lock3(bridge_mutex_);

    if (self_id_ == -1)
    {
      ROS_WARN("[SwarmBridge] [TCPBridge] self ID not set");
      return;
    }

    for (auto it : id_ip_map_)
    {
      auto iter = map.find(it.first);
      if (iter != map.end())
      {
        continue;
      }
      ROS_WARN("[SwarmBridge] [TCPBridge] delete ID IP map: %d %s", it.first, it.second.c_str());
      // bridge_->register_callback(it.first, typeid(crepes_msgs::imu).name(), [](int ID, ros::SerializedMessage &m) {});
      // bridge_->register_callback(it.first, typeid(crepes_msgs::cloud_d).name(), [](int ID, ros::SerializedMessage &m) {});
      // bridge_->register_callback(it.first, typeid(crepes_msgs::cloud_xyz).name(), [](int ID, ros::SerializedMessage &m) {});
      bridge_->delete_bridge(it.first);
    }

    for (auto it : map)
    {
      auto iter = id_ip_map_.find(it.first);
      if (iter != id_ip_map_.end() && id_ip_map_[it.first] == it.second)
      {
        continue;
      }

      if (it.first == self_id_)
      {
        continue;
      }
      ROS_WARN("[SwarmBridge] [TCPBridge] update ID IP map: %d %s", it.first, it.second.c_str());
      bridge_->update_bridge(it.first, it.second);
    //   bridge_->register_callback(it.first, typeid(crepes_msgs::imu).name(), [this](int ID, ros::SerializedMessage &m)
    //                              { bridge_callback<crepes_msgs::imu>(ID, m, imuCallFunc_); });
    //   bridge_->register_callback(it.first, typeid(crepes_msgs::cloud_d).name(), [this](int ID, ros::SerializedMessage &m)
    //                              { bridge_callback<crepes_msgs::cloud_d>(ID, m, uwbCallFunc_); });
    //   bridge_->register_callback(it.first, typeid(crepes_msgs::cloud_xyz).name(), [this](int ID, ros::SerializedMessage &m)
    //                              { bridge_callback<crepes_msgs::cloud_xyz>(ID, m, camCallFunc_); });
    }

    id_ip_map_ = map;
  };

  template <typename T>
  int sendMsg(T msg)
  {
    std::shared_lock<std::shared_mutex> lock1(map_mutex_);
    std::shared_lock<std::shared_mutex> lock2(id_mutex_);
    std::unique_lock<std::shared_mutex> lock3(bridge_mutex_);
    if (self_id_ == -1)
    {
      return -1;
    }

    int err_code = 0;
    for (auto it : id_ip_map_) // Only send to all devices.
    {
      if (it.first == self_id_) // skip myself
      {
        continue;
      }
      err_code += bridge_->send_msg_to_one(it.first, typeid(T).name(), msg);
      if (err_code < 0)
      {
        // ROS_WARN("[SwarmBridge] [TCPBridge] send error %s !!", typeid(T).name());
      }
    }
    return err_code;
  };

private:
  std::map<int, std::string> id_ip_map_; // id -> ip
  mutable std::shared_mutex map_mutex_;

  int self_id_ = -1;
  mutable std::shared_mutex id_mutex_;

  std::unique_ptr<ReliableBridge> bridge_;
  mutable std::shared_mutex bridge_mutex_;

  // std::function<void(crepes_msgs::imu)> imuCallFunc_;
  // std::function<void(crepes_msgs::cloud_d)> uwbCallFunc_;
  // std::function<void(crepes_msgs::cloud_xyz)> camCallFunc_;

  template <typename T>
  void bridge_callback(int ID, ros::SerializedMessage &m, std::function<void(T)> callFunc)
  {
    T msg;
    ros::serialization::deserializeMessage(m, msg);
    if (callFunc)
    {
      callFunc(msg);
    }
  }
};

#endif