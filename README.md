# Auto Swarm Bridge

A ROS package for multi-robot message transport based on zmqpp

This package is a separate package version of the `swarm_bridage` used in [CREPES3](https://github.com/fast-fire/CREPES3)

## Feature

- **HEADER Only**, easy to use
- **Hith Performance**, capable to transfer odom message higher than **1000Hz** with out fluctuation, tested on Ubuntu 18.04 / 20.04
- **Automatically** get self `ip`, and **Broadcast** self `id` and self `ip` using **UDP**.
- **Automatically Connect** with others **Under Same Network**, using **TCP** to transport messages in need
- **ROS-like** publish/subscribe API
- Capable of simulating **Network Delay**, by setting `simulation` to `true` and giving `virtual_network_delay` manually in the launch file  

## Usage

Install zmqpp first

```sh
sudo apt install libzmqpp-dev
```

See [example](src/swarm_bridge_test_node.cpp) for usage

```cpp
// Initialization
SwarmBridge::Ptr swarm_bridge(new SwarmBridge(nh));

// Register subscriber
swarm_bridge->subscribe<${YOUR_MSG_TYPE}>("${YOUR_TOPIC_NAME}", [&](${YOUR_MSG_TYPE} msg){${YOUR_CODE});

// Publish
swarm_bridge->publish<${YOUR_MSG_TYPE}>("${YOUR_TOPIC_NAME}", msg);
```
Change `${YOUR_MSG_TYPE}` into the message type you want to transfer, like `nav_msgs::Odometry` or `visualization_msgs::Marker`.
Change `${YOUR_TOPIC_NAME}` into the name of the topic, just like ros.
Implement your customized code to substitute `${YOUR_CODE}`

## Future work

- separate different groups for broadcast
- add manual mode to get rid of situations getting `ip` in unexpected networks
