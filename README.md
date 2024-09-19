# Auto Swarm Bridge

A ROS package for multi-robot message transport based on zmqpp

This package is a separate package version of the `swarm_bridage` used in [CREPES3](https://github.com/fast-fire/CREPES3)

## Feature

- **HEADER Only**, easy to use
- **High Performance**, capable to transfer odom message higher than **1000Hz** with out fluctuation, tested on Ubuntu 18.04 / 20.04
- **Automatically** get self `ip`, and **Broadcast** self `id` and self `ip` using **UDP**.
- **Automatically Connect** with others **Under Same Network**, using **TCP** to transport messages in need
- **ROS-like** publish/subscribe API
- Capable of simulating **Network Delay**, by setting `simulation` to `true` and giving `virtual_network_delay` manually in the launch file  

## Usage

Install zmqpp first

```sh
sudo apt install libzmqpp-dev libspdlog-dev
```

Add this package into your workspace, and compile it, a simple demo could be done by roslaunch:
```sh
roslaunch swarm_bridge test_swarm_bridge.launch
```
or by rosmon:
```sh
mon launch swarm_bridge test_swarm_bridge.launch
```

See [example](src/swarm_bridge_test_node.cpp) for detailed usage

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

Sometimes there may meet situations that cannot automatically get `ip` in proper network.
E.g., using both wireless network and wired network, and want to transfer message under wireless network.
The priority of wired network usually higher than the wireless network, and the program will automatic get information under uninterested network.
To deal with this situation, please modify the param in launch file of `net_mode` to `manual` and set `self_ip` and `broadcast_ip` under proper network. 

## Known issues

If you are using this project as a inner class in your work, and have multi ros callback queues, there might have a little problem in 
```cpp
template <typename T>
void SwarmBridge::publish(const std::string &topic_name, const T &msg)
{
    ...
    if (this->simulation_)
    {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count() < this->virtual_network_delay_)
        {
        ros::spinOnce();
        }
    }
    ...
}
```
Set the `simulation` to `false` might help. 

## Future work

- separate different groups for broadcast