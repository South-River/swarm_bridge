# Auto Swarm Bridge

A ROS package for multi-robot message transport based on zmqpp

This package is a separate package version of the `swarm_bridage` used in [CREPES3](https://github.com/fast-fire/CREPES3)

## Feature

- **HEADER Only**, easy to use
- **Automatically** get self `ip`, and **Broadcast** self `id` and self `ip` using **UDP**.
- **Automatically Connect** with others **Under Same Network**, using **TCP** to transport messages in need
- **ROS-like** publish/subscribe API
- Capable of simulating **Network Delay**, by setting `simulation` to `true` and giving `virtual_network_delay` manually in launch file  
- Tested on Ubuntu 18.04 / 20.04

## Usage

install zmqpp first

```sh
sudo apt install libzmqpp-dev
```

see [example](src/swarm_bridge_test_node.cpp) for usage

## Future work

- separate different groups for broadcast
- add manual mode to get rid of situations getting `ip` in unexpected networks
