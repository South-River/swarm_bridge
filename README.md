# Auto Swarm Bridge

A ROS package for multi-robot message transport based on zmqpp

This package is a separate package version of the `swarm_bridage` used in [CREPES3](https://github.com/fast-fire/CREPES3)

## Feature

- Automatically get self `ip`, use UDP to broadcast self `id` and self `ip`, and receive from others to discover other robots in the current network environment
- Automatically connect with others, use TCP to transport messages in need
- ROS-like publish/subscribe API
- Tested on ubuntu20.04 and ubuntu18.04

## Usage

install zmqpp first

```sh
sudo apt install libzmqpp-dev
```

see [example](src/swarm_bridge_test_node.cpp) for usage

## Future work

- separate different groups for broadcast
- add manual mode to get rid of situations getting `ip` in unexpected networks
