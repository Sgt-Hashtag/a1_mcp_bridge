# A1 MCP Bridge

This package provides a **Model Context Protocol (MCP)** interface for the Unitree A1 robot. It bridges the gap between the low-level C++ control loop and LLM-based tools.

## Setup
1. **Dependencies**: 
   ```bash
   pip install mcp fastmcp

2. **Build**
   ```bash
   cd ~/Unitree_ws
   catkin_make
   source devel/setup.bash

## Usage
   ```bash
   roslaunch a1_mcp_bridge bridge.launch
   ```

## Features
  - ***Tools***: get_robot_telemetry, set_gait, emergency_stop.
  - ***​Newton Integration***: Syncs real-time foot forces to the Newton Solver for RL verification.
  - ***​LLM Connection***: Add the server to your claude_desktop_config.json to allow Claude to control the A1.
