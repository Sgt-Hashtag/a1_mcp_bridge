#!/usr/bin/env python3
import rospy
from mcp.server.fastmcp import FastMCP
from a1_mcp_bridge.msg import RobotStateReport
from std_msgs.msg import String

# Initialize MCP
mcp = FastMCP("Unitree_A1_Bridge")

# Internal State Storage
last_report = RobotStateReport()

def report_callback(msg):
    global last_report
    last_report = msg

@mcp.tool()
def get_robot_telemetry() -> str:
    """Returns the current physical state of the A1 including forces and orientation."""
    if not last_report.header.stamp:
        return "Error: No data received from robot yet."
    
    return (f"Foot Forces (FR, FL, RR, RL): {list(last_report.foot_forces)}\n"
            f"Orientation (Euler): {list(last_report.imu_euler)}\n"
            f"Safety Status: {'SAFE' if last_report.is_safe else 'DANGER'}")

@mcp.tool()
def set_gait(mode: str) -> str:
    """Changes the walking gait (e.g., 'trot', 'stand', 'crawl')."""
    pub = rospy.Publisher('/a1_control/command', String, queue_size=10)
    pub.publish(mode)
    return f"Gait command '{mode}' sent to Newton solver."

if __name__ == "__main__":
    rospy.init_node("a1_mcp_server")
    rospy.Subscriber("/a1_robot/full_state", RobotStateReport, report_callback)
    mcp.run()

