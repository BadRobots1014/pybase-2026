"""
This file defines constants related to your robot.  These constants include:

 * Physical constants (exterior dimensions, wheelbase)

 * Mechanical constants (gear reduction ratios)

 * Electrical constants (current limits, CAN bus IDs, roboRIO slot numbers)

 * Operation constants (desired max velocity, max turning speed)

 * Software constants (USB ID for driver joystick)

 * Vision constants (defining cameras, enabling vision poses)
"""

from collections import namedtuple

kraken_bot = False

if kraken_bot:
    from robots import KrakenBot as Bot
else:
    from robots import NeoBot as Bot

# Physical constants
PHYS = Bot.PHYS

# Mechanical constants
MECH = Bot.MECH

# Electrical constants
ELEC = Bot.ELEC

# Operation constants
OP = Bot.OP

# Software constants
SW = Bot.SW

# Vision constants
vi_data = {}
VI = namedtuple("Data", vi_data.keys())(**vi_data)

# Simulation constants
sim_data = {
    # Initial robot position for simulation
    "initial_x": 2.0,  # meters from left edge of field
    "initial_y": 4.0,  # meters from bottom edge of field
    "initial_heading_deg": 0.0,  # degrees (0 = facing +X/right)
    # AprilTag specifications (FRC standard)
    "april_tag_physical_size": 6.5
    * 0.0254,  # 6.5 inches in meters (outer black square)
    # Camera detection resolution limits
    "minimum_pixels_per_tag": 10,  # Minimum tag size for reliable detection at distance
    # Motion penalty constants (higher = detection range drops faster with speed)
    "motion_blur_constant": 1.5,  # Meters of range lost per 1 m/s of linear velocity
    "angular_blur_constant": 2.0,  # Meters of range lost per 1 rad/s of angular velocity
    "rolling_shutter_penalty": 1.0,  # Additional range penalty for rolling shutter cameras during linear motion
    # Rolling shutter hard distortion limits
    "max_angular_distortion_rad": 0.17,  # Maximum rotation during readout (~10°) before detection fails
    # Tag visibility constraints
    "max_tag_viewing_angle_rad": 1.047,  # Maximum off-axis angle to see tag (~60°, ±60° cone)
    # Camera simulation parameters (adjust to match your actual camera)
    "camera_horizontal_fov_deg": 70.0,  # Camera horizontal field of view (degrees)
    "camera_horizontal_pixels": 1280,  # Camera horizontal resolution (pixels)
    "camera_frame_rate": 30.0,  # Camera frame rate (fps)
    "camera_latency_ms": 35.0,  # Average processing + network latency (milliseconds)
    "camera_is_rolling_shutter": True,  # True for rolling shutter, False for global shutter
    # Vision measurement uncertainty (for Kalman filter)
    "vision_base_std_dev": 0.25,  # Base standard deviation (meters) at 0 distance/perfect conditions
    "vision_multi_tag_improvement": True,  # Whether multiple tags improve confidence
}
SIM = namedtuple("Sim", sim_data.keys())(**sim_data)
