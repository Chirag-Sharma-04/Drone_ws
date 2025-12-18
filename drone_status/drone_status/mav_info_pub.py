import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32, Bool
from geometry_msgs.msg import Vector3

from dronekit import connect, VehicleMode
import time

print("connecting to vehicle......")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, timeout=10)

#vehicle.wait_ready('battery', 'system_status')

global arm_val

class mav_info_pub(Node):
    def __init__(self):
        super().__init__("mav_info_pub")
        self.bat_level_publisher = self.create_publisher(Float32, "drone/battery_level", 10)
        self.bat_voltage_publisher = self.create_publisher(Float32, "drone/battery_voltage", 10)
        self.arm_publisher = self.create_publisher(Bool, "drone/arm_status", 10)
        self.gps_position_publisher = self.create_publisher(Vector3, "drone/gps_position", 10)
        self.gps_lock_publisher = self.create_publisher(Bool, "drone/gps_lock", 10)
        self.system_status_publisher = self.create_publisher(String, "drone/system_status", 10)
        self.failsafe_publisher = self.create_publisher(Bool, "drone/failsafe", 10)
        self.timer = self.create_timer(1.0, self.publish_info)
        self.get_logger().info("Mav info publisher has started")
        

    def publish_info(self):
        battery_level_msg = Float32()
        battery_level_msg.data = float(vehicle.battery.level)
        self.bat_level_publisher.publish(battery_level_msg)

        battery_voltage_msg = Float32()
        battery_voltage_msg.data = float(vehicle.battery.voltage)
        self.bat_voltage_publisher.publish(battery_voltage_msg)

        arm_status = Bool()
        arm_status.data = vehicle.armed
        self.arm_publisher.publish(arm_status)

        # Publish GPS lock status
        gps_lock_msg = Bool()
        gps_lock_msg.data = vehicle.gps_0.fix_type >= 2  # 2 or higher means GPS lock
        self.gps_lock_publisher.publish(gps_lock_msg)

        # Publish GPS position (zeros if not locked)
        gps_position_msg = Vector3()
        if vehicle.gps_0.fix_type >= 2 and vehicle.location.global_frame is not None:
            gps_position_msg.x = vehicle.location.global_frame.lat
            gps_position_msg.y = vehicle.location.global_frame.lon
            gps_position_msg.z = vehicle.location.global_frame.alt
        else:
            gps_position_msg.x = 0.0
            gps_position_msg.y = 0.0
            gps_position_msg.z = 0.0
        self.gps_position_publisher.publish(gps_position_msg)

        # Publish system status
        system_status_msg = String()
        system_status_msg.data = str(vehicle.system_status.state)
        self.system_status_publisher.publish(system_status_msg)

        # Publish failsafe status
        failsafe_msg = Bool()
        # Check if any failsafe is active (battery, GPS, radio, etc.)
        failsafe_msg.data = hasattr(vehicle, 'last_heartbeat') and (
            getattr(vehicle, 'ekf_ok', True) == False or
            vehicle.mode.name == 'RTL' and not vehicle.armed
        )
        self.failsafe_publisher.publish(failsafe_msg)

def main(args=None):
    rclpy.init(args=args)
    node= mav_info_pub()
    rclpy.spin(node)
    rclpy.shutdown()

