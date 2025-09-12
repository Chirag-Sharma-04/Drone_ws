import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32, Bool

from dronekit import connect, VehicleMode
import time

print("connecting to vehicle......")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, timeout=10)

#vehicle.wait_ready('battery', 'system_status')

global arm_val

class mav_info_pub(Node):
    def __init__(self):
        super().__init__("mav_info_pub")
        self.bat_publisher = self.create_publisher(Float32, "drone/battery", 10)
        self.arm_publisher = self.create_publisher(Bool, "drone/arm_status", 10)
        self.timer = self.create_timer(1.0, self.publish_info)
        self.get_logger().info("Mav info publisher has started")
        

    def publish_info(self):
        battery_msg = Float32()
        battery_msg.data = float(vehicle.battery.level)
        self.bat_publisher.publish(battery_msg)

        arm_status = Bool()
        arm_status.data = vehicle.armed
        self.arm_publisher.publish(arm_status)

def main(args=None):
    rclpy.init(args=args)
    node= mav_info_pub()
    rclpy.spin(node)
    rclpy.shutdown()

        
