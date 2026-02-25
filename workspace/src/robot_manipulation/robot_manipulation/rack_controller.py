import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from smbus2 import SMBus, i2c_msg

from robot_interfaces.action import MoveRack


STATE_IDLE = 0
STATE_MOVING = 1
STATE_DONE = 2
STATE_ERROR = 3


class RackActionServer(Node):

    def __init__(self):
        super().__init__("rack_action_server")

        # ---------- PARAMETERS ----------
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_addr", 0x13)

        self.declare_parameter("shelf1_mm", 0.0)
        self.declare_parameter("shelf2_mm", 120.0)
        self.declare_parameter("shelf3_mm", 240.0)

        self.bus_id = self.get_parameter("i2c_bus").value
        self.addr = self.get_parameter("i2c_addr").value

        self.shelf_map = {
            1: self.get_parameter("shelf1_mm").value,
            2: self.get_parameter("shelf2_mm").value,
            3: self.get_parameter("shelf3_mm").value,
        }

        self.bus = SMBus(self.bus_id)

        # ---------- ACTION SERVER ----------
        self._action_server = ActionServer(
            self,
            MoveRack,
            "move_rack",
            execute_callback=self.execute_callback,
        )

        self.get_logger().info("Rack Action Server Ready")

    # -------------------------------------------------------
    # I2C HELPERS
    # -------------------------------------------------------

    def send_height(self, height_mm: float):
        data = struct.pack("<f", height_mm)
        msg = i2c_msg.write(self.addr, data)
        self.bus.i2c_rdwr(msg)

    def read_state(self):
        msg = i2c_msg.read(self.addr, 1)
        self.bus.i2c_rdwr(msg)
        return list(msg)[0]

    # -------------------------------------------------------
    # ACTION EXECUTION
    # -------------------------------------------------------

    async def execute_callback(self, goal_handle):

        shelf = goal_handle.request.shelf_level

        if shelf not in self.shelf_map:
            goal_handle.abort()
            result = MoveRack.Result()
            result.success = False
            result.message = "Invalid shelf"
            return result

        height = self.shelf_map[shelf]

        self.get_logger().info(
            f"Move rack -> Shelf {shelf} ({height} mm)"
        )

        try:
            self.send_height(height)
        except Exception as e:
            goal_handle.abort()
            result = MoveRack.Result()
            result.success = False
            result.message = str(e)
            return result

        feedback_msg = MoveRack.Feedback()
        feedback_msg.target_height_mm = float(height)

        timeout = 25.0
        start = time.time()

        while True:

            state = self.read_state()

            feedback_msg.state = state
            goal_handle.publish_feedback(feedback_msg)

            if state == STATE_DONE:
                goal_handle.succeed()
                result = MoveRack.Result()
                result.success = True
                result.message = "Rack motion complete"
                return result

            if state == STATE_ERROR:
                goal_handle.abort()
                result = MoveRack.Result()
                result.success = False
                result.message = "ESP32 error"
                return result

            if time.time() - start > timeout:
                goal_handle.abort()
                result = MoveRack.Result()
                result.success = False
                result.message = "Timeout"
                return result

            time.sleep(0.05)


def main():
    rclpy.init()
    node = RackActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()