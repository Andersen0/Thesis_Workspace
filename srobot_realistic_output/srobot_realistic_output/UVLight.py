import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import usb.core
import usb.util

class UVLight(Node):
    def __init__(self):
        super().__init__('uv_light')
        self.subscription = self.create_subscription(Bool, '/sRobotTurnoffUVC', self.uvc_callback, 10)
        self.usb_dev = usb.core.find(idVendor=2109, idProduct=0x0817)  # Replace with your device's vendor and product ID
        if self.usb_dev is None:
            raise ValueError("Device not found")

    def uvc_callback(self, msg):
        if msg.data:
            self.turn_off_usb()
        else:
            self.turn_on_usb()

    def turn_off_usb(self):
        # Assuming there is a specific command or method to turn off the USB device
        if self.usb_dev:
            self.usb_dev.set_configuration()
            # Specific USB command to turn off
            print("USB Device Turned Off")

    def turn_on_usb(self):
        # Assuming there is a specific command or method to turn on the USB device
        if self.usb_dev:
            self.usb_dev.set_configuration()
            # Specific USB command to turn on
            print("USB Device Turned On")

def main(args=None):
    rclpy.init(args=args)
    uv_light = UVLight()
    rclpy.spin(uv_light)
    uv_light.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
