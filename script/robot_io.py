import RobotIO

# 妙算 4pin 口通信
# RobotIO.background_robot_io_4pin_auto_restart(
#     robot_port_name="/dev/ttyTHS2"
# )

# TTL 通信，同种设备相同
RobotIO.background_robot_io_usb_auto_restart(
    robot_usb_hid="USB VID:PID=1a86:7523 ",
)
