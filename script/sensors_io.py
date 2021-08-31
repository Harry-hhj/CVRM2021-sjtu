import SensorsIO

SensorsIO.background_sensors_io_auto_restart(
    # 旧哨兵：Guard
    # 新哨兵下云台： sentry-down
    # 新哨兵上云台： sentry-up
    camera_name="sentry-down",
    camera_cfg="../asset/MV-SUA133GC.Config",
    sensor_param_file="../asset/camera-param.yml",
    # imu_usb_hid="USB VID:PID=0483:5740 SNR=207C349D3056",
    # 旧哨兵： USB VID:PID=0483:5740 SNR=207C349D3056
    # 新哨兵下云台：USB VID:PID=0483:5740 SNR=205334823056
    # 新哨兵上云台：USB VID:PID=0483:5740 SNR=207E349D3056
    imu_usb_hid="USB VID:PID=0483:5740 SNR=205334823056",
    sync_period_ms=10,
)

