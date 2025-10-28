import pyrealsense2 as rs
ctx = rs.context()
if len(ctx.devices) == 0:
    print("No RealSense device found.")
else:
    for d in ctx.devices:
        name = d.get_info(rs.camera_info.name)
        sn   = d.get_info(rs.camera_info.serial_number)
        usb  = d.get_info(rs.camera_info.usb_type_descriptor)  # 2.1/3.2 等
        fw   = d.get_info(rs.camera_info.firmware_version)
        print(f"Device: {name}, SN: {sn}, USB: {usb}, FW: {fw}")
