import pyrealsense2 as rs

pipe = rs.pipeline()
cfg  = rs.config()

cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

profile = pipe.start(cfg)
print("Started RealSense streams OK.")
pipe.stop()
