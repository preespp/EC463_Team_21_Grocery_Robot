import pyrealsense2 as rs

pipe = rs.pipeline()              # 建一个“流水线”，后面由它统一拿帧
cfg  = rs.config()                # 配置对象：告诉相机要什么流

# 要求开启两路数据流：
cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
#   - 深度流（depth）
#   - 分辨率 848×480（D435 的常用原生分辨率之一）
#   - 像素格式 z16（16 位深度原始值；后续用 get_distance 可直接得到以米计的距离）
#   - 30 FPS

cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#   - 彩色流（color）
#   - 分辨率 1280×720（清晰、帧率适中，适合给 YOLO）
#   - 像素格式 bgr8（8 位 BGR，OpenCV 直接可用）
#   - 30 FPS

profile = pipe.start(cfg)         # 真正打开相机并启动采集；如果不支持会抛异常
print("Started RealSense streams OK.")
pipe.stop()                       # 立刻停止（这里只做连通性自检）
