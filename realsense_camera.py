import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import threading

class RealSenseCamera:
    def __init__(self, serial_number, width=640, height=480, fps=30):
        self.serial_number = serial_number
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.image = None
        self.running = True
        
        # 初始化并启动相机
        self.init_camera()
        self.start_streaming()
        
        # 启动图像采集线程
        self.thread = threading.Thread(target=self.camera_worker, daemon=True)
        self.thread.start()
        
        # 等待预热
        time.sleep(2)

    def init_camera(self):
        """初始化相机配置"""
        # 查找设备
        ctx = rs.context()
        devices = ctx.query_devices()
        found = False
        for dev in devices:
            if dev.get_info(rs.camera_info.serial_number) == self.serial_number:
                found = True
                break
        if not found:
            raise RuntimeError(f"未找到序列号为 {self.serial_number} 的设备")

        # 启用设备
        self.config.enable_device(self.serial_number)
        
        # 配置颜色流
        self.config.enable_stream(rs.stream.color, 
                                self.width, 
                                self.height, 
                                rs.format.bgr8, 
                                self.fps)

    def start_streaming(self):
        """启动相机流"""
        try:
            profile = self.pipeline.start(self.config)
            color_sensor = profile.get_device().first_color_sensor()
            
            # 禁用自动曝光
            if color_sensor.supports(rs.option.enable_auto_exposure):
                color_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
            
            # 设置手动曝光（示例值）
            if color_sensor.supports(rs.option.exposure):
                color_sensor.set_option(rs.option.exposure, 156)
                
            # 设置手动白平衡（示例值）
            if color_sensor.supports(rs.option.white_balance):
                color_sensor.set_option(rs.option.white_balance, 3000)
                
        except Exception as e:
            raise RuntimeError(f"启动相机失败: {e}")

    def camera_worker(self):
        """图像采集线程"""
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                
                if color_frame:
                    bgr_image = np.asanyarray(color_frame.get_data())
                    self.image = cv.cvtColor(bgr_image, cv.COLOR_BGR2RGB)
                    
                time.sleep(0.01)  # 控制采集频率 ~100Hz
                    
            except Exception as e:
                print(f"[{self.serial_number}] 相机采集错误: {e}")
                time.sleep(0.5)

    def get_image(self):
        """获取最新图像"""
        return self.image.copy() if self.image is not None else None

    def close(self):
        """关闭相机资源"""
        self.running = False
        self.pipeline.stop()
        self.image = None

def main():
    # 替换为你的实际相机序列号
    BASE_CAMERA_SERIAL = "250122073394"  # 底座相机
    # WRIST_CAMERA_SERIAL = "242322071749"  # 手腕相机

    # 初始化两个相机
    base_cam = None
    wrist_cam = None
    
    try:
        base_cam = RealSenseCamera(BASE_CAMERA_SERIAL)
        # wrist_cam = RealSenseCamera(WRIST_CAMERA_SERIAL)
        
        print("✅ 相机已启动，按 's' 保存图像，按 'q' 退出")
        
        while True:
            base_img = base_cam.get_image()
            # wrist_img = wrist_cam.get_image()
            
            if base_img is not None and wrist_img is not None:
                # 显示图像
                cv.imshow('Base Camera', cv.cvtColor(base_img, cv.COLOR_RGB2BGR))
                # cv.imshow('Wrist Camera', cv.cvtColor(wrist_img, cv.COLOR_RGB2BGR))
                
                key = cv.waitKey(1)
                
                # 保存图像
                if key == ord('s'):
                    timestamp = int(time.time() * 1000) % 1000000
                    cv.imwrite(f'base_{timestamp}.jpg', cv.cvtColor(base_img, cv.COLOR_RGB2BGR))
                    # cv.imwrite(f'wrist_{timestamp}.jpg', cv.cvtColor(wrist_img, cv.COLOR_RGB2BGR))
                    print(f"📸 已保存图像: base_{timestamp}.jpg 和 wrist_{timestamp}.jpg")
                    
                # 退出
                if key == ord('q'):
                    break
                    
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        # 清理资源
        if base_cam:
            base_cam.close()
        if wrist_cam:
            wrist_cam.close()
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()