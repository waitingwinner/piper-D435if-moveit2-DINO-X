import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import supervision as sv
import os
import threading  # [新增] 导入线程模块
import time       # [新增] 导入时间模块
from dds_cloudapi_sdk import Config, Client, TextPrompt
from dds_cloudapi_sdk.tasks.dinox import DinoxTask

# API配置
API_TOKEN = "用自己的！"
TEXT_PROMPT = "face"

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.bridge = CvBridge()
        
        # [新增] 初始化控制参数
        self.last_process_time = self.get_clock().now()
        self.frame_interval = 5  # 采样间隔（秒）
        self.latest_rgb = None
        self.latest_depth = None
        self.lock = threading.Lock()
        
        # 订阅深度图像
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        # 订阅RGB图像
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        
        # [新增] 创建定时器
        self.create_timer(0.5, self.timer_callback)
        
        # 初始化DINO-X API
        self.config = Config(API_TOKEN)
        self.client = Client(self.config)
        self.temp_frame_path = "./temp_frame.jpg"

    # [修改] 深度回调函数
    def depth_callback(self, msg):
        try:
            with self.lock:  # [新增] 线程锁
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Depth processing error: {str(e)}")

    # [修改] RGB回调函数
    def rgb_callback(self, msg):
        try:
            with self.lock:  # [新增] 线程锁
                self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB processing error: {str(e)}")

    # [新增] 定时处理回调函数
    def timer_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_process_time).nanoseconds < 5e9:
            return

        with self.lock:
            if self.latest_rgb is None or self.latest_depth is None:
                self.get_logger().warn("等待初始帧数据...")
                return

            try:
                # 复制当前帧数据
                rgb_copy = self.latest_rgb.copy()
                depth_copy = self.latest_depth.copy()

                # 执行人脸检测
                start_time = time.time()
                center = self.detect_faces(rgb_copy)
                
                if center:
                    # 处理深度数据
                    depth_value = depth_copy[center[1], center[0]] / 1000.0
# 在 timer_callback 中修改以下输出
                    self.get_logger().info(
                        f"[深度采样] 距离: {depth_value:.2f}m | 相对坐标: ({center[0]}, {center[1]})"
                    )
                    
                    # 更新显示
                    cv2.imshow("Fusion View", rgb_copy)
                    cv2.waitKey(1)

                # 记录处理性能
                proc_time = time.time() - start_time
                self.get_logger().info(
                    f"帧处理完成 | 耗时: {proc_time:.2f}s | 下次采样: {current_time.to_msg().sec + 5}"
                )
                
            except Exception as e:
                self.get_logger().error(f"定时处理异常: {str(e)}")
            
        self.last_process_time = self.get_clock().now()

    def detect_faces(self, frame):
    # 保存临时帧并上传
        cv2.imwrite(self.temp_frame_path, frame)
        image_url = self.client.upload_file(self.temp_frame_path)
    
    # 创建检测任务
        task = DinoxTask(
        image_url=image_url,
        prompts=[TextPrompt(text=TEXT_PROMPT)]
        )
        self.client.run_task(task)
    
    # 处理检测结果
        boxes = [obj.bbox for obj in task.result.objects]
        if not boxes:
            return None

    # 获取图像尺寸
        h, w = frame.shape[:2]
        origin_x = w // 2  # 坐标系原点X
        origin_y = h // 2  # 坐标系原点Y

    # 计算原始中心点
        xmin, ymin, xmax, ymax = boxes[0]
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)
    
    # 转换为新坐标系（基于图像中心）
        new_x = center_x - origin_x  # 向右为正
        new_y = origin_y - center_y  # 向上为正
    
    # 标注中心点（保持原始坐标系绘制）
        cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), -1)
    
    # 在图像上添加坐标系说明
        cv2.putText(frame, f"Coordinate System: Center=({new_x},{new_y})", 
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # 绘制坐标系轴线
        cv2.line(frame, (origin_x, 0), (origin_x, h), (255, 0, 0), 1)  # Y轴
        cv2.line(frame, (0, origin_y), (w, origin_y), (255, 0, 0), 1)  # X轴
    
        return (new_x, -new_y)  # 返回新坐标系坐标
def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()
        if os.path.exists(node.temp_frame_path):
            os.remove(node.temp_frame_path)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()