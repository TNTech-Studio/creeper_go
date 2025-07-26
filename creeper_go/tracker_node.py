#!/usr/bin/env python3
"""
ROS2 Foxy节点：小车追踪状态机
结合SSE客户端接收YOLO检测结果，并通过状态机控制小车移动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import asyncio
import aiohttp
import json
import time
from enum import Enum, auto
import threading
import math
import random

class Event(Enum):
    RESTART = auto()
    TARGET_DETECTED = auto()
    TARGET_LOST = auto()
    INTERRUPT = auto()
    COUNTDOWN_COMPLETE = auto()

class TopState(Enum):
    IDLE = auto()
    CHASING = auto()
    EXPLOSION = auto()
    STOP = auto()

# class ChasingSubState(Enum):
#     PATH_TRACKING = auto()
#     CHECK_CONDITION = auto()

class ExplosionSubState(Enum):
    COUNTDOWN = auto()
    TRIGGER = auto()
    INTERRUPTED = auto()

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        
        # ROS2发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.creeper_sound_pub = self.create_publisher(Int32, '/creeper/sound', 10)
        
        # 状态机初始化
        self.state = TopState.IDLE
        # self.seek_substate = ChasingSubState.PATH_TRACKING
        self.explosion_substate = ExplosionSubState.COUNTDOWN
        self.timer_start = None
        self.target_offset = 0.0
        self.target_detected = False
        self.target_bbox = None
        self.target_id = None
        self.target_height = 0
        self.target_width = 0
        self.state_count = 0
        self.config_aggressive = "idle"
        self.last_rotation_second = -1
        self.last_sound_time = 0  # 上一次发送creeper声音的时间戳
        self.sound_interval = 5   # 每隔5秒发送一次
        
        # 控制参数
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.2  # rad/s
        self.distance_threshold = 1.0  # 米
        self.height_threshold = 0.9  # 百分比
        self.weight_threshold = 0.4  # 百分比
        self.offset_threshold = 100  # 像素
        self.frame_width = 640  # 相机画面宽度
        self.frame_height = 480  # 相机画面高度
        
        # SSE客户端配置
        self.server_url = self.declare_parameter('server_url', 'http://localhost:8000').value
        self.session = None
        self.running = True
        
        # 创建定时器，10Hz更新频率
        self.create_timer(0.1, self.timer_callback)
        
        # 启动异步SSE客户端
        self.sse_thread = threading.Thread(target=self._run_sse_client)
        self.sse_thread.daemon = True
        self.sse_thread.start()
        
        self.get_logger().info(f'Tracker node started, connecting to {self.server_url}')

    def _run_sse_client(self):
        """在单独线程中运行异步SSE客户端"""
        asyncio.run(self._async_sse_client())

    async def _async_sse_client(self):
        """异步SSE客户端主函数"""
        async with aiohttp.ClientSession() as session:
            self.session = session
            while self.running:
                try:
                    await self._subscribe_to_metadata()
                except Exception as e:
                    self.get_logger().error(f'SSE client error: {e}')
                    await asyncio.sleep(5)  # 重连延迟

    async def _subscribe_to_metadata(self):
        """订阅元数据流"""
        url = f"{self.server_url}/stream/metadata"
        
        try:
            async with self.session.get(url) as response:
                if response.status != 200:
                    self.get_logger().error(f'Failed to connect to metadata stream: {response.status}')
                    return
                
                self.get_logger().info('Connected to metadata stream!')
                
                async for line in response.content:
                    if not self.running:
                        break
                    
                    line = line.decode('utf-8').strip()
                    if line.startswith('data: '):
                        try:
                            data = json.loads(line[6:])
                            self._handle_metadata(data)
                        except json.JSONDecodeError as e:
                            self.get_logger().warn(f'Failed to decode metadata: {e}')
                            
        except aiohttp.ClientError as e:
            self.get_logger().error(f'Connection error: {e}')

    def _handle_metadata(self, data: dict):
        """处理接收到的检测数据"""
        try:
            # 获取追踪目标信息
            tracked_target = data.get('extra_metadata', {}).get('tracked_target', {})
            # 获取追踪目标信息
            current_state_count = data.get('state_count')
            if self.state_count is not current_state_count:
                self.handle_event(Event.RESTART)
                self.state_count = current_state_count

            self.config_aggressive = data.get('config_aggressive')
            
            if tracked_target and tracked_target.get('bbox'):
                # 目标被检测到
                self.target_detected = True
                self.target_bbox = tracked_target['bbox']
                self.target_id = tracked_target.get('target_id')
                # self.flag_chasing = tracked_target.get('require_badge', False)
                
                # 计算目标位置信息
                self._calculate_target_info()

                if self.config_aggressive != "idle":
                    # 触发目标检测事件
                    self.handle_event(Event.TARGET_DETECTED)
                                
                self.get_logger().debug(
                    f'Target detected - ID: {self.target_id}, '
                    f'Offset: {self.target_offset:.2f}'
                )
            else:
                # 目标丢失
                if self.target_detected:
                    self.target_detected = False
                    self.handle_event(Event.TARGET_LOST)
                    self.get_logger().info('Target lost')
            
        except Exception as e:
            self.get_logger().error(f'Error handling metadata: {e}')

    def _calculate_target_info(self):
        """根据边界框计算目标距离和偏移"""
        if not self.target_bbox:
            return
        
        # bbox格式: [x1, y1, x2, y2]
        x1, y1, x2, y2 = self.target_bbox
        
        # 计算目标中心
        target_center_x = (x1 + x2) / 2
        target_center_y = (y1 + y2) / 2
        
        # 计算水平偏移（像素）
        frame_center_x = self.frame_width / 2
        self.target_offset = target_center_x - frame_center_x
        
        # 估算距离（基于目标大小）
        self.target_width = x2 - x1
        self.target_height = y2 - y1
        # target_area = target_width * target_height
        
    def handle_event(self, event: Event):
        """处理状态机事件"""
        if self.state == TopState.IDLE:
            if event == Event.RESTART or event == Event.TARGET_DETECTED:
                self.get_logger().info("[IDLE] -> [SEEK]")
                self.state = TopState.CHASING
                # self.seek_substate = ChasingSubState.PATH_TRACKING

        elif self.state == TopState.CHASING:
            if event == Event.TARGET_LOST:
                self.get_logger().info("[SEEK] 目标丢失，保持在SEEK")
            elif event == Event.RESTART:
                self.get_logger().info("[SEEK] 被RESTART中断 -> [SEEK]")

        elif self.state == TopState.EXPLOSION:
            if event == Event.INTERRUPT or event == Event.RESTART:
                self.get_logger().info("[EXPLOSION] 被打断 -> [SEEK]")
                self.state = TopState.CHASING
                # self.seek_substate = ChasingSubState.PATH_TRACKING
                return

        elif self.state == TopState.STOP:
            if event == Event.RESTART:
                self.get_logger().info("[STOP] -> [IDLE]")
                self.state = TopState.IDLE

    def _handle_chasing_logic(self):
        """处理寻敌逻辑"""
        # if self.seek_substate == ChasingSubState.PATH_TRACKING:
        #     if self.target_detected:
        #         self.seek_substate = ChasingSubState.CHECK_CONDITION

        # elif self.seek_substate == ChasingSubState.CHECK_CONDITION:
        if self._target_meets_trigger_conditions():
            if self.config_aggressive == "explosion":
                self.get_logger().info("[CHASING::检查条件] 满足进入爆炸流程")
                self.state = TopState.EXPLOSION
                self.explosion_substate = ExplosionSubState.COUNTDOWN
                self.timer_start = time.time()
                msg = Int32()
                msg.data = 2
                self.creeper_sound_pub.publish(msg)
                self.get_logger().info("进入 EXPLOSION 状态，播放爆炸前摇音效（ID 2）")
            elif self.config_aggressive == "chasing":
                self.get_logger().info("[CHASING::检查条件] 满足 -> [STOP]")
                self.state = TopState.STOP
        # else:
            # self.seek_substate = ChasingSubState.PATH_TRACKING

    def _handle_explosion_logic(self):
        """处理爆炸流程逻辑"""
        if self.explosion_substate == ExplosionSubState.COUNTDOWN:
            if time.time() - self.timer_start >= 3:
                self.get_logger().info("[EXPLOSION::倒计时] 3秒完成，起爆")
                self.explosion_substate = ExplosionSubState.TRIGGER
                self.handle_event(Event.COUNTDOWN_COMPLETE)
                
        elif self.explosion_substate == ExplosionSubState.TRIGGER:
            self.get_logger().info("[EXPLOSION::起爆] 起爆完成 -> [STOP]")
            self.state = TopState.STOP
            msg = Int32()
            msg.data = 3
            self.creeper_sound_pub.publish(msg)
            self.get_logger().info("播放爆炸音效（ID 3）")

    def _target_meets_trigger_conditions(self):
        """检查是否满足触发条件"""
        return (
            # self.sonar_distance < self.distance_threshold and
            (self.target_height / self.frame_height) > self.height_threshold and
            (self.target_width / self.frame_width) > self.weight_threshold and
            abs(self.target_offset) < self.offset_threshold
            # self.config_aggressive == "explosion"
        )

    def timer_callback(self):
        """定时器回调，更新状态机并发送速度命令"""
        twist = Twist()
        
        # 根据状态执行相应逻辑
        if self.state == TopState.IDLE:
            # 空闲状态，不移动
            twist.linear.x = 0.0
            current_second = int(time.time())  # 当前时间戳的整数秒
            # 仅在 current_second 是 4 的倍数，且和上次不同的时候才发一次
            if current_second % 4 == 0 and current_second != self.last_rotation_second:
                twist.angular.z = random.uniform(-self.angular_speed, self.angular_speed)
                self.last_rotation_second = current_second
            else:
                twist.angular.z = 0.0
            
        elif self.state == TopState.CHASING:
            self._handle_chasing_logic()
            
            if self.target_detected:
                # 有目标时，追踪目标
                # 前进速度与距离成正比
                # twist.linear.x = min(self.linear_speed * self.target_distance, self.linear_speed)
                twist.linear.x = self.linear_speed

                # 角速度与偏移成正比
                angular_error = self.target_offset / self.frame_width
                twist.angular.z = -self.angular_speed * angular_error * 2.0
            else:
                # 无目标时，原地旋转搜索
                twist.linear.x = 0.0
                current_second = int(time.time())  # 当前时间戳的整数秒

                # 仅在 current_second 是 4 的倍数，且和上次不同的时候才发一次
                if current_second % 4 == 0 and current_second != self.last_rotation_second:
                    twist.angular.z = random.uniform(-self.angular_speed, self.angular_speed)
                    self.last_rotation_second = current_second
                else:
                    twist.angular.z = 0.0
            # 每隔 sound_interval 秒播放一次 creeper sound（ID 1）
            now = time.time()
            if now - self.last_sound_time > self.sound_interval:
                msg = Int32()
                msg.data = 1
                self.creeper_sound_pub.publish(msg)
                self.get_logger().info("播放 creeper sound（ID 1）")
                self.last_sound_time = now
                
        elif self.state == TopState.EXPLOSION:
            self._handle_explosion_logic()
            # 爆炸倒计时期间停止移动
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif self.state == TopState.STOP:
            # 停止状态
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # 发布速度命令
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """清理资源"""
        self.running = False
        if self.sse_thread.is_alive():
            self.sse_thread.join(timeout=2)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrackerNode()
        
        # 启动后立即进入寻敌模式
        node.handle_event(Event.RESTART)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()