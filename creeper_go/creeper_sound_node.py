import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import asyncio
import random
import sounddevice as sd
import soundfile as sf
from pathlib import Path
from threading import Thread

class CreeperSoundPlayer:
    def __init__(self, sounds_dir="/home/agilex/creeper-sound-player/tntsounds", output_device=None):
        self.sounds_dir = Path(sounds_dir)
        self.output_device = output_device
        if output_device is not None:
            sd.default.device = output_device
        
        # 音频文件路径
        self.creeper_sounds = [
            self.sounds_dir / "mob_creeper_say1.mp3",
            self.sounds_dir / "mob_creeper_say2.mp3"
        ]
        
        self.explosion_sounds = [
            self.sounds_dir / "random_explode1.mp3",
            self.sounds_dir / "random_explode3.mp3", 
            self.sounds_dir / "random_explode4.mp3"
        ]
        
        self.pre_explosion_sound = self.sounds_dir / "creeper-pre-explosion-shaking.mp3"

    async def play_random_creeper_sound(self):
        """随机播放creeper声音"""
        sound_file = random.choice(self.creeper_sounds)
        await self._play_sound(sound_file)

    async def play_random_explosion_sound(self):
        """随机播放爆炸声音"""
        sound_file = random.choice(self.explosion_sounds)
        await self._play_sound(sound_file)

    async def play_pre_explosion_sound(self):
        """播放爆炸前摇声音"""
        await self._play_sound(self.pre_explosion_sound)

    async def _play_sound(self, sound_file):
        """异步播放音频文件"""
        def play():
            data, fs = sf.read(str(sound_file))
            # 确保音频数据格式正确
            if len(data.shape) == 1:
                data = data.reshape(-1, 1)
            sd.play(data, fs, device=self.output_device)
            sd.wait()
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, play)
    
    @staticmethod
    def list_audio_devices():
        """列出所有可用的音频输出设备"""
        return sd.query_devices()
    
class CreeperSoundNode(Node):
    def __init__(self):
        super().__init__('creeper_sound_node')

        self.get_logger().info("Creeper Sound Node 启动！")
        self.player = CreeperSoundPlayer()
        self.subscription = self.create_subscription(
            Int32,
            '/creeper/sound',
            self.sound_callback,
            10
        )

        # 启动 asyncio event loop（单独线程）
        self.loop = asyncio.new_event_loop()
        self.thread = Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def sound_callback(self, msg):
        sound_code = msg.data
        self.get_logger().info(f"接收到声音指令: {sound_code}")

        if sound_code == 1:
            asyncio.run_coroutine_threadsafe(self.player.play_random_creeper_sound(), self.loop)
        elif sound_code == 2:
            asyncio.run_coroutine_threadsafe(self.player.play_pre_explosion_sound(), self.loop)
        elif sound_code == 3:
            asyncio.run_coroutine_threadsafe(self.player.play_random_explosion_sound(), self.loop)
        else:
            self.get_logger().warn(f"未知指令: {sound_code}")


# async def main():
#     # 创建播放器实例
#     player = CreeperSoundPlayer(output_device=1)
    
#     # 播放不同类型的音效
#     print("播放随机creeper声音...")
#     await player.play_random_creeper_sound()
    
#     await asyncio.sleep(1)
    
#     print("播放爆炸前摇...")
#     await player.play_pre_explosion_sound()
    
#     await asyncio.sleep(1)
    
#     print("播放随机爆炸声音...")
#     await player.play_random_explosion_sound()

# if __name__ == "__main__":
#     asyncio.run(main())

def main(args=None):
    rclpy.init(args=args)
    node = CreeperSoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.loop.call_soon_threadsafe(node.loop.stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()