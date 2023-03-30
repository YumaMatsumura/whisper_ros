import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from whisper_ros.whisper import Whisper
from whisper_msgs.srv import CreateTranscript

class WhisperROS(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.declare_parameter('max_record_time', 10)
        self.declare_parameter('silence_span', 1)
        self.declare_parameter('volume_threshold', 0.5)
        self.declare_parameter('record_file', '/path/to/file.wav')
        self.srv = self.create_service(CreateTranscript, 'create_transcript', self.create_transcript_callback)
        
    def create_transcript_callback(self, request, response):
        max_record_time = self.get_parameter('max_record_time').value
        silence_span = self.get_parameter('silence_span').value
        volume_threshold = self.get_parameter('volume_threshold').value
        record_file = self.get_parameter('record_file').value
        whisper = Whisper(api_key=os.environ["OPENAI_API_KEY"], max_record_time=max_record_time, silence_span=silence_span, volume_threshold=volume_threshold, record_file=record_file)
        
        try:
            response.transcript = whisper.voice_to_text()
            response.result = True
            self.get_logger().info("Whisper Output: " + response.transcript)
        
            return response
            
        except Exception as e:
            self.get_logger().error('whisper_ros has failed %r' & (e,))
            response.result = False
            
            return response
        
def main(args=None):
    rclpy.init(args=args)
    try:
        whisper_ros = WhisperROS()
        executor = SingleThreadedExecutor()
        executor.add_node(whisper_ros)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            whisper_ros.destroy_node()
    finally:
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
