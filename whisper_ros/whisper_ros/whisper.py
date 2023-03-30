import os
import pyaudio
import wave
import openai
import time
import numpy as np

class Whisper:
    def __init__(self, api_key, max_record_time=10, silence_span=1, volume_threshold=0.5, record_file='record.wav'):
        self.api_key = api_key
        self.max_record_time = max_record_time
        self.volume_threshold = volume_threshold
        self.silence_span = silence_span
        self.record_file = record_file
        
    def record(self):
        try:
            # PyAudioオブジェクトを初期化
            audio = pyaudio.PyAudio()

            # マイクからの音声を録音
            stream = audio.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=1024)
            frames = []
            recording = False
            silence = False
            start_time = time.time()

            # 録音
            while True:
                data = stream.read(1024)
                x = np.frombuffer(data, dtype="int16") / 32768.0
            
                if recording:
                    frames.append(data)
                
                    if silence:
                        if x.max() > self.volume_threshold:
                            silence = False
                            print("Speak state")
                    
                        # 一定以下の音量が一定時間経過したら録音終了
                        if time.time() - start_time > self.silence_span:
                            print("Recodring finished")
                            break
                    else:
                        if x.max() < self.volume_threshold:
                            silence = True
                            start_time = time.time()
                            print("Silence state")
                
                
                    # 一定時間経過したら録音終了
                    if len(frames) > 44100 / 1024 * self.max_record_time:
                        print("Recodring finished")
                        break
                
                else:
                    # 一定以上の音量が入力されたら録音開始
                    if x.max() > self.volume_threshold:
                        print("Recording started")
                        print("Speak state")
                        recording = True
                
            # 録音した音声をファイルに保存します
            wave_file = wave.open(self.record_file, 'wb')
            wave_file.setnchannels(1)
            wave_file.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
            wave_file.setframerate(44100)
            wave_file.writeframes(b''.join(frames))
            wave_file.close()
            
        except Exception as e:
            raise
        
    def create_transcript(self):
        # APIキーの設定
        openai.api_key = self.api_key
        
        try:
            # whisperにテキスト化してもらう
            audio_file = open(self.record_file, "rb")
            transcript = openai.Audio.transcribe("whisper-1", audio_file)
        
            return transcript["text"]
            
        except Exception as e:
            raise
        
    def remove_record_file(self):
        try:
            os.remove(self.record_file)
            
        except Exception as e:
            raise
        
    def voice_to_text(self):
        try:
            self.record()
            text = self.create_transcript()
            self.remove_record_file()
            print("Whisper Output: " + text)
        
            return text
            
        except Exception as e:
            raise
