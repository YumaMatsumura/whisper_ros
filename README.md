# whisper_ros
## 動作確認環境
- Ubuntu22.04
- ROS2 Humble

## 環境セットアップ
1. 必要なモジュールをインストールする
   ```bash
   sudo apt install portaudio19-dev
   ```
   ```bash
   pip3 install pyaudio openai
   ```
   
2. ros2ワークスペースに本パッケージをクローンする
   ```bash
   mkdir -p ~/whisper_ws/src && ~/whisper_ws/src
   ```
   ```bash
   git clone https://github.com/YumaMatsumura/whisper_ros.git
   ```
   
3. ビルドする
   ```bash
   cd ~/whisper_ws
   ```
   ```bash
   colcon build
   ```
   
4. openaiのapi_keyを環境変数に設定する
   ```bash
   export OPENAI_API_KEY="XXX"
   ```
   
## 動作確認
1. whisper_rosを起動する
   ```bash
   ros2 run whisper_ros whisper_ros
   ```
   
2. サービスをコールする
   ```bash
   ros2 service call /create_transcript whisper_msgs/srv/CreateTranscript
   ```
   
3. マイクに向かって話す。一定以上の音量が観測されると録音が開始し、一定以上の時間が経過するか、沈黙状態が一定時間経過すると録音が終了する。
