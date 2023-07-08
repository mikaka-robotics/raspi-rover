# Raspi-Rover

![rover_gif](https://github.com/mikaka-robotics/raspi-rover/assets/45257346/d5421312-f1be-49ac-8b68-002ba3b1d22b)

このレポジトリは開発中です。動作の保証はありませんのでご注意ください。

このレポジトリはAIロボット部の開発するロボットについてのレポジトリです。
以下のような情報を含みます。
- ロボットの車体の設計図
- ロボットの回路図
- ロボットの制御プログラム
- ロボットの移動指令プログラム

TO DO:
- [ ] ロボットの経路追従プログラム
- [ ] ロボットの経路作成プログラム
- [ ] ロボットのマッピングのプログラム
- [ ] ロボットの自己位置推定のプログラム

## Getting Started
### 組み立て

### arduino
Arduino IDEをインストール。

### ros2
以下の環境で実行を確認しています。他の環境での実行は保証されません。
- Ubuntu 22.04
- ROS2 Humble
  
ロボットは指令を受けて姿勢の指令値を車輪の速度の指令値に変換し、その指令値に基づいて制御部でロボットの車輪の速度を制御しています。

なお、ここでいう制御部は先ほどArduino IDEでESP32にビルドした.inoファイルを指しています。また、指令部は`raspi_ws/src/raspi_rover/raspi_rover/twist_to_radsec_diff.py`を指しています。

指令部はROS 2のTwist型の指令を受け取り、逆運動学で各車輪への指令値に変換した後、制御部に向けて二つの車輪の角速度を送信します。制御部は指令値に基づいて車輪の角速度(rad/sec)を計算しています。

図にすると以下のようになります。

![ros2_rover_cmd](https://github.com/mikaka-robotics/raspi-rover/assets/45257346/d5e42860-2475-424a-b0aa-d028ec99d878)


以下は実行環境を作成するためのチュートリアルです。

#### Ubuntu22.04をraspberry piにインストールする
Raspberry Piである必要はありませんが、Ubuntu20.04が動作するPCにUbuntu20.04をインストールします。
```
ここに色々入れる（前書いたcodimd）
```
#### ROS2 Humbleをインストールし、依存関係をインストールする
TODO

#### micro-ROSをインストールし、ESP32との通信を確認する
TODO

#### ros2-teleop-twist-keyboardでキーボードから指令値を出す
TODO

#### 指令部を立ち上げ、Twist型の指令値を変換し、各車輪の速度へ変換してPublishする
TODO
