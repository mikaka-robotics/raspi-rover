このレポジトリはこの[チュートリアル](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html)に基づいて作成されました。

- 依存関係の解決
```bash
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
//必要に応じて
sudo apt install ros-humble-ros-gz
```
- ビルド
```bash
cd ros2_ws
colcon build
```
- 実行
```bash
ros2 launch sam_bot_description display.launch.py
```