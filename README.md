# zlac_can_bringup — MIMO-LADRC 完成版

ROS 2 Humble / Ubuntu 22.04 / ZLAC8015D。ブランチ: `release/mimo-ladrc-stable`。
検証済み状態遷移を持つ `zlac8015d_hardware` の同名ブランチを使用する。

## 実行に必要なもの

- `launch/zlac_can_bringup.launch.py` : `robot_state_publisher`、`ros2_control_node`、コントローラ起動
- `urdf/orange_go_2025.urdf` : モータと車輪のhardware_interface構成（MIMO-LADRCが既定）
- `config/controllers.yaml` : 200 Hz controller_manager・差動二輪制御
- `CMakeLists.txt`・`package.xml` : ROS 2パッケージ登録・リソース導入

## 初回の切替・ビルド

ROS 2制御プロセスを完全に停止し、CAN通信を他のプログラムが占有していないことを確認する。

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch release/mimo-ladrc-stable
git pull --ff-only origin release/mimo-ladrc-stable

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch release/mimo-ladrc-stable
git pull --ff-only origin release/mimo-ladrc-stable

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

## モータ起動（端末1）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

このモードはURDFの既定値にも設定済み。明示的な環境変数指定は従来どおり有効。
本launchを重複起動しない。既存のCAN watchdogを使う場合も二重起動を防止する。

## 確認（端末2）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 topic info /cmd_vel
```

`joint_state_broadcaster` と `diff_drive_controller` が `active` の後、外部の操作コントローラから `/cmd_vel` (`geometry_msgs/msg/Twist`) へ指令を送る。
機体を停止させてからlaunchをCtrl+Cで終了する。

起動ログで `MIMO_TRANSITION_FIX_20261113_R2` と `Configured mode=mimo_ladrc_torque` を確認可能。
ハードウェア内部のCSV出力 `/tmp/mimo_ladrc_track_200hz.csv` は元の制御ソースに含まれるためそのまま継続する。

**注意:** 元の研究ブランチにあったログ取得、ゲイン探索、実験launch、試験スクリプトはこの完成版に含まれない。必要時は `feat/mimo-ladrc-pid-straight-tracking` を使用する。
CAN配線、非常停止装置、許容電流・速度、車輪回転方向、他ノードからの指令競合を確認してから運転する。
