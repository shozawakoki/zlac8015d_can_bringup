# zlac8015d_can_bringup

ZLAC8015D モータドライバを搭載した対向二輪型移動ロボット向けの **ROS 2 bringup パッケージ**です。ROS 2 パッケージ名は `zlac_can_bringup` です。

`robot_state_publisher`、`ros2_control_node`、`joint_state_broadcaster`、`diff_drive_controller` を起動し、別リポジトリの [zlac8015d_hardware](https://github.com/shozawakoki/zlac8015d_hardware) をハードウェアプラグインとして読み込みます。

> 本 README は `main` ブランチの実ファイルに基づいています。起動対象は `launch/zlac_can_bringup.launch.py`、設定は `config/controllers.yaml` と `urdf/orange_go_2025.urdf` です。CAN 速度やモータ配線などの機体固有条件は、実機で確認してください。

## 動作環境

- Ubuntu 22.04 / ROS 2 Humble
- `ros2_control`、`ros2_controllers`、`robot_state_publisher`
- Linux SocketCAN（初期設定 `can0`）
- ZLAC8015D モータドライバ
- [`zlac8015d_hardware`](https://github.com/shozawakoki/zlac8015d_hardware)（同一 ROS 2 ワークスペース内）

必要な ROS 2 パッケージなどを導入する例：

```bash
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-robot-state-publisher python3-colcon-common-extensions can-utils
```

## ファイル構成

```text
zlac8015d_can_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── zlac_can_bringup.launch.py
├── config/
│   └── controllers.yaml
└── urdf/
    └── orange_go_2025.urdf
```

| ファイル | 役割 |
| --- | --- |
| `launch/zlac_can_bringup.launch.py` | ロボット状態配信、ハードウェア、2つのコントローラの起動 |
| `config/controllers.yaml` | 差動二輪の車輪寸法、更新周期、オドメトリ設定 |
| `urdf/orange_go_2025.urdf` | 車輪ジョイント、ROS 2 制御インターフェース、CAN・LADRC 設定 |
| `CMakeLists.txt` / `package.xml` | `ament_cmake` によるビルドとインストール |

この `main` ブランチには、旧 URDF、CAN 再起動用 watchdog、モデル同定用スクリプトは含まれていません。

## インストール・ビルド

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/shozawakoki/zlac8015d_hardware.git
git clone https://github.com/shozawakoki/zlac8015d_can_bringup.git
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

すでにクローン済みの場合は、重複して `git clone` せず、各リポジトリの `main` を更新してからビルドしてください。`package.xml` では依存関係がすべて宣言されていないため、上記の ROS 2 実行時依存パッケージは別途インストールする必要があります。

## CAN インターフェースの確認

```bash
ip -details link show can0
candump can0
```

`candump` は受信確認後 `Ctrl+C` で終了します。CAN のビットレート、インターフェース有効化、物理配線、ドライバの電源はこのパッケージでは設定しません。使用機器に適した条件で事前に構成してください。

## 起動・動作確認

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

別ターミナルから確認：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /odom
```

起動される要素：

| 要素 | 機能 |
| --- | --- |
| `robot_state_publisher` | URDF のリンク・ジョイント情報を配信 |
| `controller_manager`（`ros2_control_node`） | ZLAC8015D ハードウェアプラグインと制御ループを実行 |
| `joint_state_broadcaster` | 左右駆動輪の関節状態を配信 |
| `diff_drive_controller` | 車体の速度指令を左右車輪の角速度指令に変換し、オドメトリを計算 |

`launch` では `/diff_drive_controller/cmd_vel_unstamped` → `/cmd_vel`、`/diff_drive_controller/odom` → `/odom` のリマッピングを設定しています。

| 主なトピック | 内容 |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist`：`linear.x` [m/s]、`angular.z` [rad/s] |
| `/joint_states` | `sensor_msgs/msg/JointState`：車輪の位置・角速度・effort |
| `/odom` | `nav_msgs/msg/Odometry`：車輪オドメトリ |
| `/tf` | 有効化されたオドメトリ TF など |

実機を動かす試験では、事前に非常停止手段を準備し、最初は車輪を浮かせて回転方向を確認してください。終了時は起動ターミナルで `Ctrl+C` を押します。

## 現在のパラメータ

### `config/controllers.yaml`

| 設定 | `main` の値 | 説明 |
| --- | --- | --- |
| `controller_manager.update_rate` | `200` | 制御更新レート [Hz] |
| `left_wheel_names` | `left_wheel_joint` | 左車輪ジョイント |
| `right_wheel_names` | `right_wheel_joint` | 右車輪ジョイント |
| `wheel_separation` | `0.243` | 差動二輪コントローラで使用する車輪間距離 [m] |
| `wheel_radius` | `0.064` | 車輪半径 [m] |
| `publish_rate` | `50.0` | オドメトリ公開レート [Hz] |
| `open_loop` | `false` | フィードバックを用いてオドメトリを算出 |
| `position_feedback` | `true` | エンコーダ位置を使用 |
| `enable_odom_tf` | `true` | オドメトリ TF を配信 |
| `odom_frame_id` | `odom` | オドメトリフレーム |
| `base_frame_id` | `base_footprint` | 車体基準フレーム |
| `use_stamped_vel` | `false` | 速度指令にタイムスタンプなしの `Twist` を使用 |

**要確認：** 現在の URDF では車輪ジョイントの Y 座標が `+0.138 m` / `-0.138 m` で、その差は **0.276 m** です。一方、`controllers.yaml` の `wheel_separation` は **0.243 m** です。両者は一致していません。どの値を使うべきかは実機の有効車輪間距離やキャリブレーション結果に基づいて確認してください。ここではコードや設定値を変更していません。

### `urdf/orange_go_2025.urdf`

| 設定 | `main` の値 | 説明 |
| --- | --- | --- |
| `can_interface` | `can0` | SocketCAN インターフェース |
| `node_id` | `1` | CANopen ノード ID |
| `control_mode` | `ladrc_torque` | 現在の制御モード |
| `ladrc_b0_left` / `ladrc_b0_right` | `18.0` / `18.0` | 左右の公称電流入力ゲイン [(rad/s²)/A] |
| `ladrc_wc` | `5.0` | 制御帯域幅 [rad/s] |
| `ladrc_wo` | `20.0` | ESO 帯域幅 [rad/s] |
| `current_limit_a` | `8.0` | LADRC TRACK 時の電流上限 [A] |
| `current_rate_limit_a_per_s` | `20.0` | LADRC TRACK 時の電流変化率上限 [A/s] |
| `feedback_timeout_ms` | `30.0` | LADRC 用速度フィードバック監視閾値 [ms] |

使用するプラグインは `zlac8015d_hardware/Zlac8015dSystemHardware` です。両車輪とも ROS 2 側のコマンドインターフェースは `velocity` [rad/s]、状態インターフェースは `position`、`velocity`、`effort` です。

`control_mode` で以下を切り替えられます。

- `velocity`：ZLAC8015D 内蔵の速度制御に角速度指令を送信。
- `ladrc_torque`：ハードウェア側で車輪角速度指令を LADRC により電流指令へ変換。現在の設定。

LADRC の詳細、CANopen PDO、起動時の静止摩擦対策、減速ランプ、ログは [hardware 側 README](https://github.com/shozawakoki/zlac8015d_hardware#readme) を参照してください。設定変更後は再ビルドし、launch を再起動してください。

## 注意事項・トラブルシューティング

- `can0` がない：`ip -details link show can0` で CAN デバイスの状態を確認。
- コントローラが起動しない：`ros2 control list_controllers`、車輪ジョイント名、YAML と URDF、プラグインのビルド状況を確認。
- `/joint_states` が更新されない：ドライバ電源、CAN 接続、PDO 受信を確認。
- `/odom` の位置がリセットされる：ハードウェア起動時にエンコーダの相対原点が再設定されるため、再起動前後の位置連続性に注意。
- 実機の回転方向が合わない：左モータはハードウェア実装で符号反転されるため、配線と座標系を含めて確認。
- 通信断からの自動再起動用 `can_launch_watchdog.py` は **このリポジトリには含まれません**。別途使用する場合、launch の二重起動を避けてください。
- 通常停止や通信断に対するソフトウェア処理のみで安全停止は保証できません。非常停止回路、モータドライバの保護設定を別途用意してください。

## ライセンス

現時点の `package.xml` のライセンス欄は `TODO` です。公開・再利用に関する条件は、正式なライセンスを設定するまで未確定です。
