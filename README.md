# zlac8015d_can_bringup

> **作業ブランチ `devel/staircase-evaluation`：** 始動・失速復帰の改良版ハードウェア
> [`zlac8015d_hardware/devel/startup-stall-rework`](https://github.com/shozawakoki/zlac8015d_hardware/tree/devel/startup-stall-rework)
> に対応します。短距離の階段状速度指令と `stage_metrics.csv` を追加しました。
> 詳細は [STAIRCASE_EXPERIMENTS.md](STAIRCASE_EXPERIMENTS.md) を参照してください。
> 最初は車輪を浮かせた状態で単一条件のみ検証してください。

ZLAC8015D モータドライバを搭載した対向二輪型移動ロボット向けの **ROS 2 bringup パッケージ**です。ROS 2 パッケージ名は `zlac_can_bringup` です。

`robot_state_publisher`、`ros2_control_node`、`joint_state_broadcaster`、`diff_drive_controller` を起動し、別リポジトリの [zlac8015d_hardware](https://github.com/shozawakoki/zlac8015d_hardware) をハードウェアプラグインとして読み込みます。

> 本 README は `main` ブランチの実ファイルに基づいています。起動対象は `launch/zlac_can_bringup.launch.py`、設定は `config/controllers.yaml` と `urdf/orange_go_2025.urdf` です。CAN 速度やモータ配線などの機体固有条件は、実機で確認してください。

## Python 記録ノードが起動しない場合

`executable 'record_experiment.py' not found on the libexec directory` の場合、ROS 2 用の
`lib/zlac_can_bringup/` に Python 実行ファイルが存在しない、または
`colcon build --symlink-install` 下のシンボリックリンク先に実行権限がない可能性があります。

この修正ブランチでは CMake のビルドディレクトリに実行権限付きのスクリプトを複製してから
`install(PROGRAMS)` を行います。変更後は次を実行します。

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch fix/python-executable-symlink
cd ~/ros2_ws
colcon build --packages-select zlac_can_bringup --symlink-install --cmake-force-configure
source install/setup.bash
ls -lL ~/ros2_ws/install/zlac_can_bringup/lib/zlac_can_bringup/record_experiment.py
```

上記 `ls` でファイルが存在し、実行権限（`x`）が付いていることを確認してください。
作業ブランチを切り替えても、現在の `main` には変更が入りません。

## ROS 2 controller active 状態の誤判定修正

`ros2 control list_controllers` では `active` が表示されていても、
ターミナルのANSI色付けを含む場合、従来の文字列比較で
`active=[]` になる可能性がありました。

このブランチはCLI出力の制御文字を除去して状態を解析します。
また、準備判定の後に取得する新しい車輪フィードバックの確認と、
中断した本来の理由を保存する従来修正も含みます。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch fix/sweep-active-state-parser
cd ~/ros2_ws
colcon build --packages-select zlac_can_bringup --symlink-install --cmake-force-configure
source install/setup.bash

# Offline parser test (commands no motors)
python3 ~/ros2_ws/src/zlac8015d_can_bringup/test/test_active_state_parser.py

# Dry run
ros2 run zlac_can_bringup run_gain_sweep.py --config ~/single_trial.yaml
```

## 一括試験で active_samples=0 となる場合（診断修正）

起動後の `ros2 control list_controllers` 呼び出しが同期処理で数秒待機する間、
スイープ実行ノードは `/joint_states` を処理できませんでした。
その直後に `last_feedback < 0.35秒` を判定していると、
実際にフィードバックが受信されていても起動確認を失敗する可能性があります。

このブランチではCLIの戻り後に新しい `/joint_states` を処理してから
準備完了を判定します。また、起動段階の中断理由を
`No usable active speed-response measurements` で上書きしないよう修正しました。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch fix/sweep-readiness-diagnostics

cd ~/ros2_ws
colcon build --packages-select zlac_can_bringup --symlink-install --cmake-force-configure
source install/setup.bash

# まず安全なドライラン
ros2 run zlac_can_bringup run_gain_sweep.py --config ~/single_trial.yaml
```

既存の `sweep_...` にある `001_.../launch.log` と `trial.json` は、
失敗原因の切り分けに利用できます。新たな実機動作の前に確認してください。
新ブランチは既存の制御器・ゲイン設定・モータの起動方式を変更しません。

## 一括試験スクリプトのモジュール読み込み修正

`ros2 run zlac_can_bringup run_gain_sweep.py` の実行時に
`ModuleNotFoundError: No module named 'sweep_core'` が出る問題に対応しました。
`--symlink-install` の場合、`run_gain_sweep.py` がビルドディレクトリ側の
実体を参照することがあります。補助モジュール `sweep_core.py` を
スクリプト実体と同じ `build/.../executable_scripts/` に配置し、
ROS 2 の `lib/zlac_can_bringup/` にもインストールするように修正しています。

```bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch fix/sweep-core-import

cd ~/ros2_ws
colcon build --packages-select zlac_can_bringup \
  --symlink-install --cmake-force-configure
source install/setup.bash

ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/gain_sweep_example.yaml
```

最後の実行は**ドライラン**です。モータへの速度指令を送信しません。

## ゲイン候補を順番に自動試験（devel/auto-gain-sweep）

PID、LADRC、内蔵速度制御の複数候補をYAMLで定義し、**ゲイン適用 → 速度指令 →
車輪応答記録 → 停止 → RMSE/電流RMS算出 → 次候補**まで自動実行します。
`zlac8015d_hardware` は `devel/pid` を使用してください。
`main` への変更はありません。

**初期動作はドライラン（車輪を動かさない）**です。

```bash
ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/gain_sweep_example.yaml
```

安全停止手段と試験空間を確認した後、`--execute --i-have-e-stop` を付け、
対話確認を行うと速度指令を自動送信します。**運転中の監視者と物理非常停止が必須**です。
使い方、既定の10候補、制限値、出力データについては
[一括ゲイン試験の説明](AUTO_GAIN_SWEEP.md) を参照してください。

## PID / LADRC 実機実験と自動記録（devel/experiment-logging）

このブランチでは、制御方式・各ゲイン・試験条件を launch 引数で指定し、
`/cmd_vel`、`/joint_states`、`/odom` を実験ごとに CSV へ記録する
`zlac_experiment.launch.py` を追加しています。使用には
[hardware の `devel/pid`](https://github.com/shozawakoki/zlac8015d_hardware/tree/devel/pid)
が必要です。**速度指令は自動生成せず、記録専用**です。

```bash
ros2 launch zlac_can_bringup zlac_experiment.launch.py \
  mode:=pid_torque pid_kp:=0.7 pid_ki:=0.5 pid_kd:=0.03 \
  test_label:=straight trial:=01
```

ゲイン、反復番号、予定速度、試験時間、路面・外乱条件などの指定方法、
ビルド、ログの保存形式は [EXPERIMENTS.md](EXPERIMENTS.md) を参照してください。
既存の `zlac_can_bringup.launch.py` は変更していません。

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
| `wheel_separation` | `0.354` | 実機で確認した有効車輪間距離 [m] |
| `wheel_radius` | `0.064` | 車輪半径 [m] |
| `publish_rate` | `50.0` | オドメトリ公開レート [Hz] |
| `open_loop` | `false` | フィードバックを用いてオドメトリを算出 |
| `position_feedback` | `true` | エンコーダ位置を使用 |
| `enable_odom_tf` | `true` | オドメトリ TF を配信 |
| `odom_frame_id` | `odom` | オドメトリフレーム |
| `base_frame_id` | `base_footprint` | 車体基準フレーム |
| `use_stamped_vel` | `false` | 速度指令にタイムスタンプなしの `Twist` を使用 |

**車輪間距離の設定：** `main` では、実機で確認した有効車輪間距離 **0.354 m** が `wheel_separation` に設定されています。URDF の左右車輪ジョイントの Y 座標も `+0.177 m` / `-0.177 m`（間隔 **0.354 m**）となっており、両設定の数値は一致しています。なお、URDF のジョイント位置はモデルの幾何情報であり、有効車輪間距離が実際の車軸の物理寸法と異なる場合には、表示・運動学の目的に応じて区別して管理してください。

### `urdf/orange_go_2025.urdf`

| 設定 | `main` の値 | 説明 |
| --- | --- | --- |
| `can_interface` | `can0` | SocketCAN インターフェース |
| `node_id` | `1` | CANopen ノード ID |
| `left_wheel_joint` / `right_wheel_joint` の Y 位置 | `+0.177` / `-0.177` | URDF 上の車輪ジョイント位置 [m] |
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
- `/odom` の再初期化：ハードウェア起動時に車輪エンコーダの相対原点が再設定されます。本機では GPS・IMU 情報も利用する外部の位置推定系で扱う前提です。外部推定系はこのリポジトリには含まれず、車輪オドメトリ単体の連続性は保証されません。
- 左右車輪の回転方向：現行実機では調整済みです。ハードウェア実装では左モータの符号を反転しており、別機体へ適用する場合には再確認してください。
- 通信断からの自動再起動用 `can_launch_watchdog.py` は **このリポジトリには含まれません**。別途使用する場合、launch の二重起動を避けてください。
- 通常停止や通信断に対するソフトウェア処理のみで安全停止は保証できません。非常停止回路、モータドライバの保護設定を別途用意してください。

## ライセンス

現時点の `package.xml` のライセンス欄は `TODO` です。公開・再利用に関する条件は、正式なライセンスを設定するまで未確定です。
