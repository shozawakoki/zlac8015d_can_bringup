# PID / LADRC / 内蔵速度制御：実機速度応答記録

このドキュメントは、`devel/experiment-logging` ブランチの実験用 launch と
**受信専用** CSV 記録ノードの使い方です。既存の `zlac_can_bringup.launch.py`、
元の URDF・YAML、ハードウェアの `main` ブランチには変更を加えません。

## 実験の構成

- 実験用 launch: `launch/zlac_experiment.launch.py`
- ログノード: `scripts/record_experiment.py`
- 利用するハードウェア: `zlac8015d_hardware` の `devel/pid` ブランチ
- モード: `velocity` / `ladrc_torque` / `pid_torque`
- 制御ゲイン: launch 引数から指定し、実行時の `robot_description` のみに反映
- 取得する ROS 2 トピック: `/cmd_vel`、`/joint_states`、`/odom`
- 実験メタデータ: `experiment.json`

このプログラムは **モータを動かす速度指令を生成しません**。
指令は、実機で使用中の既存の経路追従ノードやテレオペレーションなど、
別の安全管理されたシステムから与えてください。設定した
`planned_linear_m_s`、`planned_angular_rad_s`、`planned_duration_s`
は **記録用の予定条件** であり、自動的な走行指令・試験終了を意味しません。
実際に入力された指令速度は `cmd_vel.csv` に保存されます。

## 1. インストール

すでに双方のリポジトリが `~/ros2_ws/src` に存在している場合：

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch devel/pid
git pull --ff-only

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch devel/experiment-logging
git pull --ff-only

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

Ubuntu 22.04 / ROS 2 Humble を想定しています。`rosdep` を使う前に必要な
ROS 2 / rosdep のセットアップを済ませてください。
既存のワークスペースの `main` 等を直接書き換えることはありません。

## 2. 起動例

安全停止手段とロボットの設置を確認してから、次の **いずれか1つ** を起動します。
`Ctrl+C` で launch と記録ノードを停止します。

### PID

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zlac_can_bringup zlac_experiment.launch.py \
  mode:=pid_torque pid_kp:=0.7 pid_ki:=0.5 pid_kd:=0.03 \
  pid_derivative_filter_tau_s:=0.05 \
  test_label:=straight planned_linear_m_s:=0.2 \
  planned_angular_rad_s:=0.0 planned_duration_s:=10.0 \
  trial:=01 surface:=flat disturbance:=none
```

### LADRC

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zlac_can_bringup zlac_experiment.launch.py \
  mode:=ladrc_torque ladrc_b0_left:=18 ladrc_b0_right:=18 \
  ladrc_wc:=5 ladrc_wo:=20 \
  test_label:=straight planned_linear_m_s:=0.2 \
  planned_angular_rad_s:=0.0 planned_duration_s:=10.0 \
  trial:=01 surface:=flat disturbance:=none
```

### ZLAC8015D 内蔵速度制御

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zlac_can_bringup zlac_experiment.launch.py \
  mode:=velocity \
  test_label:=straight planned_linear_m_s:=0.2 \
  planned_angular_rad_s:=0.0 planned_duration_s:=10.0 \
  trial:=01 surface:=flat disturbance:=none
```

共通のパラメータ例：

| Launch 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `mode` | `ladrc_torque` | 使用する制御方式 |
| `pid_kp`, `pid_ki`, `pid_kd` | `0.7`, `0.5`, `0.03` | PID ゲイン |
| `pid_derivative_filter_tau_s` | `0.05` | PID 微分フィルタ時定数 [s] |
| `ladrc_b0_left`, `ladrc_b0_right` | `18.0`, `18.0` | 左右の公称入力ゲイン |
| `ladrc_wc`, `ladrc_wo` | `5.0`, `20.0` | 制御・ESO 帯域幅 [rad/s] |
| `current_limit_a` | `8.0` | 電流指令の上限 [A] |
| `current_rate_limit_a_per_s` | `20.0` | TRACK 電流変化率の上限 [A/s] |
| `feedback_timeout_ms` | `30.0` | 電流制御モードの速度フィードバック監視 [ms] |
| `planned_linear_m_s` | `0.2` | 予定並進速度 [m/s]（記録のみ） |
| `planned_angular_rad_s` | `0.0` | 予定旋回速度 [rad/s]（記録のみ） |
| `planned_duration_s` | `10.0` | 予定測定時間 [s]（記録のみ） |
| `test_label`, `trial` | `speed_tracking`, `01` | 条件名と反復試験番号 |
| `surface`, `disturbance`, `notes` | `unspecified`, `none`, 空 | 路面・外乱・備考 |
| `output_root` | `~/ros2_experiments` | 記録先ディレクトリ |

launch が読み込む URDF とコントローラ設定は、インストール済み bringup の
`orange_go_2025.urdf` と `controllers.yaml` です。元ファイルを書き換えず、
起動時だけハードウェア設定を置換します。環境変数 `ZLAC_CONTROL_MODE` が
シェル内で設定されていても、実験用 launch が `mode` と整合させます。
ゲイン変更後の再ビルドは不要です。

**同じパラメータを明示的に指定してから、毎回別の launch として起動してください。**
`mode` を走行中に変更する機能は提供しません。

## 3. 記録されるデータ

```text
~/ros2_experiments/
  20261112T014300Z_pid_torque_straight_trial01_a1b2c3d4/
    experiment.json
    cmd_vel.csv
    joint_states.csv
    odom.csv
```

ディレクトリ名は UTC 時刻とランダムなサフィックスを含むため、
同じ条件の実験を繰り返しても以前の結果を上書きしません。

- `experiment.json` : 制御方式、PID/LADRCゲイン、電流制限、試験名、予定条件、
  車輪半径、車輪間距離、`cmd_vel` タイムアウト、開始・終了時刻、受信メッセージ数
- `cmd_vel.csv` : 受信した並進速度・旋回速度の指令
- `joint_states.csv` : 左右輪の位置・実測角速度・`effort`
- `odom.csv` : オドメトリの位置・ヨーの正弦/余弦・並進速度・角速度

各 CSV の `time_s` は記録ノード起動からの **単調増加時計に基づく受信時刻**で、
ROS ヘッダーを持つメッセージでは `header_stamp_s` も別に保存します。
3つのトピックは独立して記録されます。分析時はタイムスタンプで同期させてください。

`joint_states` の `effort` は現行 ZLAC ハードウェア実装では実測電流 [A] であり、
トルク [N·m] ではありません。`/odom` は bringup 側の車輪オドメトリであり、
外部 GPS・IMU 融合結果とは区別してください。

**LADRC と PID の追加内部ログ**（指令電流、推定値、TRACK状態等）は、
hardware プラグインが従来どおり `/tmp/ladrc_track_200hz.csv` /
`/tmp/pid_track_200hz.csv` に出力します。
**これら /tmp の内部ログは、この記録ノードでは自動コピーしません。**
必要な場合は次回起動で上書きされる前に別途保存してください。
内蔵速度制御ではこれらの内部ログは生成されません。

## 4. 試験手順と注意

1. 最初は車輪を浮かせ、非常停止手段・通信異常時の停止機能を確認します。
2. 上記 launch を1つだけ起動し、`ros2 control list_controllers` で動作状態を確認します。
3. 別の既存制御ノードから、予定条件に合わせて安全に `/cmd_vel` を入力します。
4. 試験終了後、モータを停止したことを確認し、launch を `Ctrl+C` で終了します。
5. 出力した `experiment.json` の `message_counts` を確認し、記録漏れがないかを確認します。
6. ゲイン・条件を変えて繰り返します。ファイル名には必ず別の識別子が付きます。

`planned_duration_s` が経過してもモータや記録ノードは自動停止しません。
ユーザー側の走行制御が引き続き安全に速度指令を管理する必要があります。

PID・LADRC は起動時の BREAKAWAY、TRACK_HOLD、停止時の減速ランプ、
直進同期処理などの補助制御を共有します。研究比較では START/TRACK/STOP
区間を区別し、各方式の条件を記録してください。

記録機構自体にはモータ制御の安全保証機能はなく、非常停止回路や
ドライバ側の安全停止機能の代わりにはなりません。
