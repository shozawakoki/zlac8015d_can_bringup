# /cmd_vel 応答試験：制御モード・内部状態の自動保存

`run_cmdvel_trial.py` は**従来の起動コマンドのまま**、現在のコントローラ設定と対応するハードウェアログを保存します（制御器の起動・切り替えは行いません）。

```bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 1.0 --angular 0.0 --duration 5.0 --arm
```

- 稼働中の `ros2_control_node` の `ZLAC_CONTROL_MODE` 環境変数（別プロセス）から制御モードを判定。未設定ならインストール済みURDFの `control_mode` 既定値を参照し、**推定・未確認の状態も明記**します。これはプラグイン実行モードの直接証明ではありません。起動識別子も目視確認してください。
- PID・ADRCのモードに応じた `/tmp/..._200hz.csv` を自動選択し、積分状態・微分フィルタ・ESO状態・指令/実電流・過速度ラッチなどを、実際に記録された列のまま保存します（モードにより列の意味は異なります）。
- `metadata.json` はモードの判定根拠、制御プロセスPID/起動識別、インストール済みURDFのパラメータとハッシュ、前後でのプロセス再起動判定、収集できたCSVの状態を保存します。旧来の固定 `mode_expected=mimo_ladrc_no_breakaway` は廃止しました。
- `hardware_controller_200hz.csv` はコントローラの累積ログのコピー、`hardware_trial_200hz.csv` は試験前のファイルサイズより後に追記されたCSV行の抽出です。ハードウェアCSVの `time_s` は独立原点、出力バッファの影響もあり**ROSの `t_s` との厳密な時刻同期ではありません**。
- PID版 `mimo_pid_no_breakaway` と実電流ESO版 `mimo_ladrc_no_breakaway_actual_current` の両方に対応します。CSV不存在・未更新・制御プロセス再起動などでは、誤データをコピーせず理由を記録します。

保存先：`~/ros2_ws/mimo_cmdvel_trials/<日時・速度値>/`（既存の `commands.csv`、`joint_states.csv`、`odom.csv` も継続）。

運用詳細、ゲインの記録、トラブルシューティングは [CMDVEL_TRIAL.md](CMDVEL_TRIAL.md) 参照。

---

# 検証用：/cmd_vel 定時間応答試験・ログ収集

このブランチ `experiment/cmdvel-response-logger` は、安定版bringupから分岐した検証用ブランチです。
従来の `launch/zlac_can_bringup.launch.py` は変更していません。
試験プログラム `scripts/run_cmdvel_trial.py` は `/cmd_vel` に指定した並進速度・旋回角速度を指定秒数だけ送信し、左右エンコーダ、車輪換算機体速度、`/odom`、実電流とハードウェアの電流指令・ESO CSVを保存します。
速度指令の標準許容範囲は **並進速度±1.0 m/s・角速度±1.0 rad/s**（同時指定可能）。実測車輪周速度1.5 m/sでの試験中断と、ハードウェア側2.0 m/sの0 Aラッチは維持しています。

モータが別端末で起動している場合の使用例（安全確認後）：

```bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.10 --angular 0.00 --duration 3.0 --arm
```

ビルド、実験前チェック、ファイル構成は [CMDVEL_TRIAL.md](CMDVEL_TRIAL.md) を参照してください。

---
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
