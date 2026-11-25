# 指令速度ステップ応答試験：/cmd_vel と実機ログの一括保存

ブランチ：`experiment/cmdvel-response-logger`（`zlac8015d_can_bringup`）。通常起動の `release/mimo-ladrc-stable` は変更しない。
ハードウェア側には `zlac8015d_hardware` の `experiment/mimo-ladrc-no-breakaway`（過速度遮断2.0 m/s、8 A指令上限）を使用する。

## 機能

ユーザーが `--linear` [m/s]、`--angular` [rad/s]、`--duration` [s] を指定し、`/cmd_vel` (`geometry_msgs/msg/Twist`) に指定した速度をデフォルト20 Hzで送信する。
指定秒数が経過したら、`/cmd_vel=0` を送信し、デフォルト3秒間は停止経過の計測を続ける。実験ノード終了時・Ctrl+C・エラー時も0速度を繰り返し指令する。
本スクリプトはハードウェアプラグインの起動、CAN初期化、制御ゲイン変更、電流の直接指令は**行わない**。実機launchは別端末で実行する。

### 自動保存データ

| ファイル | データ |
|---|---|
| `commands.csv` | 送信時刻、並進目標速度、目標角速度 |
| `joint_states.csv` | 左右車輪のエンコーダ由来角位置/角速度、`effort`（提供される場合は実電流[A]）、車輪速度から換算した v, omega |
| `odom.csv` | `/odom` の姿勢(x, y, quaternion)、並進速度、角速度 |
| `hardware_mimo_ladrc_200hz.csv` | ハードウェアが生成したCSVのコピー：左右角速度・位置、左右の**電流指令と実電流**、LESO z1/z2、機体速度・ESO推定、過速度ラッチ等 |
| `metadata.json` | 指令、試験条件、サンプル数、速度追従RMSE、最大角速度偏差、正常終了/中断、診断CSVコピー状態 |

ROS側のCSVは同じプログラム内の単調時計を基準とする `t_s` と受信時 `epoch_s` を併記する。ハードウェア内部の `time_s` は**ros2_control起動基準**で別原点のため、そのまま結合しないこと。
`joint_states.effort` が出ない環境では実電流値は `nan`。`hardware_mimo_ladrc_200hz.csv` の `actual_current_*_A` を確認すること。
なお `/odom` はエンコーダベースのオドメトリであり、地面に対する独立した実測速度ではない。

## 1. ブランチ更新とビルド（launch停止状態で実施）

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch experiment/mimo-ladrc-no-breakaway
git pull --ff-only origin experiment/mimo-ladrc-no-breakaway

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch experiment/cmdvel-response-logger
git pull --ff-only origin experiment/cmdvel-response-logger

python3 -m unittest discover -s tests -p 'test_cmdvel_trial_core.py' -v

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

## 2. モータ起動（端末1）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

起動ログに `MIMO_LADRC_NO_BREAKAWAY_EXP_20261115_R5` が表示され、`ros2 control list_controllers` に `joint_state_broadcaster` と `diff_drive_controller` が `active` と表示されることを確認する。
**走行指令の重複を避けるため、path_follower_nodeなど `/cmd_vel` を送信する別ノードを停止**する。

## 3. 試験（端末2）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 車輪を浮かせた低速初回試験の例：v=0.10 m/s、omega=0、3秒
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.10 --angular 0.0 --duration 3.0 --arm
```

旋回を含む入力例：

```bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.10 --angular 0.20 --duration 4.0 --arm
```

その場旋回の入力例：

```bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.0 --angular 0.20 --duration 3.0 --arm
```

既定の試験上限は **`|v| <= 1.0 m/s`、`|omega| <= 1.0 rad/s`**。前進・後進、左右旋回に適用し、両方の最大値を同時に指定できる。1回の指令時間は最大60秒。

`v=1.0 m/s, omega=1.0 rad/s` の場合、車輪半径0.064 m、トレッド0.354 mから車輪周速度の指令は左0.823 m/s／右1.177 m/sとなり、ソフトウェア側の中断閾値1.5 m/sとハードウェア側の2.0 m/sのどちらも下回る。実測値が1.5 m/s以上になった場合は試験を中断する。

高速度の設定は測定プログラムが**受け付ける最大値**を示すもので、安全な実機走行を保証するものではない。初回は引き続き低速・短時間かつ車輪を浮かせた状態から試験する。
動作中の実測車輪周速度が1.5 m/s以上、`/joint_states`が0.5秒以上途絶、別の `/cmd_vel` 発行ノードが現れる場合は、試験を中断してゼロ速度を送る。
この1.5 m/sは記録プログラム側の保守的な中断値であり、ハードウェア側の独立した2.0 m/s電流0 Aラッチとは異なる。

最大入力の指定例（**動作確認と安全な走行条件の確認後にのみ実施**）：

```bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 1.0 --angular 1.0 --duration 3.0 --arm
```

この例は入力上限での記録可否を示すもの。実機での実行前に路面、転倒リスク、外部非常停止、十分な試験スペースと制動距離を確認する。

## 4. 保存先

```text
~/ros2_ws/mimo_cmdvel_trials/
  20261116TXXXXXXZ_vp0_100_wp0_000_00/
    commands.csv
    joint_states.csv
    odom.csv
    hardware_mimo_ladrc_200hz.csv
    metadata.json
```

ファイル名の日時・速度値は実行時に自動生成される。測定後はこのフォルダをtar.gz圧縮し、CSVとJSONを解析に使用する。

## 安全上の注意

実機の安全確保には独立した物理非常停止が必要。プログラムからの0速度コマンドは停止を保証しない。
制御プログラム側の0 Aラッチは車輪周速度2.0 m/sで作動するが、回転を瞬時に止めるものではない。
試験中は車輪を手で拘束しない。屋外・地上走行では作業区域を隔離し、短時間・低速の試験から開始する。
試験後は車輪が完全に停止したことを確認する。
