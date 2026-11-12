# PID / LADRC ゲイン候補の一括実機試験

**対象ブランチ:** `zlac8015d_can_bringup/devel/auto-gain-sweep`

`gain_sweep_example.yaml` に指定した複数の PID・LADRC 候補を、
**1つのコマンドで順番に実行・速度応答記録・結果集計**するための実験用プログラムです。
ドライバ内蔵速度制御 `velocity` の基準試験も実行できます。
既存の `main` と `zlac_experiment.launch.py` は変更していません。

**注意：** `--execute` を付けると実機に速度指令を送信します。実行中は操作者が
実機のそばで監視し、独立した非常停止装置を使用できる状態にしてください。
このコードのゼロ速度指令や CAN エラー検出は、ハードウェア非常停止を代替しません。
初回は車輪を浮かせた状態で1候補の低速試験から実施してください。
独立したCAN通信断時の停止手段が未確認なら、床上での自動試験は実行しないでください。

## 処理の流れ

1. YAMLの候補、ゲイン、速度・時間、車輪速度・電流上限、指令移動距離を検証
2. 試験ごとの専用ディレクトリを作成
3. 選択した制御方式・ゲインで `zlac_experiment.launch.py` を新規起動
4. `diff_drive_controller` と `joint_state_broadcaster` の active を確認
5. 車輪速度フィードバックが新鮮で停止状態であることを確認
6. プロファイルに従い20 Hz（変更可能）で `/cmd_vel` を送信
7. 全時間の `/cmd_vel`・`/joint_states`・`/odom` をCSVに記録
8. ゼロ指令を送信し、車輪速度の収束を確認
9. launch を停止し、PID/LADRC内部電流ログもコピー
10. 各候補の速度RMSE・電流RMS・最大電流を算出して `summary.csv` に追記
11. 次の候補を同様に実行。異常時は**残りの候補をすべて中止**

**各候補の間ではハードウェアを再起動します。**
同一プロセスで制御方式やゲインをホットスワップしないため、
候補ごとに確実に新しい設定が適用されます。

## セットアップ

ハードウェア側はPID機能のある `devel/pid` を使用してください。

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch devel/pid
git pull --ff-only

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch devel/auto-gain-sweep
git pull --ff-only

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

## 1. 最初に試験内容を確認（モータは動かない）

```bash
ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/gain_sweep_example.yaml
```

**既定動作はドライラン**です。YAMLを検証して実施予定の全候補と設定値を表示し、
ROSトピックへの書き込みやモータへの指令は行いません。

初期YAMLには、基準となる内蔵速度制御1条件、PID 6条件、
LADRC 3条件（合計10条件）が登録されています。1回あたり9秒、
`0 → 0.10 m/s → 0` の直進指令です。

## 2. 実行前にYAMLを必要な候補だけに絞る

編集対象：

`config/gain_sweep_example.yaml`

```yaml
schema_version: 1
repeats: 1
publish_hz: 20

safety:
  max_linear_m_s: 0.20
  max_angular_rad_s: 0.40
  max_wheel_rad_s: 10.0
  max_current_a: 4.0

profiles:
  - name: straight_010
    stages:
      - {duration_s: 2.0, linear_x: 0.00, angular_z: 0.00}
      - {duration_s: 4.0, linear_x: 0.10, angular_z: 0.00}
      - {duration_s: 3.0, linear_x: 0.00, angular_z: 0.00}

candidates:
  - name: pid_p02
    mode: pid_torque
    gains: {pid_kp: 0.2, pid_ki: 0.0, pid_kd: 0.0}
  - name: pid_pi_01
    mode: pid_torque
    gains: {pid_kp: 0.7, pid_ki: 0.1, pid_kd: 0.0}
  - name: adrc_wc3_wo12
    mode: ladrc_torque
    gains: {ladrc_wc: 3.0, ladrc_wo: 12.0}
```

上記は設定項目の説明用の抜粋です。実際はリポジトリに含まれる
`gain_sweep_example.yaml` を元に編集してください。
`defaults` に変更しないパラメータ値を置き、
各 `candidates[].gains` で候補ごとの変更分だけ指定します。

編集後は必ずドライランをもう一度実行してください。

### 固定されたソフトウェア制約

- 許容する設定上の最大並進速度：0.20 m/s
- 許容する設定上の最大角速度：0.40 rad/s
- 許容する設定上の最大車輪角速度：10 rad/s
- 許容する設定上の最大電流：8 A（付属例は4 A）
- 1プロファイルの指令並進移動量の絶対積分：0.50 m以下
- 1プロファイルの指令旋回角度の絶対積分：0.80 rad以下
- 試験時間：最大45秒、候補×繰り返し×プロファイル：最大120回
- 開始・終了速度はともにゼロ必須
- 制御周期は20 Hzを基本とし、10～50 Hzのみ設定可

**これらはソフトウェア検証制約であり、モータの安全を保証する許容値ではありません。**
車輪を浮かせた状態で安全確認した後、現場の実際の速度・電流・空間条件に合わせて
更に低い制限へ変更してください。

## 3. 実機で一括実行（操作者の確認が必要）

他の bringup や `/cmd_vel` 発行ノードを停止し、物理非常停止を準備してください。
以下のコマンドで試験内容を再表示し、確認文を入力した場合のみ一括実行します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/gain_sweep_example.yaml \
  --execute --i-have-e-stop
```

起動後に **`I HAVE AN EMERGENCY STOP`** と正確に入力すると、
自動試験が開始されます。

手動確認を別途完了させたうえで、無人ではなく操作者がその場にいる
非対話シェル等から実行する場合は
`--execute --i-have-e-stop --yes` の指定も可能ですが、
通常は対話確認を推奨します。

**測定の途中で異常が発生した場合は、ゼロ速度指令を送信して launch を停止し、
残りの全試験を中止します。** キーボードから `Ctrl+C` でも中断できます。
万一モータが停止しない場合は、ゼロ指令の送信を待たずに物理非常停止を操作してください。

## 4. 出力ファイル

```text
~/ros2_experiments/
  sweep_YYYYMMDDTHHMMSSZ_XXXXXXXX/
    plan.json
    summary.csv
    001_pid_p02_straight_010_rep01/
      trial.json
      launch.log
      hardware_current_trace.csv    # PID/LADRC の場合
      raw/
        <日時_mode_profile_trial_識別子>/
          experiment.json
          cmd_vel.csv
          joint_states.csv
          odom.csv
    002_pid_pi_01_straight_010_rep01/
      ...
```

- `plan.json`: 一括試験全体の設定
- `trial.json`: 候補名、制御方式、ゲイン、ステージごとの指令、実行状態、集計値
- `launch.log`: 起動・制御に関するROS2ログ
- `cmd_vel.csv`: 受信された速度指令（Twist）
- `joint_states.csv`: 左右車輪の実速度・電流・位置
- `odom.csv`: 差動二輪のオドメトリ
- `hardware_current_trace.csv`: PID/LADRCの指令電流など、ハードウェア内部ログ
- `summary.csv`: 全候補をまとめた評価表

`summary.csv` には次の指標が保存されます。

| 指標 | 内容 |
| --- | --- |
| `left_rmse_rad_s` | 左車輪速度のRMSE [rad/s] |
| `right_rmse_rad_s` | 右車輪速度のRMSE [rad/s] |
| `combined_rmse_rad_s` | 左右の速度誤差を合わせたRMSE [rad/s] |
| `left_current_rms_a` / `right_current_rms_a` | 左右の電流RMS [A] |
| `left_peak_current_a` / `right_peak_current_a` | 左右の実測最大電流 [A] |
| `status`, `reason` | 完了・中断の状態と理由 |

RMSEは指令値が非ゼロの区間のみを対象とし、`/cmd_vel` と `/joint_states` の
受信時刻によって紐づけます。各車輪の目標角速度は
`wheel_radius_m=0.064`、`wheel_separation_m=0.354` を使用して計算します。
速度指令変化の直後も含むため、TRACKのみの純粋な制御則の評価とは区別してください。

### 比較実験上の注意

現在の PID/LADRC プログラムは静止摩擦補償、TRACK_HOLD、直進同期補正、
減速フィードフォワードを共有します。
`RMSE` には始動直後の過渡応答と共通の補助制御も含まれます。
ゲイン探索の一次選定には使用できますが、卒業研究の最終比較では
TRACK区間、始動区間、停止区間を区別し、条件を複数回反復することを推奨します。

**実行検証:** コードはROS 2 Humble実機・ビルド環境ではまだ動作確認していません。
最初はYAML候補を1つに減らして、車輪を浮かせた状態でドライランと
単一候補の実行・停止動作を検証してください。
