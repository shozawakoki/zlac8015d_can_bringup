# /cmd_vel 応答試験：PID / ADRC 制御状態付きデータ収集

対象：`zlac_can_bringup` の `main`（スクリプト `scripts/run_cmdvel_trial.py`）。モータ制御器は別端末の `ros2_control_node` で稼働していることを前提とする。

## 1. 機能・保存形式

従来どおり `--linear`, `--angular`, `--duration`, `--arm` を指定し、/cmd_vel を20 Hzで送信する。終了時に /cmd_vel=0 を送信し、既定で3秒の停止応答を記録する。別ノードの /cmd_vel 競合、/joint_states タイムアウト、車輪周速度1.5 m/s以上で試験を中断する。制御器の起動やモード変更はしない。

**追加機能:** 起動済みの `ros2_control_node` プロセスの `ZLAC_CONTROL_MODE` 環境変数（記録プログラム自身の環境変数ではない）を読み取る。見つからない場合はインストール済みURDFの制御モード既定値を参照するが、根拠・不確実性をJSONに残す。**実行されたプラグインのコンパイル済み制御モードを直接取得する機構ではない**ため、起動ログのマーカーによる照合も行うこと。複数の`ros2_control_node`を検出した場合は安全上試験を拒否する。

| 保存ファイル | 主な内容 |
| --- | --- |
| `commands.csv` | 指令時刻、`v_ref`, `omega_ref` |
| `joint_states.csv` | 車輪角速度・位置、実電流、車輪由来の機体速度 |
| `odom.csv` | オドメトリXY位置と姿勢、速度 |
| `metadata.json` | モード判定結果・判定根拠、コントローラPID/再起動判定、URDFパラメータ、ゲイン・制限、サンプル数、RMSE、停止/異常状態 |
| `hardware_controller_200hz.csv` | 制御器が出力した**累積**CSVのコピー（入手可能時） |
| `hardware_trial_200hz.csv` | 試験開始前の記録済みファイル末尾以後に追加された**おおよその試験区間**のCSV（入手可能時） |

内部CSVは選択した制御モードの実データをそのままコピーする。PIDの積分状態・微分フィルタ値、ADRCのESO推定速度・推定外乱、左右指令電流・実電流、安全ラッチ等は、それぞれ対応した列を持つ。**PIDの`hat_d_*`列はESO推定値ではない。** ハードウェアログがないと積分状態・ESOの実記録は取得できない。

ログファイルは制御器の起動時から追記される場合があり、試験専用抽出の先頭境界はC++の出力バッファ処理により厳密ではない。C++の `time_s` とROSレコーダの `t_s` は同じ時間原点を共有しない。単純結合せず、必要なら信号変化の時刻で別途同期する。

### 対応制御モードと自動ログ選択

| `ZLAC_CONTROL_MODE` | 実測電流/内部状態の保存元 |
| --- | --- |
| `mimo_pid_no_breakaway` | `/tmp/mimo_pid_no_breakaway_200hz.csv` |
| `mimo_ladrc_no_breakaway_actual_current` | `/tmp/mimo_ladrc_no_breakaway_actual_current_200hz.csv` |
| `mimo_ladrc_no_breakaway` | `/tmp/mimo_ladrc_no_breakaway_200hz.csv` |
| `mimo_pid_torque` | `/tmp/mimo_pid_track_200hz.csv` |
| `mimo_ladrc_torque` | `/tmp/mimo_ladrc_track_200hz.csv` |
| `pid_torque` / `ladrc_torque` | `/tmp/pid_track_200hz.csv` / `/tmp/ladrc_track_200hz.csv` |
| `velocity` | 200 Hz外部電流制御CSVなし。ROSデータのみ。 |

ファイルが存在しない、今回の試験で更新されていない、ヘッダーが期待と異なる、コントローラプロセスが再起動した場合は不正なCSVをコピーせず`hardware_csv.reason`に理由を記録する。

## 2. 更新・ビルド

試験ノード、モータ制御を停止した状態で変更する。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git switch main
git pull --ff-only origin main

python3 -m unittest discover -s tests -p 'test_cmdvel_trial*.py' -v

cd ~/ros2_ws
colcon build --packages-select zlac_can_bringup --symlink-install
source install/setup.bash
```

PID/LADRCハードウェア側は対応する実験ブランチ `experiment/mimo-ladrc-no-breakaway-actual-current-eso` のビルドを利用する。READMEの記載だけではプラグイン更新は完了しないので、ハードウェアも別途ビルドしておく。

## 3. 起動（端末1、必要な安全確認後のみ）

PID実験：

```bash
ZLAC_CONTROL_MODE=mimo_pid_no_breakaway \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

ADRC実験（別起動）：

```bash
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway_actual_current \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

**同時に起動しない。** PIDなら `MIMO_PID_NO_BREAKAWAY_ZERO_REFERENCE_20261126_R1`、ADRCなら `MIMO_LADRC_ZERO_REFERENCE_CLOSED_LOOP_20261125_R1` の起動ログと実際の稼働状況を確認する。

## 4. 試験（端末2、元のコマンドのまま）

```bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 1.0 --angular 0.0 --duration 5.0 --arm
```

出力先：`~/ros2_ws/mimo_cmdvel_trials/YYYYMMDDTHHMMSSZ_v..._w..._00/`。各試験で必ず一意のフォルダーが作られる。

### 任意の検証オプション

```bash
# プロセス環境と異なれば、走行前に拒否
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.1 --angular 0.0 --duration 3.0 \
  --control-mode mimo_pid_no_breakaway --arm

# 既定の自動選択が使えない場合のみパスを手動指定
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.1 --angular 0.0 --duration 3.0 \
  --hardware-log /tmp/mimo_pid_no_breakaway_200hz.csv --arm
```

`--control-mode` はオペレーターの申告であり、プラグイン実行状態の直接証明ではない。モードとログの取り違えを避けるには起動識別ログも保存すること。

## 5. 注意点

- 空転・車輪浮上試験のエンコーダ速度は地上に対する移動速度の真値ではない。
- ADRCとPIDでゲイン調整・物理条件・指令系列を記録し、同一条件・評価窓を用いて比較すること。
- 実機の停止を保証するのはROSの0速度指令ではなく、別系統の適切な非常停止装置である。
- 初回は車輪浮上・低速から検証し、指令競合・車輪拘束・転倒リスクを避けること。
