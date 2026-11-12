# 段階速度指令・始動制御の再評価

このブランチは `fix/sweep-active-state-parser` から分岐し、従来の
速度応答ロギング・コントローラ起動判定を引き継いだものです。
対応するハードウェアブランチは
[zlac8015d_hardware/devel/startup-stall-rework](https://github.com/shozawakoki/zlac8015d_hardware/tree/devel/startup-stall-rework)
です。**双方を同時に切り替えてください。**

今回の目的は以下の3段階に分かれます。

1. **空転状態での動作検証：** BREAKAWAYとTRACKの切替、急加速、停止を確認
2. **接地・実荷重下での同定とゲイン調整：** b0・摩擦・左右差を再評価
3. **段階速度を含む性能比較：** 3方式で同一入力・負荷・試験回数を統一

速度指令の自動送信は物理的な走行を伴います。
実機の独立非常停止・CAN断線時の停止機構・試験空間の確認が必要です。
ソフトウェアによるゼロ指令は非常停止の代用になりません。

## インストール

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch devel/startup-stall-rework
git pull --ff-only

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch devel/staircase-evaluation
git pull --ff-only

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup \
  --symlink-install --cmake-force-configure
source install/setup.bash
```

## 変更したパラメータ

`zlac_experiment.launch.py` の引数、`gain_sweep_example.yaml` の
`defaults` または各候補 `gains`、`experiment.json` に以下を追加しました。

| 名前 | 初期値 | 説明 |
| --- | ---: | --- |
| `breakaway_start_a` | 0.65 | 始動電流 [A] |
| `breakaway_step_a` | 0.10 | 始動時の電流増分 [A] |
| `breakaway_max_a` | 2.0 | 始動上限 [A] |
| `breakaway_step_samples` | 20 | 電流増加の周期 [制御サンプル] |
| `breakaway_start_speed_rad_s` | 0.30 | TRACK移行速度の基準 |
| `breakaway_start_samples` | 6 | TRACK移行連続確認サンプル数 |
| `stall_speed_rad_s` | 0.10 | 失速速度閾値 |
| `stall_samples` | 80 | 失速継続サンプル数 |
| `stall_min_drive_current_a` | 0.20 | 再始動を許す最小正方向電流 [A] |

これらは**初期の試験候補**であり、実機での性能改善を保証しません。
車輪の接地状態での始動電流・b0は別途測定・調整してください。

## 階段状速度プロファイル（短距離の初期例）

`config/staircase_sweep_example.yaml` を追加しました。これは以下の速度指令です。

| 速度 [m/s] | 保持時間 [s] |
| ---: | ---: |
| 0.00 | 2.0 |
| 0.05 | 1.0 |
| 0.10 | 1.0 |
| 0.15 | 1.0 |
| 0.10 | 1.0 |
| 0.05 | 1.0 |
| 0.00 | 2.0 |

総試験時間9秒、非ゼロ指令距離の絶対積分は0.45 mです。現行の0.50 m上限を
引き上げずに試験可能な設計ですが、実測移動距離は指令より長くなる場合があります。
**各速度1秒保持では、定常偏差が十分収束しない可能性があります。**
まずは過渡応答を確認する用途とし、定常性能は別の安全な長時間試験で評価します。

付属YAMLでは、予期せぬ連続走行を防ぐため候補を**内蔵速度制御1条件だけ**
にしてあります。PIDとLADRCは、始動補償と停止が各1条件で検証できた後に
別の設定ファイルで追加してください。

### まずドライラン

```bash
ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/staircase_sweep_example.yaml
```

このコマンドではモータは動きません。

### 1条件のみ実機で確認

事前に車輪を浮かせ、非常停止装置を準備し、他のbringupと
`/cmd_vel` 発行ノードを終了してから次を実行します。

```bash
ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/staircase_sweep_example.yaml \
  --execute --i-have-e-stop
```

確認文の入力後、単一候補の速度指令を送信します。初期例は
`velocity` のみで、PID/LADRCは含みません。
実機の動作が不安定な場合はすぐ物理非常停止を使用してください。

## ステージごとの評価

従来の `summary.csv`（速度RMSE/電流RMSなど）に加え、各試験フォルダに
`stage_metrics.csv` を保存します。

`stage_metrics.csv` は、記録された `cmd_vel.csv` の指令値の変化時刻を
区間境界として識別し、各非ゼロ速度ステージについて以下を集計します。

- 区間開始・終了・保持時間・指令速度
- 左右車輪別の速度RMSE、左右合成RMSE
- 区間最後の30%における平均速度偏差と標準偏差

ここで「最後30%」は定常到達を保証しません。値が収束していない場合は
定常偏差と呼ばず、**区間末尾平均偏差**として報告します。

実装ファイル：
- `scripts/stage_metrics.py`（ロギング解析）
- `test/test_stage_metrics.py`（モータに指令を送らない単体テスト）
- `scripts/run_gain_sweep.py`（候補実行後の区間別CSV保存）
- `config/staircase_sweep_example.yaml`（短距離の階段指令）

テストは次のコマンドで実行できます。

```bash
python3 ~/ros2_ws/src/zlac8015d_can_bringup/test/test_stage_metrics.py
python3 ~/ros2_ws/src/zlac8015d_can_bringup/test/test_active_state_parser.py
ros2 run zlac_can_bringup run_gain_sweep.py \
  --config ~/ros2_ws/src/zlac8015d_can_bringup/config/staircase_sweep_example.yaml
```

**検証範囲：** GitHub上にコードと初期テストを追加しましたが、
ROS 2 Humbleでのビルドや実機試験は未実施です。
最初は空転で単一条件の安全停止を確認してから接地試験に移ってください。
