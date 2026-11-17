# MIMO-LADRC・ROS 2実機制御・cmd_vel応答試験：ソースコード技術解説

> 対象：Ubuntu 22.04 / ROS 2 Humble / ZLAC8015D CANopen / 対向二輪型移動ロボット  
> 作成基準：2026-11-17時点にGitHubで確認したソースコード。**設計の一般論ではなく、実装されている処理を説明する。**  
> **この文書は解説のみ**。制御プログラムやパラメータを変更するものではない。

## 0. 説明対象と実装の所在

**重要：`experiment/cmdvel-response-logger` は計測・起動用の `zlac8015d_can_bringup` リポジトリのブランチであり、MIMO-LADRCの演算本体は別リポジトリにある。** 両者を組み合わせた実験構成を本書の対象とする。

| 用途 | リポジトリ | ブランチ | 確認したコミット |
|---|---|---|---|
| ROS 2起動・試験指令・ログ保存 | [zlac8015d_can_bringup](https://github.com/shozawakoki/zlac8015d_can_bringup/tree/experiment/cmdvel-response-logger) | `experiment/cmdvel-response-logger` | `0688f009f16f52ebb57882b8b1bc3edbe5143c5c` |
| MIMO-LADRC、ESO、電流/CAN通信、安全監視 | [zlac8015d_hardware](https://github.com/shozawakoki/zlac8015d_hardware/tree/experiment/mimo-ladrc-no-breakaway) | `experiment/mimo-ladrc-no-breakaway` | `6375092965dd62762990edf37cfd5047b6465cc8` |

主要なソース（ファイル名を選択すると対象に移動できる）：

| ファイル | 主な責務 |
|---|---|
| [`hardware/include/zlac8015d_hardware/mimo_control.hpp`](https://github.com/shozawakoki/zlac8015d_hardware/blob/experiment/mimo-ladrc-no-breakaway/include/zlac8015d_hardware/mimo_control.hpp) | 2×2座標変換、入力行列、一次MIMO-LADRC/LESO、MIMO-PID、過速度判定 |
| [`hardware/include/zlac8015d_hardware/zlac8015d_system.hpp`](https://github.com/shozawakoki/zlac8015d_hardware/blob/experiment/mimo-ladrc-no-breakaway/include/zlac8015d_hardware/zlac8015d_system.hpp) | ROS 2 `hardware_interface::SystemInterface` 宣言、状態変数、パラメータ |
| [`hardware/src/zlac8015d_system.cpp`](https://github.com/shozawakoki/zlac8015d_hardware/blob/experiment/mimo-ladrc-no-breakaway/src/zlac8015d_system.cpp) | CANopen設定、エンコーダ読出し、制御モード選択、`read()`/`write()`、電流送信、保護、内部CSV |
| [`bringup/launch/zlac_can_bringup.launch.py`](launch/zlac_can_bringup.launch.py) | `robot_state_publisher`、`ros2_control_node`、2つのコントローラを起動 |
| [`bringup/urdf/orange_go_2025.urdf`](urdf/orange_go_2025.urdf) | 車輪ジョイント、`ros2_control` インターフェース、制御パラメータ |
| [`bringup/config/controllers.yaml`](config/controllers.yaml) | 制御周期、差動二輪速度変換、オドメトリ |
| [`bringup/scripts/run_cmdvel_trial.py`](scripts/run_cmdvel_trial.py) | 指定時間の`/cmd_vel`送信、`/joint_states`・`/odom`・内部ログの保存 |
| [`bringup/scripts/cmdvel_trial_core.py`](scripts/cmdvel_trial_core.py) | 車輪速度の変換、入力範囲検証、発行者競合判定 |
| [`bringup/CMakeLists.txt`](CMakeLists.txt)・[`package.xml`](package.xml) | 起動設定と試験スクリプトのインストール、ROS依存関係 |
| [`hardware/tests/test_no_breakaway_ladrc.cpp`](https://github.com/shozawakoki/zlac8015d_hardware/blob/experiment/mimo-ladrc-no-breakaway/tests/test_no_breakaway_ladrc.cpp)・[`bringup/tests/test_cmdvel_trial_core.py`](tests/test_cmdvel_trial_core.py) | 制御モデル・速度境界のオフライン単体試験 |

以降、`hardware` は `zlac8015d_hardware`、`bringup` は `zlac8015d_can_bringup` の略記とする。

## 1. システム全体のデータフロー

~~~mermaid
flowchart TD
  A["試験ノード run_cmdvel_trial.py<br/>v_ref, omega_ref, duration"] -->|"/cmd_vel : Twist"| B["diff_drive_controller"]
  B -->|"左右目標角速度 rad/s"| C["ros2_control / hardware_interface"]
  C --> D{"制御モード"}
  D -->|"mimo_ladrc_no_breakaway"| E["MIMO-LADRC + LESO<br/>出力: I_L, I_R"]
  D -->|"mimo_ladrc_torque"| F["BREAKAWAY/TRACK<br/>既存のMIMO-LADRC"]
  D -->|"mimo_pid_torque"| G["MIMO-PID"]
  E --> H["電流制限 ±8 A / 20 A/s"]
  H --> I["ZLAC8015D CANopen<br/>RPDO0: 左右目標電流"]
  I --> J["左右モータ / エンコーダ"]
  J -->|"TPDO0,TPDO1,TPDO2"| K["hardware read(): 速度・電流・位置"]
  K --> E
  K --> L["/joint_states と /odom"]
  L --> A
  E --> M["/tmp/mimo_ladrc_no_breakaway_200hz.csv"]
  M --> A
  A --> N["試験ごとのCSV / metadata.json"]
~~~

- `/cmd_vel` は**機体の目標並進速度 [m/s] と目標旋回角速度 [rad/s]**。これを受けるのは `diff_drive_controller`。
- `hardware_interface` の指令インターフェースは `velocity` [rad/s] であっても、`mimo_ladrc_no_breakaway` モードでは内部演算した **左右目標電流 [A]** をドライバへ送る。
- `joint_state_broadcaster` はジョイントの位置・速度・effortを公開する。`/odom` は差動二輪オドメトリから算出される。
- Python試験ノードは**制御則を実装せず、一定速度を指令して観測する**。20 Hzで指令し、hardware側は200 Hzで更新する。20 Hz・200 Hz・ROSトピック配信レートは同一ではない。

## 2. 対向二輪モデルと各ベクトルの順序

### 2.1 座標と符号

以下は実装の **左→右** の順序（過去の右→左の式をそのまま代入しないこと）：

\[
\boldsymbol q=\begin{bmatrix}\omega_L\\\omega_R\end{bmatrix},
\quad
\boldsymbol u=\begin{bmatrix}I_L\\I_R\end{bmatrix},
\quad
\boldsymbol y=\begin{bmatrix}v\\\omega\end{bmatrix}.
\]

ロボット前進を左右車輪の正回転、反時計回り旋回を \(\omega>0\) とする。車輪半径 \(r=0.064\;\mathrm{m}\)、車輪間隔 \(\ell=0.354\;\mathrm{m}\)。

\[
v=\frac r2(\omega_L+\omega_R),\qquad
\omega=\frac r\ell(\omega_R-\omega_L).
\]

行列形式：

\[
\boldsymbol y=\boldsymbol T\boldsymbol q,\qquad
\boldsymbol T=
\begin{bmatrix}
r/2&r/2\\
-r/\ell&r/\ell
\end{bmatrix}
=
\begin{bmatrix}
0.032&0.032\\
-0.18079&0.18079
\end{bmatrix}\,.
\]

逆変換：

\[
\omega_{L,\mathrm{ref}}=\frac{v_{\mathrm{ref}}}{r}
-\frac{\ell\,\omega_{\mathrm{ref}}}{2r},\qquad
\omega_{R,\mathrm{ref}}=\frac{v_{\mathrm{ref}}}{r}
+\frac{\ell\,\omega_{\mathrm{ref}}}{2r}.
\]

- 実装：`DifferentialDriveModel::wheel_to_body()`、`body_to_wheel()`。
- Python側も `cmdvel_trial_core.py::wheel_speeds()` と `body_speeds()` に同じ式がある。
- 例：`v=1.0 m/s, omega=1.0 rad/s` の目標車輪周速度は左 **0.823 m/s**、右 **1.177 m/s**（角速度は約12.859 / 18.391 rad/s）。

### 2.2 公称制御対象

各車輪の一次速度モデルを、目標電流を入力として

\[
\dot{\boldsymbol q}(t)=
\underbrace{\mathrm{diag}(b_L,b_R)}_{\boldsymbol D_b}
\boldsymbol u(t)+\boldsymbol f_q(t)
\]

と近似する。`b_L=b_R=18` [(rad/s²)/A] はURDFに指定された公称値であり、静止摩擦・車輪間干渉・電流追従誤差・無駄時間をすべて含む物理的に厳密なモデルを意味しない。

機体座標へ変換すると、

\[
\dot{\boldsymbol y}=
\underbrace{\boldsymbol T\boldsymbol D_b}_{\boldsymbol B}
\boldsymbol u+\boldsymbol f,\qquad
\boldsymbol f=\boldsymbol T\boldsymbol f_q.
\]

\[
\boldsymbol B=
\begin{bmatrix}
rb_L/2 & rb_R/2\\
-rb_L/\ell & rb_R/\ell
\end{bmatrix}.
\]

公称ゲインが左右18の場合、

\[
\boldsymbol B\approx
\begin{bmatrix}
0.576 & 0.576\\
-3.25424 & 3.25424
\end{bmatrix}.
\]

`DifferentialDriveModel` コンストラクタが `T`、`T_inv`、`B`、`B_inv` を構築する。`acceleration_to_current()` は `B_inv` を乗じ、機体座標の仮想加速度を左右の**指令電流**に変換する。

**識別モデルとの関係：** `b0=18` が妥当かは実験条件と入力電流の追従性に依存する。`z2` は実機トルクの直接計測値ではなく、上式の公称モデルに対する**等価外乱加速度**である。

## 3. MIMO-LADRC と LESO の数式・C++対応

### 3.1 一次LADRCの状態拡張

機体座標の観測対象 `y=[v,omega]` に対して、`z1` を速度推定、`z2` を総外乱推定とする。

\[
\begin{aligned}
\dot{\boldsymbol z}_1 &=
\boldsymbol z_2+\boldsymbol B\boldsymbol u_{\mathrm{prev}}
+2\boldsymbol\Omega_o(\boldsymbol y-\boldsymbol z_1),\\
\dot{\boldsymbol z}_2 &=
\boldsymbol\Omega_o^2(\boldsymbol y-\boldsymbol z_1),\\
\boldsymbol\Omega_o&=\mathrm{diag}(\omega_{o,v},\omega_{o,\omega}).
\end{aligned}
\]

現行値：`mimo_wo_v=20`、`mimo_wo_omega=20` [rad/s]。一次制御対象なので、ESOの推定状態は各軸について **速度＋総外乱** の2階層、全体で4スカラー状態。

`mimo_control.hpp` の `Ladrc::step()`（抜粋）：

~~~cpp
const Vec2 known_acceleration = multiply(model.B, previous_current);
for (std::size_t i = 0; i < 2; ++i) {
  const double error = measured[i] - z1[i];
  const double z1_dot =
    z2[i] + known_acceleration[i] + 2.0 * wo[i] * error;
  const double z2_dot = wo[i] * wo[i] * error;
  z1[i] += dt * z1_dot;
  z2[i] += dt * z2_dot;
}
~~~

上記は**陽的Euler離散化**。200 Hzの標準周期で `dt≈0.005 s`。実際の `dt` は `write()` の `period.seconds()` を取り、異常な値なら0.005秒に代替し、0.001～0.020秒にクリップする。センサの値が毎周期更新される保証はなく、ESOは最新の保持値を使って計算する。

**`previous_current` は実測電流ではない。** `write()` は前回の `left_ladrc_.last_current_cmd / right_ladrc_.last_current_cmd` をESOに渡す。よってドライバ内部の電流応答遅れ、指令と実電流の差は総外乱側へ入り得る。`hw_efforts_` の実電流はROSの記録・監視用であり、このLESOの入力には直接採用していない。

### 3.2 制御則：速度追従＋外乱補償

\[
\boldsymbol a_{\mathrm{des}}=
\boldsymbol a_{\mathrm{ff}}+
\boldsymbol\Omega_c(\boldsymbol y_{\mathrm{ref}}-\boldsymbol z_1)
-\boldsymbol z_2,\qquad
\boldsymbol u_{\mathrm{raw}}=
\boldsymbol B^{-1}\boldsymbol a_{\mathrm{des}},
\]

\[
\boldsymbol\Omega_c=
\mathrm{diag}(\omega_{c,v},\omega_{c,\omega}).
\]

現行値：`mimo_wc_v=5`、`mimo_wc_omega=5` [rad/s]。`a_ff` は停止時の減速フィードフォワードで、通常の一定速度指令時はゼロ。

~~~cpp
desired_acceleration[i] =
  acceleration_feedforward[i]
  + wc[i] * (reference[i] - z1[i])
  - z2[i];
return model.acceleration_to_current(desired_acceleration);
~~~

- `reference`：目標車輪角速度 `control_ref_l/r` を `T` で変換した `[v_ref, omega_ref]`。
- `measured`：エンコーダ速度を同じ `T` で変換した `[v, omega]`。
- `z1`：フィードバックに使うESOの機体速度推定。
- `z2`：総外乱（v方向 [m/s²]、旋回方向 [rad/s²]）。
- `raw`：安全制限適用**前**の左右電流指令 [A]。

直進中に左輪・右輪で異なる負荷が生じたとき、`z2` のうち旋回方向の成分は差分外乱に相当する。ただしESOの「外乱」は外部から加わった負荷だけではなく、モデル誤差も含む。

### 3.3 初期化と「BREAKAWAYなし」の実装

`src/zlac8015d_system.cpp::on_init()` にて文字列 `mimo_ladrc_no_breakaway` が指定されると、内部列挙値は `MIMO_LADRC_TORQUE`、補助フラグ `experimental_no_breakaway_=true` となる。

`write()` は、通常のBREAKAWAY状態遷移に入る**前**に以下の実験用分岐を通る：

1. `zero_stop_hold_` ではなく目標が非ゼロならMIMO演算へ進む。**車輪が回り始めたことの確認は不要**。
2. 初回のみ `mimo_ladrc_.reset_to_measurement(measured_body)`：`z1=実測[v,omega]`、`z2=[0,0]` とする。
3. `mimo_ladrc_.step()` でESOを更新し、同じ周期のMIMO-LADRC電流を算出する。
4. 電流の大きさと変化率を制限し、左右のCANopen目標電流へ変換して送信する。
5. `zero_stop_hold_` または有効な速度参照なしの区間ではESOを `reset()` し、0 Aを送信する。通常の非ゼロ速度への復帰時には再初期化する。

したがって **BREAKAWAY電流の段階的増加、TRACK移行判定、TRACK電流下限の強制、方向反転時のBREAKAWAY再突入**はこの実験用分岐では使われない。一方、ソース内に `compute_current_command()` やBREAKAWAY関連の状態変数が残っているのは、**他の制御モードとの共存のため**であり、「BREAKAWAYなし」モードで実行されるという意味ではない。

**発進可能性の限界：** ESOが摩擦を総外乱として扱うことと、静止摩擦に必ず打ち勝てることは同義ではない。静止中に`z2`が補償方向へ増えても、電流上限・実電流追従性・摩擦の非線形性・機構負荷によっては動き出さない。

## 4. 直進性能：なぜ現在の「MIMO」が実質左右独立になるか

機体座標の制御ゲインが等しい場合、

\[
\boldsymbol\Omega_c=5\boldsymbol I,\quad
\boldsymbol\Omega_o=20\boldsymbol I.
\]

車輪座標での速度誤差を `e_q=q_ref - T^{-1}z1` とすると、

\[
\boldsymbol T^{-1}\boldsymbol\Omega_c\boldsymbol T
=\boldsymbol T^{-1}(5\boldsymbol I)\boldsymbol T
=5\boldsymbol I.
\]

ESOも同じ等ゲインなので、`T^{-1}z1`、`T^{-1}z2` と座標変換すれば車輪ごとの独立なESOに還元できる（理想的な同一サンプル・変換・初期化条件）。結果として生の電流指令は

\[
I_L=\frac{5(\omega_{L,\mathrm{ref}}-\hat\omega_L)-\hat f_L}{b_L},
\qquad
I_R=\frac{5(\omega_{R,\mathrm{ref}}-\hat\omega_R)-\hat f_R}{b_R},
\]

という車輪別の式に変換できる。**`[v,omega]` を使っているだけでは追加の左右協調効果は発生しない**。

左右で異なる外乱は

\[
f_v=\frac r2(f_{q,L}+f_{q,R}),\qquad
f_\omega=\frac r\ell(f_{q,R}-f_{q,L})
\]

として機体速度へ影響する。左右速度の追従誤差 `e_R-e_L` が残れば、実際の旋回速度偏差は `(r/ell)(e_R-e_L)` に比例し、直進軌跡が曲がる。しかし**独立制御でも各輪の外乱を十分補償できれば直進は可能**。性能低下を「MIMOではないから」とだけ断定しないこと。

### 4.1 異なる帯域幅にすると何が変わるか

例えば `wc_v=5`、`wc_omega=10` と設定した場合、車輪座標の誤差フィードバック行列は

\[
\boldsymbol T^{-1}
\begin{bmatrix}5&0\\0&10\end{bmatrix}
\boldsymbol T
=
\begin{bmatrix}7.5&-2.5\\-2.5&7.5\end{bmatrix}.
\]

非対角の `-2.5` により、片側の車輪速度誤差がもう片側への電流演算にも入る。これが**左右輪を結合させる**一例である。観測器側の帯域を異ならせても同様に座標間の結合が生じる。

ただし `wc_omega` を上げるとヨー補正が強くなる一方、無駄時間、ノイズ、ESO帯域、電流飽和によっては不安定化・オーバーシュートが増える。**今回のブランチではゲインを変更しておらず、現在も `wc=[5,5]`、`wo=[20,20]`。** 新しい数値はあくまで数学的な説明例である。

### 4.2 片輪拘束をMIMOフィードバックだけで安全停止できない理由

片輪が実際に回れない状態では、与えた `v_ref,omega_ref` が同時に実現できない場合がある。通常のLADRCは誤差を解消しようと電流を増加させ得る。現行実験ブランチでは、**片輪の拘束検出に基づいて他輪を制動する保護は削除済み**。よって正常な追従制御と異常停止を混同しない。手で回転車輪を止める実験は行わない。

## 5. 電流指令生成・制限

`write()` のBREAKAWAYなし分岐は、MIMO-LADRCからの生電流 `raw` に対して、車輪ごとに

\[
I_{\mathrm{amp}}[k]
=\mathrm{clip}(I_{\mathrm{raw}}[k],-8,8),
\]

\[
I_{\mathrm{cmd}}[k]
=\mathrm{clip}
\left(I_{\mathrm{amp}}[k],
I_{\mathrm{cmd}}[k-1]-20\Delta t,\,
I_{\mathrm{cmd}}[k-1]+20\Delta t\right)
\]

を適用し、最後に再び±8 Aへクリップする。既定の周期5 msなら1周期あたりの指令電流変化幅は **±0.1 A**。

- `current_limit_a=8.0`（URDF）と実験用 `no_breakaway_current_limit_a=8.0`（コード既定値）。実験用パラメータを指定する場合、全体上限を超える値は起動時に拒否する。
- `current_rate_limit_a_per_s=20.0`。電流の大きさの制限と「変化の速さ」の制限は別物。
- `I_cmd` は**ドライバへ送った目標電流**、`I_actual` はドライバから返された実電流。両者は等しいとは限らない。
- 起動時の強制最小電流は設けない。低速目標では最初の電流が静止摩擦閾値より小さい可能性がある。
- 本実験モードに片輪ストール時の低電流上限や「2 A継続で遮断」という機能はない。**拘束状態で8 Aが持続する危険性**に注意する。

## 6. ROS 2とCANopenの通信処理

### 6.1 起動・制御インターフェース

`launch/zlac_can_bringup.launch.py` は `robot_state_publisher`、`ros2_control_node`、`joint_state_broadcaster`、`diff_drive_controller` のspawnerを起動する。

`controllers.yaml`：
- `controller_manager.update_rate=200`：制御器を公称200 Hzで更新。
- `diff_drive_controller.use_stamped_vel=false`：`geometry_msgs/msg/Twist` の速度指令。
- 左右ジョイント：`left_wheel_joint` / `right_wheel_joint`。
- `wheel_radius=0.064`、`wheel_separation=0.354`。
- `open_loop=false`、`position_feedback=true`、`publish_rate=50.0`：エンコーダ状態を利用したオドメトリ。

`launch` のremapにより `/diff_drive_controller/cmd_vel_unstamped` を `/cmd_vel` に、`/diff_drive_controller/odom` を `/odom` に対応付ける。

`urdf/orange_go_2025.urdf` では `hardware_interface` の `velocity` 指令・`position/velocity/effort` 状態インターフェースを宣言する。**`control_mode` のURDF既定値は `mimo_ladrc_torque`**（BREAKAWAYあり）。BREAKAWAYなしの実験は環境変数を明示する：

~~~bash
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway \
ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
~~~

プログラムの `on_init()` で環境変数がURDFの指定より優先される。モード変更は稼働中に切り替えるものではなく、プロセスを停止して再起動する。

### 6.2 `hardware_interface` の主要メソッド

| メソッド | 実際の処理 |
|---|---|
| `on_init()` | パラメータ読込・妥当性検証、制御モード・行列`B`の設定 |
| `on_activate()` | `can0`をSocketCANでopen/bind、PDOの再マッピング、Profile Torque/Velocityの設定、サーボON、状態初期化 |
| `read()` | CANフィードバックを読み、`hw_velocities_`・`hw_efforts_`・`hw_positions_`を更新 |
| `write()` | フィードバック監視、停止状態処理、ESO/制御則、電流制限、RPDO送信、CSVログ |
| `on_deactivate()` | 0指令、サーボの状態遷移、CANソケットclose |
| `send_zero_command()` | トルクモードではRPDOの左右目標電流を0にし、内部`last_current_cmd`も0にする |

`read()` はCANソケットから**受信可能なフレームを繰り返し読んで更新**する。`write()` は200 Hzループにて最新値を利用する。センサフィードバック周期と制御周期の一致は前提ではない。

### 6.3 CANopenの主な対象と数値変換

| CANフレーム | オブジェクト | 内容 | C++での単位変換 |
|---|---|---|---|
| TPDO0 `0x180+node_id` | `606Ch:01`、`:02` | 左右実速度（I32、0.1 rpm） | 左は符号反転・右はそのまま、`0.01047197551` を掛けてrad/s |
| TPDO1 `0x280+node_id` | `6077h:01`、`:02` | 左右実電流（I16、0.1 A） | 左は符号反転・右はそのまま、`0.1`を掛ける |
| TPDO2 `0x380+node_id` | `6064h:01`、`:02` | 左右位置（I32 encoder counts） | 初期位置を0とし積算、`2π/4096` rad/count、左は符号反転 |
| RPDO0 `0x200+node_id` | `6071h:01`、`:02` | 左右目標電流（I16、mA） | `I_L`は符号反転し×1000、`I_R`は×1000 |
| RPDO1 `0x300+node_id` | `60FFh:03` | 速度制御モードの目標速度 | 0.1 rpm単位へ変換（本実験モードでは使わない） |

`node_id=1` ならトルク指令CAN-IDは **`0x201`**。左右電流は同一の4バイトRPDOに梱包して送る。`on_activate()` はトルク用にモード`6060h=4`、速度用には`6060h=3`を設定し、既定のPDO構成を調整する。

**エンコーダの速度符号と電流符号：** この実機では左モータのドライバ正方向がROSの前進正方向と逆であるため、`read()`と`write()`の双方で左側の符号を反転している。MIMO式自体にはROSの前進・反時計回りを基準とする値を渡す。

**位置の解釈：** `6064h` の32bitオーバーフローを `wrapped_count_delta()` で補正し、起動後の累積カウントを位置へ変換する。`4096 counts/rev` の設定を実際のエンコーダと照合すること。

## 7. 停止、過速度監視、フェイルセーフの実際

### 7.1 通常の `cmd_vel=0`

`write()` はゼロ速度指令を受けたとき、走行中ならその瞬間の車輪実速度を出発点として`normal_stop_ref_*` をゼロへ近づける。コード値は **機体減速度相当 1.0 m/s²**、車輪角減速度では `1.0/0.064=15.625 rad/s²`。LADRCの `a_ff` にもこの減速成分を加える。

両車輪の実測値がそれぞれ **0.50 rad/s以下**になったら `zero_stop_hold_=true` となり、0 Aを保持する。ここでいう減速ランプは**入力目標のランプ**であり、実機の減速度が厳密に1.0 m/s²になる保証ではない。0 Aは機械ブレーキではない。

### 7.2 車輪周速度2.0 m/sに達した場合

`read()` はTPDO0受信ごとに

\[
\max(|\omega_L|,|\omega_R|)\ \ge\
\frac{2.0}{0.064}=31.25\ \mathrm{rad/s}
\]

を判定する。左右どちらかが超過すれば `no_breakaway_overspeed_latched_=true` にし、`write()` の先頭で通常のLADRCより**優先**して **左右0 A** を送る。

- 正転・逆転の絶対値で比較。車輪周速度 **2.0 m/s（約298.4 rpm）** に対応し、機体重心の実並進速度2 m/sではない。
- フィードバックが閾値以下へ下がっても、`/cmd_vel=0`でも**解除しない**。`on_activate()`（制御の再初期化）で解除される。
- CSVの `wheel_overspeed_latched` は0/1。起動時識別子は `MIMO_LADRC_NO_BREAKAWAY_EXP_20261115_R5`。
- **これは実験モード専用のソフトウェア停止**。別モードへ自動的に広がるものではない。
- この判定はCANの速度フィードバックを受信・処理できることが前提。CAN途絶に対しては別のフィードバックタイムアウト監視を使う。
- 0 Aでは慣性で車輪が回り続ける可能性がある。衝突・転倒を防止する安全認証済みの停止機能と同一視しない。

### 7.3 フィードバック欠落・異常値

- 速度TPDO未受信の間は電流0を指令。
- 最終速度TPDO受信から**30 ms超**なら`send_zero_command()` して `ERROR` を返す（`feedback_timeout_ms=30`）。
- MIMO-LADRCが非有限な電流を生成した場合も0 Aを試行し `ERROR`。
- 絶対電流上限`8 A`と変化率`20 A/s`を適用する。
- **片輪拘束を検知して他輪を制動する安全ロジックは存在しない**。拘束中に実速度が小さくても、誤差補償のため8 Aに達し得る。
- 実験スクリプト側の **1.5 m/sの中断閾値**はハードウェア側の2.0 m/sより低いが、こちらは`/cmd_vel=0`送信というソフトウェア操作であり、0 A直送ではない。独立の非常停止が不可欠。

## 8. 比較対象のPID・通常LADRC・速度モード

同じhardwareソースは、以下のモードを選択できる（実験記録では指定を厳密に保存すること）。

| `ZLAC_CONTROL_MODE` | 制御入力・出力 | 発進の扱い |
|---|---|---|
| `velocity` | 左右速度参照をドライバ内蔵速度制御へ | モータドライバ内蔵速度ループ |
| `ladrc_torque` | 車輪別SISO-LADRCで左右電流指令 | 既存BREAKAWAY/TRACK |
| `pid_torque` | 車輪別PIDで左右電流指令 | 既存BREAKAWAY/TRACK |
| `mimo_ladrc_torque` | 機体座標MIMO-LADRCで左右電流指令 | **既存の**BREAKAWAY→TRACK移行後にMIMO演算 |
| `mimo_pid_torque` | 機体座標MIMO-PIDで左右電流指令 | 既存のBREAKAWAY/TRACK |
| `mimo_ladrc_no_breakaway` | 機体座標MIMO-LADRCで左右電流指令 | **初回非ゼロ参照からLADRC**、BREAKAWAYなし。今回の推奨計測対象 |

### 8.1 MIMO-PIDの式とプログラム

`mimo_control.hpp::Pid::step()` は、機体座標の `e=y_ref-y` に対し

\[
\boldsymbol a_{\mathrm{PID}}=
\bar b\left(
\boldsymbol K_p\boldsymbol e+
\boldsymbol K_i\int\boldsymbol e\,dt-
\boldsymbol K_d\dot{\boldsymbol y}_{\mathrm{filtered}}
\right)+\boldsymbol a_{\mathrm{ff}},\qquad
\boldsymbol u_{\mathrm{PID}}=\boldsymbol B^{-1}\boldsymbol a_{\mathrm{PID}}.
\]

`\bar b=(b_L+b_R)/2`。初期パラメータは各軸 `Kp=0.7`、`Ki=0.5`、`Kd=0.03`、微分フィルタ `tau=0.05 s`。P/Iはコントローラ周期、Dは **新しい速度TPDO0が入ったときだけ** 更新し、実測サンプル時刻差を用いる。微分には目標値の差分ではなく実測機体速度を使用し、目標速度ステップ時の微分キックを抑える。

MIMO-PIDは両輪の候補電流が飽和に近づくと積分を保留する条件付きアンチワインドアップを用いる。**MIMO-PIDとMIMO-LADRCを同じ「ESOのz2を用いる」制御として扱わない。** PIDモードのCSVにあるESO関連列は診断用の値・ゼロであり、総外乱推定結果ではない。

### 8.2 車輪別のSISO-LADRC/PID

`compute_current_command()` は、各車輪独立のESO/PID制御、旧BREAKAWAY、TRACK電流下限、ストール復帰などの機能を実装する。通常の `ladrc_torque`、`pid_torque` や旧MIMOモードでは関係する。一方、**`mimo_ladrc_no_breakaway` はこの関数に進まず、前述の専用分岐からCAN送信後に`return`する**。

## 9. 試験ノードの内部構造（Python）

### 9.1 `arguments()` と入力チェック

コマンド例：

~~~bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.10 --angular 0.20 --duration 3.0 --arm
~~~

- `--linear`：目標並進速度 [m/s]、`--angular`：目標旋回速度 [rad/s]、`--duration`：非ゼロ速度を送る時間[s]。
- `--rate`：指令送信周期、既定20 Hz、許容10～100 Hz。
- `--post-stop`：停止後の0速度指令・観測、既定3.0秒。
- `--max-linear`/`--max-angular`：両軸とも既定かつ最大 **1.0**。正負両方向を許す。
- `--abort-wheel-speed`：Python側の中断閾値、既定**1.5 m/s**、2.0 m/s未満のみ指定可能。
- `--feedback-timeout`：`/joint_states`欠落検出、既定0.5秒。
- `--arm`：実際の移動指令を出す明示的フラグ。これがないと開始拒否。

`cmdvel_trial_core.py::validate_trial()` は全値有限、非ゼロ指令、指定時間0.1～60秒、各上限、**変換後の左右車輪周速度の目標値**が中断上限未満であることを確認する。例えば`v=1.0`、`omega=1.0`なら左右車輪周速度が0.823/1.177 m/sなので既定制約内に入る。

**入力チェックは実測速度の保証ではない。** 応答時の速度超過は試験中の `_guard_motion()` で判定する。

### 9.2 `CmdVelTrial` の主要メソッド

| メソッド | 動作 |
|---|---|
| `__init__()` | `/cmd_vel` publisher、`/joint_states`・`/odom` subscriber、3種のROS CSVを準備 |
| `preflight()` | 最大8秒、速度・オドメトリデータが届くこと、`/cmd_vel`購読があること、他の指令発行者がいないことを確認 |
| `publish(v,w)` | `Twist.linear.x` と `Twist.angular.z` を送信し、`commands.csv`に時刻付きで記録 |
| `on_joint()` | 左右関節名を探し、角位置、角速度、effort、エンコーダ換算`v,omega`を記録 |
| `on_odom()` | `/odom`の位置・姿勢Quaternion・Twistを記録 |
| `_guard_motion()` | 他の`/cmd_vel` publisher、`/joint_states`タイムアウト、実測車輪周速度1.5 m/sを監視 |
| `_phase()` | 期限まで一定周期で指令を発行し、ROSコールバックを実行 |
| `run_trial()` | 非ゼロ速度指令→0速度指令→停止後記録 |
| `send_zero_for()` | 中断・終了後にソフトウェア上の0速度指令を繰り返す |
| `summary()` | サンプル数、実験条件、速度RMSE、エラーをJSON化 |
| `copy_hardware_csv()` | 実験終了時に`/tmp`のhardware内部200 Hz CSVを試験フォルダへコピー |
| `main()` | 引数処理、試験フォルダ作成、ROS起動・終了、保存 |

`preflight()` は他の`/cmd_vel`発行ノードを検出したら実験開始を拒否する。コントローラの起動自体はこのスクリプトでは行わない。

### 9.3 停止時の挙動と限界

指令時間終了後は0速度を送って`post-stop`の計測を続ける。Ctrl+C・SIGTERM・異常時は可能な限り0速度を繰り返す。ただし**Pythonプロセス停止後の制御や通信途絶中の停止は保証できない**。

取得する`/joint_states`の関節名は `left_wheel_joint`/`right_wheel_joint` 固定。関節名が見つからないデータは無効としてカウントする。`effort` が存在しないサンプルは`nan`にする。

## 10. CSVの解釈と評価指標

### 10.1 試験フォルダの内容

~~~text
~/ros2_ws/mimo_cmdvel_trials/<UTC日時>_<v条件>_<omega条件>/
  commands.csv                    # 送信したv_ref,omega_ref と時刻
  joint_states.csv                # 位置、左右速度、実電流、車輪換算v,omega
  odom.csv                        # オドメトリ姿勢・速度
  hardware_mimo_ladrc_200hz.csv   # hardware内部ログ（存在・鮮度チェック後コピー）
  metadata.json                   # 条件と集計結果／実験終了理由
~~~

**`hardware_mimo_ladrc_200hz.csv` は常に存在するとは限らない。** Python側は `/tmp/mimo_ladrc_no_breakaway_200hz.csv` を終了時にコピーする。ファイルがない、古い、コピー失敗の場合は `metadata.json` の `hardware_csv.copied=false` と理由を確認する。コピーはリアルタイム同期されたrosbag記録ではない。

### 10.2 注目する内部CSV列

| 列 | 内容と解釈 |
|---|---|
| `ref_left_rad_s` / `ref_right_rad_s` | ros2_controlに届いた目標車輪角速度。**停止ランプ内の`control_ref`そのものとは限らない** |
| `vel_left_rad_s` / `vel_right_rad_s` | ドライバ速度TPDOから変換した車輪実速度 |
| `pos_left_rad` / `pos_right_rad` | エンコーダ積算角度（起動後の相対位置） |
| `cmd_current_left_A` / `cmd_current_right_A` | LADRCが最終的に送信した目標電流 |
| `actual_current_left_A` / `actual_current_right_A` | ドライバ電流TPDOから取得した実電流 |
| `z1_left` / `z1_right` | 機体LESO推定`z1`を車輪座標に逆変換した速度推定 |
| `z2_left` / `z2_right` | 機体LESO`z2`の車輪座標換算 [rad/s²] |
| `ref_v_m_s` / `ref_omega_rad_s` | `hw_commands`から変換した機体速度目標 |
| `v_m_s` / `omega_rad_s` | エンコーダ車輪速度から換算した機体速度 |
| `hat_v_m_s` / `hat_omega_rad_s` | MIMO-LESOの速度推定`z1` |
| `hat_d_v_m_s2` / `hat_d_omega_rad_s2` | MIMO-LESOの総外乱推定`z2` |
| `tracking_left/right` | 実験分岐でLADRC演算を実行中かを示すフラグ。BREAKAWAY→TRACK完了の証明には使えない |
| `breakaway_target_left/right_A` | 既存共通ログ項目。BREAKAWAYなしモードでは起動時に0へ設定され、通常の始動電流として使われない |
| `mimo_pid_*` | PID用の診断列。MIMO-LADRCではPID性能として解釈しない |
| `wheel_overspeed_latched` | 車輪周速度2 m/s超過による0 Aラッチの0/1 |

### 10.3 時刻同期の制約

ROS側`commands.csv`、`joint_states.csv`、`odom.csv`は**同じPython recorderノード**の起動を基準にした `t_s` と、受信時wall-clock `epoch_s`を持つ。対してhardware内部CSVの `time_s` は**最初にログファイルを開いた時刻**を原点としている。

したがって**`commands.csv.time_s` と `hardware_mimo_ladrc_200hz.csv.time_s` をそのまま重ねるのは誤り**。共通の目標速度変化イベントなどを使って遅れを推定するか、将来的に同じ時間基準をhardware側にも記録する必要がある。`/odom`のメッセージ時刻と受信時刻にも差が生じる。

### 10.4 RMSEの定義と現行の集計範囲

`run_cmdvel_trial.py` は非ゼロ速度指令区間中に到着した`/joint_states`サンプルについて

\[
\mathrm{RMSE}_v=
\sqrt{\frac{1}{N}\sum_{i=1}^{N}(v_i-v_{\mathrm{ref}})^2},
\qquad
\mathrm{RMSE}_\omega=
\sqrt{\frac{1}{N}\sum_{i=1}^{N}(\omega_i-\omega_{\mathrm{ref}})^2}
\]

を `metadata.json` に保存する。その他、最大角速度偏差・最大車輪角速度・サンプル数を保存する。

注意点：
1. **指令開始から立ち上がりまでの過渡応答もRMSEに含む**。定常誤差だけの指標ではない。
2. この集計の`v_i`、`omega_i`は**エンコーダから計算した値**。接地すべりや地面に対する横偏差は独立に測定していない。
3. 停止後`post-stop`はCSVに残るが、現状の`RMSE`集計には含まれない。停止時間や残留振動を評価する場合はCSVから別途計算する。
4. 電流指令と実電流、ESO推定値の遅れや飽和時間は**記録されるが自動評価されない**。研究発表用にはグラフと統計値を後処理で計算する。

## 11. 現在の主要パラメータ一覧

| パラメータ | 値 | 所在 |
|---|---:|---|
| 車輪半径 `r` | 0.064 m | URDF・controllers.yaml・MIMOモデル |
| 左右輪間隔 `ell` | 0.354 m | URDF・controllers.yaml・MIMOモデル |
| 左右公称入力ゲイン `b0` | 18 / 18 [(rad/s²)/A] | URDF |
| MIMO `wc_v / wc_omega` | 5 / 5 rad/s | URDF |
| MIMO `wo_v / wo_omega` | 20 / 20 rad/s | URDF |
| MIMO-PID `Kp` | 0.7 / 0.7 | URDF |
| MIMO-PID `Ki` | 0.5 / 0.5 | URDF |
| MIMO-PID `Kd` | 0.03 / 0.03 | URDF |
| PID微分フィルタ時定数 | 0.05 s | URDF |
| hardware/実験用最大指令電流 | 8 A / 8 A | URDF / C++デフォルト |
| 電流変化率 | 20 A/s | URDF |
| 速度フィードバックタイムアウト | 30 ms | URDF |
| 過速度ラッチ | 2.0 m/s / 車輪（31.25 rad/s） | `wheel_overspeed()` |
| 通常停止ランプ | 車輪周速度換算1.0 m/s² | `write()` |
| 停止完了判定 | 両輪それぞれ0.50 rad/s以下 | `write()` |
| controller_manager | 200 Hz | controllers.yaml |
| `/odom`公開 | 50 Hz（パラメータ） | controllers.yaml |
| 試験`/cmd_vel`発行 | 20 Hz（既定） | Python |
| 試験`v`許容 | ±1.0 m/s | Python |
| 試験`omega`許容 | ±1.0 rad/s | Python |
| Python中断条件 | 車輪周速度1.5 m/s | Python |
| 指令時間 | 0.1～60 s | Python |
| 指令終了後の記録 | 3 s（既定） | Python |

**パラメータの優先順：** URDFに書かれた値はC++のデフォルト値を上書きする。`ZLAC_CONTROL_MODE` は `control_mode` のみを上書きする。`no_breakaway_current_limit_a`はURDFに現時点で記載がなく、ソース内の既定値8 Aを使用する。

## 12. ソースを読む推奨順序

1. `bringup/config/controllers.yaml` → 指令とオドメトリの意味・車輪寸法。
2. `bringup/urdf/orange_go_2025.urdf` → ゲイン・モード・電流／通信上限。
3. `hardware/include/.../mimo_control.hpp` → `T`、`B`、`Ladrc::step()`、`Pid::step()`。
4. `hardware/src/zlac8015d_system.cpp::on_init()` → 実験モード選択とパラメータ。
5. 同 `read()` → エンコーダ、実電流、過速度ラッチ。
6. 同 `write()` → 通常停止、ESO入力、BREAKAWAYなし分岐、電流制限とCAN出力。
7. `bringup/scripts/cmdvel_trial_core.py` → 試験入力妥当性。
8. `bringup/scripts/run_cmdvel_trial.py` → 指令発行、異常中断、ログとRMSE。

## 13. 起動と検証（既存の機体操作手順）

制御プロセスを**完全停止**し、両リポジトリを対応ブランチにしてビルドする：

~~~bash
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
cd ~/ros2_ws/src/zlac8015d_hardware
g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
  tests/test_no_breakaway_ladrc.cpp -o /tmp/test_no_breakaway_ladrc
/tmp/test_no_breakaway_ladrc

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
~~~

端末1：MIMO-LADRC制御の起動（車輪を浮かせた状態・物理非常停止を確認）。

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
~~~

端末2：指令とログの試験（他の`/cmd_vel`発行ノードを停止した状態）。

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run zlac_can_bringup run_cmdvel_trial.py \
  --linear 0.10 --angular 0.00 --duration 3.0 --arm
~~~

`--linear 1.0 --angular 1.0` も**ソフトウェアの入力範囲上は指定できる**が、安全性・転倒・制動距離を確認するまでは使用しない。手でタイヤを拘束しない。ログが揃わない場合は`metadata.json`を確認し、古いhardware CSVを新しい試験の記録として扱わない。

## 14. 研究で次に検証すべき論点（実装事実と区別）

ここからは**現行コードから導かれる検討事項**であり、性能向上が実証されたことを意味しない。

- `wc_v` と `wc_omega` を変えた場合の直進時のヨーレートRMSE、立ち上がり、オーバーシュート、外乱応答を比較する。
- `wo` を上げた場合のESO外乱推定の追従遅れとノイズ感度を評価する。
- `b0=18` は公称入力ゲインなので、実電流追従の遅れを含む実機の入出力で再同定し、`I_cmd`と`I_actual`のずれを区別する。
- 静止摩擦の補償効果は、**無負荷発進**と**既知負荷の走行中外乱**を分離して検証する。手による拘束は用いない。
- エンコーダ換算`v,omega`だけでは車輪すべりや真の接地走行軌跡を検証できないため、必要なら独立した自己位置・姿勢観測を追加する。
- 試験CSVの時刻原点を統一し、電流・ESO・速度を同じ時間軸で解析可能にする。現行のコピー方式だけでは厳密なタイムスタンプ同期は実現していない。

---

**要点：** 本実験構成は「`/cmd_vel`による機体速度指令 → 左右速度目標 → エンコーダフィードバック → 機体座標の一次MIMO-LADRC/LESO → 左右電流指令」という流れを実装している。`experiment/cmdvel-response-logger` が指令と記録、`experiment/mimo-ladrc-no-breakaway` が制御とCANopen通信を担う。現行の等帯域設定では左右独立LADRCとの数理的な等価性があること、実際の外乱推定・速度追従・安全性は実験で別途検証することが重要である。
