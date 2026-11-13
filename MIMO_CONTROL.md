# MIMO-LADRC / MIMO-PID による機体速度フィードバック制御

**開発ブランチ：** `feat/mimo-ladrc-pid-straight-tracking`  
**制御本体：** [zlac8015d_hardware（同名ブランチ）](https://github.com/shozawakoki/zlac8015d_hardware/tree/feat/mimo-ladrc-pid-straight-tracking)

本資料は、対向二輪型移動ロボットの電流指令型速度制御を、左右独立のLADRC/PIDから、機体の並進速度・角速度を制御量とする2入力2出力（MIMO）LADRC/PIDへ拡張した**試験用実装**について説明する。

既存の `velocity`・`ladrc_torque`・`pid_torque` モードは削除していない。URDFの初期設定は従来の `ladrc_torque` のままであり、ブランチを切り替えただけでは未検証のMIMO制御は有効にならない。

## 1. 目的と適用範囲

目的は、直進指令中に生じる左右車輪の速度差を、機体角速度 $\omega$ の追従誤差として扱い、左右のモータ電流を補正できる制御系を設計・比較することである。

新しい制御器では、機体並進速度 $v$ と機体角速度 $\omega$ に対して**別々の制御ゲインとESO帯域幅**を設定できる。一方、現行のフィードバックは左右エンコーダの角速度から運動学的に算出した値であり、IMUのヨーレート・姿勢角や位置推定結果は入力していない。

**重要：行列による2入力2出力化だけで直進性能が改善するとは限らない。** 左右の公称動特性が等しく、並進・旋回のゲインも同じなら、左右独立制御を座標変換した方式と数学的に等価になる。車輪の滑り、実効車輪径差などに起因する実際の方位誤差を抑えるには、必要に応じてIMUと上位方位制御・横偏差制御を追加する。

## 2. 制御対象と運動学モデル

**添字・符号規約**

- 左右車輪の順序：$[L,R]$（コードの `hw_velocities_[0]` が左、`[1]` が右）
- 機体速度の順序：$[v,\omega]$
- 左右車輪が同じ正の角速度なら前進
- $\omega>0$ は反時計回り・左旋回（右車輪が左車輪より速い）

車輪半径を $r$、車輪間距離を $\ell$、左右車輪角速度を $\omega_L,\omega_R$ とすると、

$$
\boldsymbol q=
\begin{bmatrix}\omega_L\\ \omega_R\end{bmatrix},\quad
\boldsymbol y=
\begin{bmatrix}v\\ \omega\end{bmatrix}
=\boldsymbol T\boldsymbol q,\quad
\boldsymbol T=
\begin{bmatrix}
r/2 & r/2\\
-r/\ell & r/\ell
\end{bmatrix}.
$$

逆変換は $\boldsymbol q=\boldsymbol T^{-1}\boldsymbol y$ である。

各車輪の公称モデルを

$$
\dot{\boldsymbol q}=
\mathrm{diag}(b_{0L},b_{0R})\boldsymbol u
+\boldsymbol f_q,\qquad
\boldsymbol u=\begin{bmatrix}I_L\\I_R\end{bmatrix}
$$

と近似すると、機体速度座標系では

$$
\boxed{\dot{\boldsymbol y}=\boldsymbol B\boldsymbol u+\boldsymbol d},\qquad
\boldsymbol B=\boldsymbol T\mathrm{diag}(b_{0L},b_{0R}),\qquad
\boldsymbol d=\boldsymbol T\boldsymbol f_q
$$

となる。**制御対象への入力は左右モータの電流指令［A］** であり、車輪角速度はセンサによる出力である。左右のモータを瞬時に希望速度へ変更できると仮定しているわけではない。

現行の設定値

- $r=0.064\ \mathrm{m}$
- $\ell=0.354\ \mathrm{m}$
- $b_{0L}=b_{0R}=18.0\ \mathrm{(rad/s^2)/A}$

を代入すると、

$$
\boldsymbol T\approx
\begin{bmatrix}0.032&0.032\\-0.18079&0.18079\end{bmatrix},\qquad
\boldsymbol B\approx
\begin{bmatrix}0.576&0.576\\-3.25424&3.25424\end{bmatrix}.
$$

したがって、仮想的な機体加速度指令 $\boldsymbol a=[a_v,a_\omega]^\mathsf T$ を電流へ変換する基本計算は

$$
\boxed{\boldsymbol u=\boldsymbol B^{-1}\boldsymbol a}
$$

となる。`mimo_control.hpp` の `DifferentialDriveModel` は $\boldsymbol T,\boldsymbol T^{-1},\boldsymbol B,\boldsymbol B^{-1}$ を作り、行列式がほぼゼロの場合を検出する。

## 3. MIMO-LADRCの計算式

### 3.1 拡張状態オブザーバ（ESO）

機体速度推定値と総外乱推定値を

$$
\boldsymbol z_1=
\begin{bmatrix}\hat v\\\hat\omega\end{bmatrix},\qquad
\boldsymbol z_2=
\begin{bmatrix}\hat d_v\\\hat d_\omega\end{bmatrix}
$$

と定義する。ESOの観測誤差は

$$
\boldsymbol e_o=\boldsymbol y_{\mathrm{meas}}-\boldsymbol z_1
$$

である。ESO帯域幅行列を

$$
\boldsymbol\Omega_o=
\mathrm{diag}(\omega_{o,v},\omega_{o,\omega})
$$

とすると、実装に対応する連続時間の設計式は

$$
\dot{\boldsymbol z}_1=
\boldsymbol z_2+\boldsymbol B\boldsymbol u_{\mathrm{prev}}
+2\boldsymbol\Omega_o\boldsymbol e_o,
\qquad
\dot{\boldsymbol z}_2=\boldsymbol\Omega_o^2\boldsymbol e_o.
$$

$\boldsymbol u_{\mathrm{prev}}$ は**前制御周期に送信した電流指令**であり、CANから取得する実測電流ではない。実装では制御周期 $\Delta t$ を用いた陽的オイラー法で $\boldsymbol z_1,\boldsymbol z_2$ を更新する。ESOは合計4状態である。

### 3.2 電流指令の算出

目標機体速度を $\boldsymbol y_{\mathrm{ref}}=[v_{\mathrm{ref}},\omega_{\mathrm{ref}}]^\mathsf T$、制御帯域幅行列を

$$
\boldsymbol K_c=\mathrm{diag}(\omega_{c,v},\omega_{c,\omega})
$$

とおくと、

$$
\boxed{
\boldsymbol u_{\mathrm{raw}}
=\boldsymbol B^{-1}\left[
\boldsymbol a_{\mathrm{ff}}
+\boldsymbol K_c(\boldsymbol y_{\mathrm{ref}}-\boldsymbol z_1)
-\boldsymbol z_2
\right]
}
$$

である。$\boldsymbol a_{\mathrm{ff}}$ は通常走行時はゼロであり、通常停止の減速ランプ中に車輪角減速度から換算した値を使用する。ESOによる総外乱推定値 $\boldsymbol z_2$ を差し引くことがLADRCの特徴である。

計算した電流は、そのままCANに送るのではなく既存の電流上限・電流変化率制限・TRACK電流下限・停止処理を通す。

## 4. MIMO-PIDの計算式

MIMO-PIDではESOを使用せず、エンコーダ由来の実測機体速度と目標値との差

$$
\boldsymbol e=\boldsymbol y_{\mathrm{ref}}-\boldsymbol y_{\mathrm{meas}}
$$

を直接使用する。比例・積分・微分のゲイン行列は

$$
\boldsymbol K_p=\mathrm{diag}(K_{p,v},K_{p,\omega}),\quad
\boldsymbol K_i=\mathrm{diag}(K_{i,v},K_{i,\omega}),\quad
\boldsymbol K_d=\mathrm{diag}(K_{d,v},K_{d,\omega})
$$

である。積分状態 $\boldsymbol \xi=\int\boldsymbol e\,dt$、フィルタ済み実測速度微分 $\boldsymbol \eta$ を用いると、

$$
\boxed{
\boldsymbol u_{\mathrm{raw}}
=\boldsymbol B^{-1}\left[
\bar b_0\left(\boldsymbol K_p\boldsymbol e
+\boldsymbol K_i\boldsymbol\xi-\boldsymbol K_d\boldsymbol\eta\right)
+\boldsymbol a_{\mathrm{ff}}
\right]
}
$$

となる。ここで $\bar b_0=(b_{0L}+b_{0R})/2$ であり、以前の車輪速度PIDのゲインを初期値として流用しやすくするために導入している。左右の $b_0$ が等しく、両軸のゲインが等しければ、公称・非飽和条件下で左右独立PIDに対応する。

**微分は目標値ではなく実測機体速度に対して行う**ため、目標速度のステップ変更による微分キックを抑えている。実装する1次ローパスフィルタは

$$
\boldsymbol\eta_k=
(1-\alpha)\boldsymbol\eta_{k-1}
+\alpha\,\frac{\boldsymbol y_k-\boldsymbol y_{k-1}}{\Delta t},
\qquad \alpha=\frac{\Delta t}{\tau_d+\Delta t}.
$$

積分更新は、電流候補の左右両方が上限内であり、通常停止の減速ランプ中ではない場合に限る（保守的な条件付き積分によるアンチワインドアップ）。その後の電流変化率制限まで考慮した完全なアンチワインドアップではない。

## 5. 共通の起動・走行・停止処理

- `diff_drive_controller` が `/cmd_vel` を受け取り、`ros2_control` に左右車輪角速度の目標値を渡す従来のインターフェースを維持する。
- 静止摩擦への対処として、左右独立のBREAKAWAY、TRACK移行、TRACK_HOLD、ストール検出・ロックアウトを維持する。
- **両輪がTRACK状態、かつ左右の内部目標角速度がともに非ゼロ**のときMIMO計算結果を使用する。それ以外は既存の車輪単位の制御処理を使用する。したがって全時間帯で常にMIMO制御しているわけではない。
- 従来モードに備わる左右車輪速度の直進同期補正は、MIMOモードでは無効にしている（二重補正防止）。
- 通常停止の減速度フィードフォワードは、既存の車輪角減速度を機体加速度へ座標変換してから計算に加える。
- 既存のCANopen指令の符号規約を維持する。送信時は左輪電流の符号を反転する。
- 実行中のモード切替には対応しない。停止した後にモードを指定して起動し直す。

## 6. 初期パラメータ

これらは既存リポジトリに由来する**比較用初期値**であり、MIMO制御として実機で最適化・検証済みの値ではない。

| パラメータ | 初期値 | 役割 |
| --- | ---: | --- |
| `ladrc_b0_left`, `ladrc_b0_right` | 18.0, 18.0 | 車輪の公称入力ゲイン |
| `mimo_wheel_radius` | 0.064 m | 車輪半径 |
| `mimo_wheel_separation` | 0.354 m | 有効車輪間距離 |
| `mimo_wc_v`, `mimo_wc_omega` | 5.0, 5.0 rad/s | MIMO-LADRC制御帯域幅 |
| `mimo_wo_v`, `mimo_wo_omega` | 20.0, 20.0 rad/s | MIMO-ESO帯域幅 |
| `mimo_pid_kp_v`, `mimo_pid_kp_omega` | 0.7, 0.7 | MIMO-PID比例ゲイン |
| `mimo_pid_ki_v`, `mimo_pid_ki_omega` | 0.5, 0.5 | MIMO-PID積分ゲイン |
| `mimo_pid_kd_v`, `mimo_pid_kd_omega` | 0.03, 0.03 | MIMO-PID微分ゲイン |
| `pid_derivative_filter_tau_s` | 0.05 s | 微分フィルタ時定数 |
| `current_limit_a` | 8.0 A | 既存URDFの電流上限（要実機確認） |
| `current_rate_limit_a_per_s` | 20.0 A/s | TRACK電流の変化率上限 |

まず左右独立制御と同じ条件を基準に比較する。旋回方向のゲインだけを増加させる場合も、実機の応答・推定ノイズ・電流飽和・無駄時間に注意すること。

## 7. ビルド・実行方法

以下はUbuntu 22.04 / ROS 2 Humbleを想定する。**制御本体とbringupの両リポジトリで同じブランチを使用する。**

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch feat/mimo-ladrc-pid-straight-tracking || git switch --track origin/feat/mimo-ladrc-pid-straight-tracking

cd ~/ros2_ws/src/zlac8015d_can_bringup
git fetch origin
git switch feat/mimo-ladrc-pid-straight-tracking || git switch --track origin/feat/mimo-ladrc-pid-straight-tracking

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

初回は車輪を浮かせ、非常停止手段を確保してから実行する。

**MIMO-LADRCモード：**

```bash
ZLAC_CONTROL_MODE=mimo_ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

**MIMO-PIDモード：**（別の試験として完全停止後に再起動）

```bash
ZLAC_CONTROL_MODE=mimo_pid_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

従来方式との比較モードは `ladrc_torque` と `pid_torque`。環境変数 `ZLAC_CONTROL_MODE` を指定しない場合、URDF設定に従う。モードを変更するには、`ros2_control_node` を起動するプロセスに環境変数を渡す必要がある。

**ROS不要の行列・制御式テスト：**

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude tests/test_mimo_control.cpp -o /tmp/test_mimo_control
/tmp/test_mimo_control
```

## 8. CSVログ・比較実験

| 制御方式 | CSVファイル |
| --- | --- |
| MIMO-LADRC | `/tmp/mimo_ladrc_track_200hz.csv` |
| MIMO-PID | `/tmp/mimo_pid_track_200hz.csv` |

両方式とも、既存の左右車輪目標角速度・実測角速度・電流・TRACK状態などの列に加え、次の項目を記録する。

- `ref_v_m_s`, `ref_omega_rad_s`：外部から受けた目標車輪速度から算出した機体目標速度
- `v_m_s`, `omega_rad_s`：左右エンコーダから算出した機体速度
- `hat_v_m_s`, `hat_omega_rad_s`：MIMO-LADRCではESO速度推定値
- `hat_d_v_m_s2`, `hat_d_omega_rad_s2`：MIMO-LADRCではESO総外乱推定値
- `mimo_pid_integral_v`, `mimo_pid_integral_omega`：MIMO-PIDの積分状態

**注意：** CSVの `ref_*` は受信した目標車輪速度から計算しており、内部の停止減速ランプを反映した目標値ではない。MIMO-PIDにおける `hat_*` は実測値を格納した診断列であり、ESO推定結果ではない。

同一の速度指令・制限条件で、従来LADRC、従来PID、MIMO-LADRC、MIMO-PIDの4方式を比較する。評価指標には、並進速度RMSE、角速度RMSE、左右車輪速度差、オーバーシュート、電流RMSを用いる。**エンコーダ値だけでは実際の方位逸脱を評価できない**ため、IMUのヨーレート・方位角、可能であれば横偏差も別途記録する。

## 9. 実機への適用条件・未検証事項

1. 初回は車輪を浮かせ、回転方向と電流指令の符号を確認する。左旋回方向の角加速度指令は、一般に左輪電流を減らし右輪電流を増やす。
2. **ROS 2 Humble環境でのビルド、200 Hzでの実行、CAN通信、実機走行は未検証。** 行列テストが成功しても実機の閉ループ安定性を保証しない。
3. 現在の同定値には約23 msの無駄時間がある。ESOは遅延を明示的にモデル化していないため、外乱推定誤差やオーバーシュートの可能性がある。
4. 電流上限値は必ずモータ・ドライバの仕様と照合する。ソフトウェアによるゼロ電流送信はCAN断線時の物理的な非常停止を保証しない。
5. 現状の角速度フィードバックはエンコーダ由来であり、**IMUフィードバック、姿勢角制御、横偏差制御、スリップ補正は未実装**である。
