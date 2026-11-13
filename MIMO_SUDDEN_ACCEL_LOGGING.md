# MIMO-LADRC 急加速再現試験のデータ・ログ収集

既存の制御則・起動ファイルは変更しない。別端末の操作コントローラから /cmd_vel を送る。
スクリプトは記録と既存launchの起動のみ行い、速度・電流指令は発行しない。

## 記録データ

- 内部200Hz CSV：mimo_ladrc_track_200hz.csv。ハードウェアが生成する /tmp/mimo_ladrc_track_200hz.csv のコピー。
- ROS CSV：cmd_vel.csv, joint_states.csv, odom.csv。既存の受信専用 record_experiment.py が収集。
- ROS bag：/cmd_vel, /joint_states, /odom, /tf, /tf_static の生メッセージ。
- ros_launch.log：launchおよびdriverのコンソールログ。
- passive_recorder.log, rosbag.log, ros_logs/, capture_metadata.json。

内部CSVの既存列：
time_s、ref_left_rad_s、ref_right_rad_s、vel_left_rad_s、vel_right_rad_s、
cmd_current_left_A、cmd_current_right_A、actual_current_left_A、actual_current_right_A、
ref_v_m_s、ref_omega_rad_s、v_m_s、omega_rad_s、hat_v_m_s、hat_omega_rad_s、
hat_d_v_m_s2、hat_d_omega_rad_s2、tracking_left、tracking_right、
breakaway_target_left_A、breakaway_target_right_A、stall_lockout_left、stall_lockout_right。

内部CSVの time_s と ROS側の time_s は記録開始起点が異なる。
厳密な時間比較には運動開始・指令変化などのイベントを用いた時刻合わせが必要。
ROS側の received_utc には受信時刻も格納される。

## 端末A：最初の更新・無通電単体テスト

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking

python3 -m unittest discover -s tests -p 'test_record_mimo_accel_session.py' -v
python3 scripts/record_mimo_accel_session.py --check-only
~~~

CHECK ONLY はモータを動かさない。rosbag2がない環境では --no-rosbag を指定する。
すでに別の ros2_control_node・launch・CAN watchdog が稼働している場合は競合を検出して起動しない。

## 端末A：収集・既存launch同時起動

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup

python3 scripts/record_mimo_accel_session.py \
  --label sudden_acceleration \
  --trial 01
~~~

内部で次のコマンドを実行する：

~~~bash
ZLAC_CONTROL_MODE=mimo_ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
~~~

既存のlaunchを別に二重起動しない。

## 端末B：コントローラの動作確認

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 topic info /cmd_vel
ros2 topic echo --once /cmd_vel
~~~

操作コントローラから /cmd_vel を送信して走行させる。
トピック名は /cmd_vel であり、cmd/vel ではない。
既存のlaunchは /diff_drive_controller/cmd_vel_unstamped を /cmd_vel にリマップする。
このログ収集器は /cmd_vel の発行は行わない。

試験終了時：操作コントローラからゼロ速度を送信、機体停止を確認。
その後端末Aで Ctrl+C を押してlaunchを終了する。
launch・内部ロガーを終了させてから /tmp の内部CSVを各実験フォルダへコピーする。
もし内部CSVが生成されなかった場合や前回の古いログの場合、コピーせず原因をメタデータへ記録する。

## データフォルダ

結果は実験ごとに以下へ生成する。

~~~text
~/ros2_ws/mimo_accel_results/
  YYYYMMDDTHHMMSSZ_sudden_acceleration_trial01_XXXXXXXX/
    capture_metadata.json
    mimo_ladrc_track_200hz.csv
    ros_launch.log
    passive_recorder.log
    rosbag.log
    rosbag2/
    ros_csv/
      YYYYMMDDTHHMMSSZ_mimo_ladrc_torque_sudden_acceleration_trial01_XXXXXXXX/
        cmd_vel.csv
        joint_states.csv
        odom.csv
        experiment.json
    ros_logs/
~~~

rosbag2がインストールされていない場合は --no-rosbag を付けて起動する。
この場合rosbag.logとrosbag2/は作成されない。

保存確認：

~~~bash
LATEST=$(find ~/ros2_ws/mimo_accel_results -mindepth 1 -maxdepth 1 \
  -type d -name '*sudden_acceleration*' | sort | tail -n 1)
echo "$LATEST"
cat "$LATEST/capture_metadata.json"
head -n 3 "$LATEST/mimo_ladrc_track_200hz.csv"
find "$LATEST/ros_csv" -type f
~~~

目標・実速度、内部電流指令、6077h電流、ESO推定値、TRACK状態を比較し、
急加速時の異常が指令・ESO・アクチュエータのどこで発生したかを検討する。
