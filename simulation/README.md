# MIRS シミュレーション

Gazebo Harmonic 上で、固定 LiDAR の 2D 地図作成とナビゲーション、
首振り LiDAR の 3D 地図作成とナビゲーションを行います。

## 起動

Jazzy コンテナ内で `package.xml` の依存を rosdep でインストールしてから、
ワークスペースをビルドし、環境を読み込みます。

```bash
cd /home/developer/ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to simulation
source install/setup.bash
```

`launch/` には 7 ファイルあります。`simulation_launch.py` が共通 API で、
残り 6 ファイルは用途ごとの入口です。

| 起動コマンド | センサーと動作 |
|---|---|
| `ros2 launch simulation slam_2d.launch.py` | 固定 LiDAR の `/scan` で 2D 地図を作る |
| `ros2 launch simulation nav_2d.launch.py` | 2D 地図で自己位置推定し、Nav2 を起動する |
| `ros2 launch simulation slam_3d.launch.py` | 首振り LiDAR の高さ選別済み点群で 3D 地図を作る |
| `ros2 launch simulation nav_3d.launch.py` | 3D 地図で自己位置推定し、Nav2 を起動する |
| `ros2 launch simulation coverage_2d.launch.py` | 保存済み 2D 地図で Nav2 とカバレッジを起動する |
| `ros2 launch simulation coverage_3d.launch.py` | 保存済み 3D 地図で Nav2 とカバレッジを起動する |

2D launchは固定LiDAR向けの`park.sdf`、3D launchは首振りLiDAR向けの
`nodding_forest.sdf`を既定worldにします。`world:=...`で上書きできます。

共通 API を直接起動する場合は、必要な機能をlaunch引数で有効にします。

```bash
ros2 launch simulation simulation_launch.py enable_slam:=true enable_nodding_lidar:=true
ros2 launch simulation simulation_launch.py enable_nav2:=true enable_nodding_lidar:=false
```

Gazebo は ROS Domain ID `91` で起動します。実機ファームウェアの Domain ID
`90` とDDS通信を分離します。3DモードではGazeboの生TFを `/tf_gazebo` に
退避し、EKFが `odom → base_footprint` を配信します。
共通APIはFoxglove Bridgeも起動します。ホスト側のFoxgloveから
`ws://localhost:8765` に接続すると、同じDomainの地図、診断、経路を表示できます。
Bridgeを別端末で重複起動する必要はありません。
Gazeboの外から `ros2 topic` などで確認するときは、別ターミナルで次を実行します。

```bash
export ROS_DOMAIN_ID=91
source /home/developer/ws/install/setup.bash
ros2 topic echo /tf --once
```

シミュレーションと実機は、それぞれ `simulation` と
`nodding_lidar` / `mirs` の起動コマンドで起動してください。
Gazeboを動かす際に実機用 `slam_launch.py` / `bringup_launch.py` も同時起動しないでください。

ナビゲーションとカバレッジを起動する前に、同じ次元の SLAM モードで地図を
保存してください。既定のGazeboデータベースは `~/.ros/mirs_gazebo_2d.db` と
`~/.ros/mirs_gazebo_3d.db` です。実機用データベースとは別です。
3D ナビゲーションでは RTAB-Map の 3D 地図で
自己位置を推定し、Nav2 は RTAB-Map が出す 2D 占有格子 `/map` で経路を
計画します。カバレッジも同じ自己位置推定と Nav2 を使用します。
スマホアプリのBLE JSONは、実機と同じ `ble_receiver_node` が受信して
`/cleaning_zone` と `/initialpose` に変換します。`/cleaning_zone` は
`coverage_bridge` がカバレッジのアクションへ渡します。

### 保存済み3D地図でカバレッジを試す

`coverage_3d.launch.py` は `nav_3d` と同じ RTAB-Map DB による自己位置推定、
Nav2、OpenNav Coverage の `coverage_server`、BLE受信、
`coverage_bridge` をまとめて
起動します。2D/3Dのcoverage入口launchがcoverageパッケージの
`coverage.launch.py`をincludeします。実機ハードウェアとRVizの重複起動は
無効にし、Nav2とcoverage serverには同じシミュレーション用パラメータを渡します。
BLE receiverは`ble_server`のlaunch API経由でcoverage側が起動します。
`/cleaning_zone`を受け取った`coverage_bridge`がNav2の
`navigate_complete_coverage`アクションへ送信し、Fields2Coverによる経路生成と
Nav2の経路追従につなぎます。
SLAMを終了してDBへの書き込みを終えてから、保存したDBを指定してください。
スマホからは実機と同じ `initial_pose` と `cleaning_zone` のJSONを送信します。
BLEを使わずトピック送信で試すときは `enable_ble:=false` を付けます。
この3Dカバレッジ入口は既定で `/home/developer/ws/maps/forest_3d.db` を読み込みます。
RTAB-Mapの初期位置は既定で地図座標の原点 `(0, 0, 0)`、姿勢 `(0, 0, 0)`
として渡します。保存DBの最後の位置から再開したい場合は
`rtabmap_initial_pose:=''` を指定します。

BLE受信にはホストで稼働するBlueZと、コンテナからのシステムD-Busアクセスが
必要です。`ws/container/jazzy/docker-compose.yml` は `/run/dbus` を読み取り専用で
共有し、Dockerfileに `python3-dbus-fast` を追加しています。変更後は
コンテナを再ビルドしてください。

```bash
ros2 launch simulation coverage_3d.launch.py
```

別の地図を使う場合は `database_path:=/path/to/map.db` で上書きできます。

すでに `nav_3d.launch.py` を起動している場合は、GazeboとNav2を重複起動せず、
別ターミナルでカバレッジ部分だけを起動できます。別ターミナルにも
`ROS_DOMAIN_ID=91` を設定します。

```bash
export ROS_DOMAIN_ID=91
source /home/developer/ws/install/setup.bash
ros2 launch coverage coverage.launch.py use_sim_time:=true \
  enable_mirs:=false enable_ble:=true use_rviz:=false \
  nav2_params_file:=/home/developer/ws/install/simulation/share/simulation/config/nodding_nav2.yaml
```

カバレッジのアクションと計画結果は、同じROS Domainの別ターミナルで確認できます。

```bash
ros2 action list | rg 'navigate_complete_coverage|compute_coverage_path'
ros2 topic echo /coverage/state
ros2 topic echo /coverage_server/coverage_plan --once
```

カバレッジ起動時は `coverage_monitor` が `/diagnostics` にセンサーHz、
最終受信からの実時間、TF、速度指令経路とシミュレーション速度を1秒ごとに出します。
`coverage_bridge` は `/coverage/phase` にゴールの処理段階を、
`/diagnostics` にゾーン占有率とNav2の受理・失敗理由を出します。
3D起動では `/scan_raw`、`/points`、`/points_icp_filtered`、
`/points_assembled_icp` も監視します。各センサー項目にはヘッダー時刻の重複・逆行回数、
点群の点数、スキャンの `time_increment` と `scan_time` を表示します。
`NON_MONOTONIC_STAMP` が最初に現れるトピックで時刻異常の発生段階を特定できます。
Foxgloveでは `/diagnostics` を Diagnostics – Summary (ROS) に指定できます。
RTAB-Map内部の警告も `/rosout` に転送します。

### 3D点群の段階別診断

Nav2・coverage・地面判別を起動せず、LiDAR経路を一段ずつ追加する場合は
`diagnose_3d.launch.py` を使います。各段階は同時起動せず、前のlaunchを終了してから
次を起動します。

| `stage` | 追加される処理 | 確認する出力 |
|---|---|---|
| `scan` | Gazebo、機体、bridge、首振り状態 | `/scan_raw`、`/scan` |
| `cloud` | EKF、スキャンの点群化 | `/odometry/filtered`、`/points` |
| `filtered` | RTAB-Map用の高さフィルタ | `/points_icp_filtered` |
| `assembled` | 2秒分の点群統合 | `/points_assembled_icp` |
| `localization` | 保存済み地図に対するRTAB-Map自己位置推定 | `/localization_pose` |

```bash
colcon build --packages-select simulation coverage
source install/setup.bash
ros2 launch simulation diagnose_3d.launch.py stage:=scan
```

`stage:=cloud`、`stage:=filtered`、`stage:=assembled`と順に変え、
`/diagnostics` の `coverage/topic/*` にある `repeated_stamps`、
`backward_stamps`、`last_point_count` を比較します。最初に異常が増えた段階が
調査対象です。`publisher_count` が2以上なら、同じノードが残っていないか
`ros2 topic info /scan --verbose` で発行元を確認します。`stage:=localization` では
`database_path:=/home/developer/ws/maps/forest_3d.db` を既定で使用します。
Foxgloveを使う場合は `enable_foxglove_bridge:=true` を指定します。
別端末から確認する場合は `ROS_DOMAIN_ID=91 ros2 topic echo /diagnostics` を実行します。
走行中の統合結果は `stage:=assembled` の状態で別端末から次を実行し、
`/points_assembled_icp` の時刻と点数が継続して更新されるか確認できます。

```bash
ROS_DOMAIN_ID=91 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

`/cleaning_zone` は地図の `map` 座標系で閉じていない多角形を送ります。
`coverage_bridge` が終点を始点に接続します。例えば障害物のない2 m四方を
選び、次のように送信します。

```bash
ros2 topic pub --once /cleaning_zone geometry_msgs/msg/PolygonStamped \
  '{header: {frame_id: map}, polygon: {points: [{x: 0.5, y: 0.5}, {x: 2.5, y: 0.5}, {x: 2.5, y: 2.5}, {x: 0.5, y: 2.5}]}}'
```

この例の座標は地図上の空き領域に合わせて変更してください。

SLAM ではキーボード操作用の xterm を別ウィンドウで開き、
`teleop_twist_keyboard` が `/cmd_vel` を出します。xterm が起動できない場合は
launch 端末に表示されるエラーを確認してください。
コンテナへは `docker compose exec -it ros bash` のように対話端末で入ります。
共通APIの`simulation_launch.py`はSLAM/Nav2を既定では起動しません。
Coverageは専用の入口launchがcoverageパッケージをincludeします。
全モードで RTAB-Map Viz と RViz が開きます。
画面が不要なら `enable_rtabmap_viz:=false enable_rviz:=false`、
キーボード操作が不要なら `enable_teleop:=false` を指定します。

xterm のウィンドウが出ない場合は、Jazzy のコンテナを再ビルドしてから確認します。

```bash
cd ws/container/jazzy
docker compose up -d --build
docker compose exec -it ros bash
command -v xterm
printf 'DISPLAY=%s XAUTHORITY=%s\n' "$DISPLAY" "$XAUTHORITY"
xterm -T test -e bash -lc 'echo opened; read'
```

最後のコマンドで窓が開かない場合は、`DISPLAY`、X11 ソケット、
X11 認証を確認してください。`xterm` が見つからない場合は、再ビルドした
コンテナでコマンドを実行しているか確認してください。

## 坂と段差のある world

`worlds/terrain_course.sdf` には、上り坂・高さ 0.4 m の平坦部・下り坂と、
1 段あたり 3 cm の階段状コースがあります。開始位置 `(0, 0)` の前方で、
坂は `y=3`、段差は `y=-3` にあります。

```bash
ros2 launch simulation slam_3d.launch.py \
  world:=/home/developer/ws/src/original/simulation/simulation/worlds/terrain_course.sdf
```

坂は 4 m で 0.4 m 上がる約 5.7 度です。角度を変える際は、坂の箱の
`<pose>` にある pitch と `<size>` の長さを、平坦部の高さに合わせて変更します。
段差は各 `<link>` の `<pose>` の z を箱の高さの半分にし、箱の `<size>` の
高さを変えます。3D 地面分類の既定の坂判定は 12 度なので、それを超える
坂を作る場合は `config/nodding_ground.yaml` の `slopeThresholdDegrees` も
確認してください。車体は 2 輪差動と前後の固定支持球による近似モデルなので、登坂や段差越えの
実機性能をこの world だけで判断しないでください。

## 走行物理

車体は 2 個の駆動輪と前後の固定支持球で支えます。削除した車輪の質量は
車体へまとめ、全体質量をおよそ維持しています。Gazebo の差動駆動は直進加減速度を
±0.5 m/s²、旋回加減速度を ±1.0 rad/s² に制限し、急な停止指令で
車輪が瞬時に止まらないようにします。Nav2 の MPPI と速度平滑化も
同じ加減速上限に合わせています。

`wheel_radius` と `wheel_track` は差動モデルの半径と左右駆動輪間隔です。
`wheel_mu_longitudinal` と `wheel_mu_lateral` で前後・横方向の接地摩擦を、
`caster_mu` で支持球の摩擦を調整できます。横滑りを抑える場合は横方向摩擦を
上げ、旋回しにくい場合は下げます。これらはシミュレーション用の近似値です。

## 地図データベース

RTAB-Map の DB はセンサー観測と地図を保存する SQLite ファイルです。
SLAM 起動時は指定された DB を初期化して新しい地図を作ります。終了後は DB が
残り、Nav と coverage はその DB を読み込んで自己位置を推定します。つまり、
前回の地図を読み込んで続きから作るのはSLAMではなくNav/coverageです。
SLAM時に既存DBは上書きされるため、残しておきたい地図がある場合は別の
`database_path` を指定してください。

```bash
ros2 launch simulation slam_3d.launch.py database_path:=/home/developer/.ros/field_a_3d.db
ros2 launch simulation nav_3d.launch.py database_path:=/home/developer/.ros/field_a_3d.db
```

同じDBでSLAMを再起動すると、そのDBから地図を読み足さずに新しい地図へ置き換えます。
過去の地図を残すには、起動ごとに異なる `database_path` を指定してください。

## API 引数

| 引数 | 既定値 | 内容 |
|---|---|---|
| `enable_slam` | `false` | RTAB-Mapで新しい地図を作る |
| `enable_nav2` | `false` | 保存済みRTAB-Map地図で自己位置推定しNav2を起動 |
| `enable_nodding_lidar` | `false` | 首振りLiDARを使う3D構成。falseは固定LiDARの2D構成 |
| `world` | 2D: `worlds/park.sdf`、3D: `worlds/nodding_forest.sdf` | Gazebo ワールド |
| `robot_model_file` | `urdf/mirs_gazebo.urdf.xacro` | 機体モデル。2D では固定 LiDAR、3D では首振り |
| `spawn_x`, `spawn_y`, `spawn_z` | `0`, `0`, `0.0` | 機体の初期位置（base_footprintを平地の地面に置く） |
| `wheel_radius`, `wheel_track` | `0.0665`, `0.465` m | 2輪差動の駆動輪半径・左右輪間隔。Gazebo DiffDriveとURDFで共通使用 |
| `wheel_mu_longitudinal`, `wheel_mu_lateral` | `1.0`, `0.2` | 駆動輪の前後・横方向の摩擦係数。横滑りは横方向係数で調整 |
| `caster_mu` | `0.001` | 前後支持球の摩擦係数 |
| `database_path` | モード別の上記パス | RTAB-Map データベース |
| `bridge_config_file` | `config/nodding_bridge.yaml` | Gazebo と ROS のトピック接続 |
| `cloud_config_file` | `config/nodding_cloud.yaml` | 3D 点群生成 |
| `cloud_assembling_time` | `2.0` 秒 | RTAB-Map に渡す 3D 点群の蓄積時間 |
| `ground_config_file` | `config/nodding_ground.yaml` | 3D 地面分類 |
| `ekf_config_file` | `config/nodding_ekf.yaml` | 3D オドメトリ融合 |
| `nav2_params_file` | 2D: `config/nav2_params.yaml`、3D: `config/nodding_nav2.yaml` | Nav2 設定 |
| `rviz_config_file` | 2D: `rviz/mapping.rviz`、3D: `rviz/nodding.rviz` | RViz 設定 |
| `enable_gazebo`, `enable_robot`, `enable_bridge` | `true` | シミュレーション基盤 |
| `enable_foxglove_bridge` | `true` | Foxglove WebSocket Bridgeを起動 |
| `foxglove_address`, `foxglove_port` | `0.0.0.0`, `8765` | Foxglove接続の待受先 |
| `enable_ble` | coverage入口: `true` | Coverage入力用BLE受信ノード |
| `enable_ground_segmentation` | `false` | 3D 点群の地面分類 |
| `icp_min_height` | `0.5` m | ICP と RTAB-Map に渡す点の、地面からの最小高さ |
| `wheel_linear_velocity_variance` | `0.04` | 車輪速度の融合共分散 [m²/s²] |
| `wheel_angular_velocity_variance` | `0.09` | 車輪角速度の融合共分散 [(rad/s)²] |
| `imu_yaw_rate_variance` | `0.01` | IMU旋回角速度の融合共分散 [(rad/s)²] |
| `enable_teleop` | `false` | SLAM 時にキーボード操作を起動 |
| `enable_rtabmap_viz` | `false` | RTAB-Map Viz を起動 |
| `enable_rviz` | `false` | RViz |

上位 launch からも共通 API を include できます。

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('simulation'), 'launch', 'simulation_launch.py'])),
    launch_arguments={
        'enable_slam': 'true',
        'enable_nodding_lidar': 'true',
        'enable_rviz': 'false',
    }.items())
```

2D モードでは Gazebo の `/scan_raw` を `/scan` に接続します。
3D モードでは `nodding_io` が `/scan_raw` と首振り関節の実測値から
`/scan` と `/servo_state` を出し、`scan_to_cloud` が `/points` を作ります。
`/scan` と各 `/points` メッセージは首振り角ひとつ分の平面です。
地面から `icp_min_height`（既定値 0.5 m）以上の点を2秒分蓄積した
`/points_assembled_icp` を、ICP オドメトリと RTAB-Map の地図照合に渡します。
草や床の点は照合に使いません。RTAB-Map の `/map` は高さを満たす点から作るため、
50 cm 未満の障害物は地図に残りません。Nav2 の現在の障害物検知には別経路の
地面分類点群を使います。地図の空き領域は障害物点までのレイ追跡で補います。
3D レイ追跡には RTAB-Map の OctoMap 対応ビルドが必要です。
地面分類では生の `/points` を2秒間の円形バッファに蓄積して
`/points_assembled` を作り、首振り1時刻分だけでなく1往復分の
3D 形状から床と障害物を判定します。起動直後の約2秒間は蓄積が未完成です。
RVizでは地面分類の緑点と障害物の赤点、RTAB-Mapの`/map`を初期表示します。
生点群の表示は重なりを避けるため初期状態で無効です。床に赤点があれば
地面分類側、黒い`/map`セルがあれば地図側を調べてください。
Navモードは既存DBの地図を読み込むため、過去の床セルを消すには新しい
`database_path`でSLAMし直してからNavを起動します。
平らな地面を測ると、傾いた走査面と地面の交線に沿って測定点が直線状に並びます。
複数の点群を重ねて表示すると、角度の異なる交線が平行線として見えます。
両モードとも RTAB-Map が `map → odom` と `/map` を出します。2Dモードでは
Gazebo DiffDriveが `odom → base_footprint` を出し、3DモードではEKFが車輪速度と
IMU旋回角速度から同じTFを出します。ICPの自己位置をEKFに戻すと点群蓄積に
循環するため、RTAB-Mapの点群照合はEKFに融合せず、地図照合による補正を
`map → odom` に分けます。3Dの点群蓄積とRTAB-Mapは
`/odometry/filtered` を使います。EKFへ入れるGazeboの生オドメトリと
IMUには、シミュレーション向けの共分散を付けています。RTAB-Mapは機体が
5 cm または約 3 度以上動いたときに新しい地図ノードを追加します。

IMU と磁気センサーは 2D・3D の両モデルで `imu_link` に取り付けます。
Gazebo の IMU・Magnetometer system と ROS ブリッジを通じて、
`/imu/data_raw` は `sensor_msgs/msg/Imu`、`/imu/mag` は
`sensor_msgs/msg/MagneticField` を出します。磁気センサーは実機の
BMX055 に合わせて約 17 Hz にしています。各 world の磁場は固定値で、
実機の周辺環境や磁気外乱は再現しません。3D の地面分類は IMU の姿勢を
使用します。現在の RTAB-Map と Nav2 は `/imu/mag` を購読しません。

```bash
ros2 topic echo /imu/mag --once
ros2 topic hz /imu/mag
```

シミュレーションの 2D LiDAR は RPLIDAR S1 の DenseBoost モードの代表値に
合わせ、360°・920 点/周（9,200 点/秒）・10 Hz、最短 0.1 m、
最長 40 m、距離分解能 0.03 m にしています。3D モードでも同じ LiDAR を
首振り台に載せます。40 m は白い対象の公称最大距離です。実機では黒い対象の
公称最大距離は 10 m ですが、Gazebo のこのセンサー設定は反射率による
測距距離や ±5 cm の測距誤差を再現しません。
Nav2 の局所障害物判定は別設定で 5 m、空間のクリアは 6 m です。
地面分類は機体周囲の X/Y 各 ±10 m を処理します。これらは LiDAR の
測距上限ではありません。

仕様値の出典: SLAMTEC, [RPLIDAR S1 Introduction and Datasheet, rev. 1.7](https://wiki.slamtec.com/download/attachments/83066883/LD601_SLAMTEC_rplidar_datasheet_S1_v1.7_en.pdf?api=v2&modificationDate=1689040187000&version=2)。
ここでは数値を参照し、文書・図面・画像は転載していません。

Gazebo の GPU LiDAR は 10 Hz の各スキャンを一瞬で取得します。
実機 S1 の回転中の移動歪み、クローラの滑りや草による段差は再現しません。
