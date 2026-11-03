# MIRS シミュレーション

Gazebo Harmonic 上で、固定 LiDAR の 2D 地図作成とナビゲーション、
首振り2D LiDAR の 3D 地図作成とナビゲーションを行います。

## 起動

Jazzy コンテナ内で `package.xml` の依存を rosdep でインストールしてから、
ワークスペースをビルドし、環境を読み込みます。

```bash
cd /home/developer/ws
mkdir -p deps
touch deps/COLCON_IGNORE
vcs import deps < src/original/simulation/dependencies.repos
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to simulation
source install/setup.bash
```

`simulation_launch.py`が共通APIで、ほかのlaunchは用途ごとの入口です。

| 起動コマンド | センサーと動作 |
|---|---|
| `ros2 launch simulation slam_2d.launch.py` | 固定 LiDAR の `/scan` で 2D 地図を作る |
| `ros2 launch simulation nav_2d.launch.py` | 2D 地図で自己位置推定し、Nav2 を起動する |
| `ros2 launch simulation slam_3d.launch.py` | 首振り2D LiDAR のGSeg3D非地面点群で 3D 地図を作る |
| `ros2 launch simulation nav_3d.launch.py` | 3D 地図で自己位置推定し、Nav2 を起動する |
| `ros2 launch simulation coverage_2d.launch.py` | 保存済み 2D 地図で Nav2 とカバレッジを起動する |
| `ros2 launch simulation coverage_3d.launch.py` | 保存済み 3D 地図で Nav2 とカバレッジを起動する |

2D launchは固定LiDAR向けの`park.sdf`、3D launchは首振り2D LiDAR向けの
`nodding_forest.sdf`を既定worldにします。`world:=...`で上書きできます。

共通 API を直接起動する場合は、必要な機能をlaunch引数で有効にします。

```bash
ros2 launch simulation simulation_launch.py enable_slam:=true enable_nodding_lidar:=true
ros2 launch simulation simulation_launch.py enable_nav2:=true enable_nodding_lidar:=false
```

Gazebo は ROS Domain ID `91` で起動します。実機ファームウェアの Domain ID
`90` とDDS通信を分離します。3DモードではGazeboの生TFを `/tf_gazebo` に
退避し、EKFが `odom → base_footprint` を配信します。
共通launchは `GZ_IP=127.0.0.1` を設定し、GazeboのGUI・サーバー・ROSブリッジの
通信に同じホスト内のループバックアドレスを使います。別のPCとのGazebo通信は対象外です。
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
保存してください。既定のGazeboデータベースは `~/ws/maps/simulation/mirs_gazebo_2d.db` と
`~/ws/maps/simulation/mirs_gazebo_3d.db` です。実機用データベースとは別です。
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
この3Dカバレッジ入口は既定で `/home/developer/ws/maps/simulation/mirs_gazebo_3d.db` を読み込みます。
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
| `assembled` | 時間・旋回量を制限した点群統合 | `/points_assembled_icp` |
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
`database_path:=/home/developer/ws/maps/simulation/mirs_gazebo_3d.db` を既定で使用します。
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
  world:=/home/developer/ws/src/original/simulation/worlds/terrain_course.sdf
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
| `enable_nodding_lidar` | `false` | 首振り2D LiDARを使う3D構成。falseは固定LiDARの2D構成 |
| `world` | 2D: `worlds/park.sdf`、3D: `worlds/nodding_forest.sdf` | Gazebo ワールド |
| `robot_model_file` | `urdf/mirs_gazebo.urdf.xacro` | 機体モデル。2D では固定 LiDAR、3D では首振り |
| `spawn_x`, `spawn_y`, `spawn_z` | `0`, `0`, `0.0` | 機体の初期位置（base_footprintを平地の地面に置く） |
| `wheel_radius`, `wheel_track` | `0.0665`, `0.465` m | 2輪差動の駆動輪半径・左右輪間隔。Gazebo DiffDriveとURDFで共通使用 |
| `wheel_mu_longitudinal`, `wheel_mu_lateral` | `1.0`, `0.2` | 駆動輪の前後・横方向の摩擦係数。横滑りは横方向係数で調整 |
| `caster_mu` | `0.001` | 前後支持球の摩擦係数 |
| `database_path` | モード別の上記パス | RTAB-Map データベース |
| `bridge_config_file` | `config/nodding_bridge.yaml` | Gazebo と ROS のトピック接続 |
| `cloud_config_file` | `config/nodding_cloud.yaml` | 3D 点群生成 |
| `cloud_assembling_time` | `2.0` 秒 | 地面分離用の3D点群の蓄積時間 |
| `motion_cloud_config` | nodding_lidarの `config/motion_cloud.yaml` | 照合用点群の時間・旋回量・点数制限 |
| `ground_config_file` | `config/nodding_ground.yaml` | 3D 地面分類 |
| `ekf_config_file` | `config/nodding_ekf.yaml` | 3D オドメトリ融合 |
| `nav2_params_file` | 2D: `config/nav2_params.yaml`、3D: `config/nodding_nav2.yaml` | Nav2 設定 |
| `rviz_config_file` | 2D: `rviz/mapping.rviz`、3D: `rviz/nodding.rviz` | RViz 設定 |
| `enable_gazebo`, `enable_robot`, `enable_bridge` | `true` | シミュレーション基盤 |
| `enable_foxglove_bridge` | `true` | Foxglove WebSocket Bridgeを起動 |
| `foxglove_address`, `foxglove_port` | `0.0.0.0`, `8765` | Foxglove接続の待受先 |
| `enable_ble` | coverage入口: `true` | Coverage入力用BLE受信ノード |
| `enable_ground_segmentation` | `false` | 3D 点群の地面分類 |
| `icp_min_height` | `0.5` m | 高さフィルタの段階別診断に使う最小高さ。3D地図作成・走行では使わない |
| `wheel_linear_velocity_variance` | `0.04` | 車輪速度の融合共分散 [m²/s²] |
| `wheel_angular_velocity_variance` | `0.09` | 車輪角速度の融合共分散 [(rad/s)²] |
| `imu_yaw_rate_variance` | `0.01` | IMU旋回角速度の観測分散 [(rad/s)²] |
| `wheel_lateral_velocity_variance` | `0.01` | 車輪の機体横速度の観測分散 [(m/s)²] |
| `wheel_vertical_velocity_variance` | `0.01` | 車輪の機体上下速度の観測分散 [(m/s)²] |
| `imu_orientation_variance` | `0.01` | IMU姿勢の各軸の観測分散 [rad²] |
| `imu_angular_velocity_variance` | `0.01` | IMUのX・Y角速度の観測分散 [(rad/s)²] |
| `imu_linear_acceleration_variance` | `0.25` | IMUの各軸の加速度の観測分散 [(m/s²)²] |
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
3Dの地図作成・自己位置推定では、GSeg3Dの
`/ground_segmentation/obstacle_points`をRTAB-Mapの`scan_cloud`へ直接渡します。
GSeg3Dが地面と分類した点は照合に使いません。GSeg3Dで既に蓄積した点群なので、
高さフィルタと照合用の追加蓄積は起動しません。地面分類はこれらのモードでは必ず起動します。
局所移動は車輪とIMUのEKFが担当し、LiDARの照合結果はEKFへ戻しません。
Nav2も同じ非地面点群を現在の障害物検知に使います。
GSeg3Dが誤分類した点はRTAB-Mapにも渡り、分類出力が止まると照合入力も止まります。
非地面点群には空間内の障害物も含まれるため、物体の静止性を保証する処理ではありません。
地図の空き領域は障害物点までのレイ追跡で補います。
高さフィルタを使った地図とは入力条件が異なるため、比較時は別のDBパスで地図を作成します。
3D レイ追跡には RTAB-Map の OctoMap 対応ビルドが必要です。
地面分類では生の `/points` を2秒間の円形バッファに蓄積して
`/points_assembled` を作り、首振り1時刻分だけでなく1往復分の
3D 形状から床と障害物を判定します。起動直後の約2秒間は蓄積が未完成です。
`nodding_ground.yaml` の `downsample_resolution` は `downsample: false` でも必要です。
GSeg3Dは点群受信時にこの値を読み取るため、省略するとパラメータ例外で終了し、
`/ground_segmentation/obstacle_points` が配信されなくなります。
RVizでは地面分類の緑点と障害物の赤点、RTAB-Mapの`/map`を初期表示します。
生点群の表示は重なりを避けるため初期状態で無効です。床に赤点があれば
地面分類側、黒い`/map`セルがあれば地図側を調べてください。
Navモードは既存DBの地図を読み込むため、過去の床セルを消すには新しい
`database_path`でSLAMし直してからNavを起動します。
平らな地面を測ると、傾いた走査面と地面の交線に沿って測定点が直線状に並びます。
複数の点群を重ねて表示すると、角度の異なる交線が平行線として見えます。
両モードとも RTAB-Map が `map → odom` と `/map` を出します。2Dモードでは
Gazebo DiffDriveが `odom → base_footprint` を出し、3DモードではEKFが車輪速度と
IMUの姿勢・角速度・重力除去後の加速度から同じTFを出します。ICPの自己位置をEKFに戻すと点群蓄積に
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

### カバレッジ終了後の帰還

`coverage_2d.launch.py` と `coverage_3d.launch.py` は、simulation の
`behavior_trees/navigate_coverage_return_home.xml` を coverage bridge に渡します。
BT は `/simulation/home_pose` を取得してから走行を開始し、カバレッジ終了後に
Nav2 で帰還経路を計画・追従します。帰還成功後に `/coverage/state` が `DONE` になります。

`home_pose_saver` は起動時の odom 姿勢を基準に、移動がないことと map TF の安定を
確認して初期位置を保存します。保存前に移動した場合は帰還位置を配信せず、再起動を
要求します。起動後、保存完了ログが出るまでは手動操作や別のNav2ゴールを送らないでください。
TFの安定確認は地図上の自己位置の正しさを保証するものではないため、使用する地図と
RTAB-Mapの初期姿勢をスポーン位置に合わせてください。
保存値はノードの生存中保持し、再スポーン時には保存ノードも再起動してください。

確認用トピック（ROS_DOMAIN_ID=91）:

```bash
ros2 topic echo /simulation/home_pose --qos-durability transient_local --once
```

独自のNav2パラメータファイルを指定する場合は、`bt_navigator.plugin_lib_names` に
`simulation_get_home_pose_bt_node` を追加してください。変更後はコンテナ内で
`colcon build --symlink-install --packages-select simulation` を実行してください。

## 走行ログの保存

Jazzyコンテナの別端末で、走行開始前にrosbag記録を開始します。
全トピックと隠しトピックを対象にし、TF、点群、オドメトリ、RTAB-Mapの情報、
Nav2のアクション状態、`/rosout`を保存します。起動後に現れるトピックも探索します。
点群や地図も含むため、保存先の空き容量を確認してください。

```bash
source /home/developer/ws/install/setup.bash
export ROS_DOMAIN_ID=91
mkdir -p /home/developer/ws/logs
ros2 bag record --all --include-hidden-topics --use-sim-time \
  --output /home/developer/ws/logs/coverage_3d_$(date +%Y%m%d_%H%M%S)
```

記録端末にトピックの購読開始が表示されてから清掃範囲を送信します。
位置推定が崩れた後も数秒記録し、記録端末でCtrl+Cを押して終了します。
シミュレーションを終了する前に記録を停止してください。
`--use-sim-time`では`/clock`の受信開始までデータは記録されません。

起動時の標準出力・標準エラーも残す場合は、起動端末で次を使います。
DBのパスは使用する地図に合わせます。

```bash
source /home/developer/ws/install/setup.bash
export ROS_DOMAIN_ID=91
mkdir -p /home/developer/ws/logs
set -o pipefail
ros2 launch simulation coverage_3d.launch.py \
  database_path:=/home/developer/ws/maps/simulation/mirs_gazebo_3d.db \
  2>&1 | tee /home/developer/ws/logs/coverage_3d_console_$(date +%Y%m%d_%H%M%S).log
```

終了後は`ros2 bag info <保存ディレクトリ>`で、`/tf`、`/tf_static`、
`/points_assembled_icp`、`/odometry/filtered`、
`/nodding_lidar/motion_cloud/diagnostics`の件数を確認します。
購読できなかったトピックや未配信のトピックは保存されません。
使用した地図DB、設定ファイル、起動引数と、異常が発生したおおよその時刻も
一緒に保存すると比較できます。rosbagだけではパラメータの初期値や地図DB全体、
Gazeboの真の機体姿勢が保存されるとは限りません。

## 高さフィルタを使う段階別診断の点群蓄積

`diagnose_3d.launch.py stage:=assembled`では、`nodding_lidar`の`motion_cloud_assembler`で点群を作ります。
3D地図作成・走行・`stage:=localization`は選択した地面分離の出力を使うため、この蓄積ノードを使いません。
単発点群の計測時刻のTFでodomへ変換し、最大2 s、絶対yaw角速度積分0.2 rad以内の
移動窓を最新スキャンのbase_link座標に戻して `/points_assembled_icp` に出します。
定速0.4 rad/sの旋回なら、保持時間の目安は0.5 sです。出力上限は2 Hzです。
設定値は調整開始用で、精度が検証済みという意味ではありません。

`map → odom`はRTAB-Mapだけが配信します。
出力TFの後処理による平滑化は行いません。地面分離用の `/points_assembled` は別経路で、
`cloud_assembling_time` に指定した首振り点群の時間幅を維持します。

`motion_cloud_config` で共通ノードの設定ファイルを指定できます。
既定は `nodding_lidar/config/motion_cloud.yaml` です。
入力欠落時の単発点群への退避、点数不足・TF欠測時の出力停止、メモリ上限、
設定項目はnodding_lidarのREADMEを参照してください。
診断は `/nodding_lidar/motion_cloud/diagnostics` に配信します。

```bash
ros2 launch simulation diagnose_3d.launch.py stage:=assembled
ros2 topic echo /nodding_lidar/motion_cloud/diagnostics --once
```

実機に移す判断には、実機の点時刻・サーボ遅延の検証と記録データでの比較が必要です。
シミュレーションの同時刻スキャンだけで、実機のスキャン中の運動補償を検証したとは扱いません。

## Linefitによる地面分離

Linefitは方位ごとの距離・高さの断面で地面線を求めます。
外部ライブラリはパッケージ内の`dependencies.repos`のコミットに固定し、`ws/deps/linefit`へ取得します。
`deps/COLCON_IGNORE`は通常の全体ビルドで外部ROSラッパーまで探索しないためのものです。
公開ROSラッパーにはROS 1由来の依存が残るため、simulationのCMakeで本体ソースを静的ライブラリとしてビルドし、
ROS 2の入出力はsimulationの`linefit_ground_filter`が担当します。外部ソースは変更しません。

Jazzyコンテナ内で次を実行します。

```bash
cd /home/developer/ws
mkdir -p deps
touch deps/COLCON_IGNORE
vcs import deps < src/original/simulation/dependencies.repos
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src deps/linefit/linefit_ground_segmentation --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to simulation
source install/setup.bash
```

地図作成は別のDBへ保存し、同じLinefit入力でカバレッジ走行を開始します。
起動コマンドは別々に実行し、SLAMを終了してから走行を起動します。

```bash
ros2 launch simulation slam_3d_linefit.launch.py
ros2 launch simulation coverage_3d_linefit.launch.py database_path:=/home/developer/ws/maps/simulation/forest_linefit_3d.db
```

`ground_method`の既定値は`gseg3d`です。Linefit選択時はGSeg3Dを起動しません。
入力は移動補償済みの`/points_assembled`（base_link座標、既定2秒蓄積）、
出力は`/ground_segmentation/ground_points`と`/ground_segmentation/obstacle_points`です。
Nav2とRTAB-Mapには同じ非地面点群を渡します。
出力stampと座標系は入力を維持します。分類だけodomに対する回転で重力方向へ合わせ、
TFがない場合はその入力を配信しません。非有限点は除外します。
範囲外などLinefitが地面ラベルを付けなかった点も、非地面出力に入ります。

`linefit_config_file:=/絶対パス/設定.yaml`で閾値を変更できます。
設定は`config/linefit_ground.yaml`を参照してください。採用値は精度未検証の比較開始値です。
`sensor_height`はLiDAR筐体ではなく点群原点base_linkの地面からの高さ[m]で、
launchが機体寸法から計算して上書きします。パラメータの反映には再起動が必要です。

## GLIMによる3D SLAM

GLIMは公式PPAのCPU版aptパッケージを使います。ソースをワークスペースへ追加しません。
`package.xml`の`glim_ros`をローカルrosdep定義が`ros-jazzy-glim-ros`へ解決します。
Dockerfileが公式PPAと`rosdep-mirs.yaml`を登録するため、イメージ再ビルド後は
ほかの依存と同じ探索コマンドでインストールできます。

```bash
cd /home/developer/ws
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-select simulation
source install/setup.bash
ros2 launch simulation slam_3d_glim.launch.py
```

`/points`をEKFのodomで移動補償しながら既定2秒間蓄積し、
`/glim/input_points`（base_link座標）と`/imu/data_raw`をGLIMへ渡します。
地面分離を通さず、地面も含めて照合します。非円形バッファを使い、同じスキャンの繰り返しを避けます。
`cloud_assembling_time`で蓄積時間[s]を指定できます。初期化の6秒間は静止させてください。
2秒ごとの点群と高頻度IMUの時刻差について、GLIMが警告を出す場合があります。
警告だけで出力停止とは判断せず、`/glim/odom`などの更新を確認してください。
首振りの実測角度はscan_to_cloudが計測時刻の点群へ反映済みです。
既定URDFではIMU原点・軸がbase_linkと一致するため、GLIMのLiDAR–IMU変換は単位変換です。
蓄積点群は既にodomで補償済みとして、`global_shutter_lidar=true`でGLIMの二重の歪み補正を避けます。
この前提は実機の回転式LiDARや異なるIMU取付位置には適用できません。

GLIMの出力TFは`glim_map → glim_odom → glim_base`です。
`glim_base`はbase_linkと同じ物理位置を表す推定用の別名で、EKFの`odom → base_footprint`と
同じ子フレームを配信しません。EKFオドメトリはGLIMの姿勢入力へ融合しませんが、蓄積点群の形状には影響します。
GLIM結果をEKFへ戻す循環経路は作りません。
RTAB-Map、Nav2、カバレッジはこのlaunchでは起動しません。
GLIMの地図はRTAB-MapのDBやNav2用2D地図とは別形式です。
首振り点群が蓄積後も疎い・線状の場合は、3D姿勢を拘束する形状が不足する場合があります。
GLIMへ切り替えるだけで位置推定精度が保証されるものではありません。

`enable_glim_viewer:=true`でGLIMのGUIを表示できます。
`glim_config_path:=/絶対パス/設定ディレクトリ`でJSON設定を上書きできます。
起動時にapt版の既定設定を一時ディレクトリへコピーし、指定JSONを重ねます。
apt配布ファイルは編集しません。設定は`config/glim/`にあります。
GLIMプロセス内だけ`/usr/local/lib`を優先し、公式PPAのGTSAM 4.3とROS配布4.2の混在を防ぎます。
`dump_path`（既定`/home/developer/ws/maps/simulation/glim_dump`）はGLIM終了時の保存先です。
既存の保存先との混在を避けるには実験ごとに別のパスを指定します。

導入元: https://koide3.github.io/glim/installation.html

## simulationのEKFとIMU融合

`config/nodding_ekf.yaml`では`two_d_mode=false`として、Gazebo IMUのワールド基準姿勢、
角速度、加速度の各3軸を融合します。IMUのヨー角は初回値を差し引かず絶対姿勢として使います。
加速度は姿勢に基づいて9.8 m/s²の重力を除去してから使います。
これはGazeboの姿勢と重力を含む加速度が有効であることを前提にした設定です。
実機IMUが姿勢を出さない場合、NED座標の場合、重力除去済みの場合にはそのまま適用できません。
磁気センサー`/imu/mag`はEKFの入力ではありません。必要なら磁場から姿勢を求める別処理が必要です。

車輪からは機体座標の前進・横・上下速度とヨー角速度を使い、車輪の位置・姿勢は融合しません。
Gazeboの車輪オドメトリが出す横・上下速度0も、転がり運動の観測として有限の分散で融合します。
地形・滑りなどによりこのモデルが成立しない場合は、観測分散と入力選択を再評価します。
IMU姿勢を入れても、外部の位置観測がないためXYZの絶対位置誤差や高さのドリフトは残り得ます。
RTAB-Mapの`Reg/Force3DoF=true`は地図照合の平面制限で、EKFの3D推定とは別の設定です。

`sensor_covariance_adapter`は各観測に起動時指定の分散を付け、観測値・時刻を維持します。
分散は精度未検証の調整開始値で、実測に基づく校正値ではありません。
分散が有限かつ正でない場合は起動を失敗させます。
IMUの非有限値、無効な姿勢、利用不可フラグ（共分散先頭の-1）は警告して入力全体を破棄します。
車輪の採用速度が非有限の場合も破棄します。
設定変更後はsimulationを再起動してください。

## 地図の保存場所

運用地図はパッケージの外の`ws/maps/simulation/`へ保存します。
通常のSLAMとcoverageはともに`mirs_gazebo_3d.db`、Linefitは`forest_linefit_3d.db`を既定にします。
別名で保存する場合は、SLAMとcoverageの両方へ同じ`database_path`を指定してください。
SLAM起動時は指定DBを削除して作り直すため、残したいDBとは別名を指定します。
コンテナ内パスは`/home/developer/ws/maps/simulation/`です。

### 既存のJazzy環境にGLIMの依存定義を登録する

DockerfileでGLIM公式PPAを設定していない環境は、公式手順でPPAを登録してから、
このパッケージの`config/rosdep.yaml`をrosdepへ登録します。
登録用の一覧には絶対パスを指定します。以下は本リポジトリの配置での例です。

```bash
cd /home/developer/ws
printf 'yaml file://%s/src/original/simulation/config/rosdep.yaml\n' "$PWD" | sudo tee /etc/ros/rosdep/sources.list.d/10-mirs-glim.list
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

PPAの登録方法: https://koide3.github.io/glim/installation.html
ローカル定義だけではaptの配布元は登録されません。
