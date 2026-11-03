# 実機とGazeboの起動構成・競合のレビュー

対象はsimulation、mirs、nodding_lidar、coverageのlaunchと設定、および実機用mapping-3dの参照先。検証環境はROS 2 Jazzy。依存ライブラリとthirdpartyのソースは変更しない。

## 検出した不具合と対処

| 優先度 | 発生条件と影響 | 対処 |
|---|---|---|
| 高 | 実機のcollision_monitorが制御器と同じcmd_vel_navへ出力し、速度平滑化へ指令が循環する。安全ゲートも制御器の指令を直接受け取れる。 | 衝突監視の出力と安全ゲートの入力をcmd_vel_safeへ分離する。 |
| 高 | mirs_launchがCoverageNavigatorをNavigateToPoseNavigatorで上書きし、清掃用アクションの型・処理が設定ファイルと異なる。 | launchの上書きを除き、mirs/config/navigation/nav2_params.yamlをそのまま使う。 |
| 高 | nodding_lidar/nav_launchで固定base_link→laserと動的servo_link→laserを同時に配信する。 | 固定laser TFを無効にし、首振り側だけがlaserの親を配信する。IMU取付TFを有効にする。 |
| 高 | 首振り側がドライバを起動するためmirsのenable_lidar=falseになり、LiDAR付き走行でも安全ゲートが起動しない。 | enable_scan_safetyをドライバ起動条件から分離する。 |
| 高 | 独立SLAMとNav2のAMCLを同時に有効にするとmap→odomを二重配信する。 | mirsの独立SLAM＋Nav2、nodding bringupのRTAB-Map＋Nav2の組合せは起動前に拒否する。Nav2自身にSLAMを起動させる場合はslam:=Trueを使う。 |
| 中 | includeで設定したenable_lidarやenable_rvizが隣のincludeへ漏れ、意図しないドライバ停止・表示設定になる。 | 首振り実機launchのmirsとLiDARのincludeをGroupActionの範囲へ閉じる。 |
| 中 | 実機launchが親端末のDomain 91を継承すると、Gazeboとscan、odom、TF、cmd_velが混在する。 | mirsと首振り実機入口は90、simulationは91を子プロセスに設定する。 |
| 中 | Foxgloveは同一ホストの8765番を両構成が使用し、ROSドメインを分けても同時に待ち受けできない。 | 実機8765、simulation8766を既定値とする。 |
| 中 | シミュレーションのBLE受信がホストの実機用BluetoothアダプターとBlueZを使用する。 | simulationのBLEは既定で無効とし、必要時だけenable_ble:=trueを指定する。同一アダプターで実機側の受信と同時起動しない。 |
| 中 | mirs/nav_launchでEKFが起動せず、ファームウェアが配信しないodom→base_footprintのTFが欠ける。また実機coverageではIMU取付TFが無効で観測を変換できない。 | Nav2入口でEKFとIMU取付TF、実機coverageでIMU取付TFを起動する。 |

Linefit清掃launchは共通のcoverage_3dへ委譲し、BLEなどの既定値が別実装として分岐しないようにする。

## 担当と分離の境界

| 項目 | 実機 | Gazebo 3D |
|---|---|---|
| ROS_DOMAIN_ID | 90 | 91 |
| 時刻 | 実時刻 | /clock、use_sim_time=true |
| センサ | sllidar・ESP32・サーボ | ros_gz_bridge・nodding_io |
| odom→base_footprint | mirsのEKF | simulationのEKF |
| base_footprint→base_link | mirsの静的TF | robot_state_publisherのURDF |
| laserの取付TF | 固定LiDARまたは首振りTFの一方 | URDFとjoint_states |
| map→odom | AMCLまたはSLAMの一方 | RTAB-Map |
| Nav2設定 | mirs/config/navigation/nav2_params.yaml | simulation/config/nodding_nav2.yaml |
| 運用地図 | ~/ws/maps/real | ~/ws/maps/simulation |
| Foxglove | TCP 8765 | TCP 8766 |

Gazebo 2DはEKFを起動せず、GazeboのオドメトリTFを使う。3DはGazeboの生TFを/tf_gazeboへ退避する。GLIMはglim_map→glim_odom→glim_baseを使い、EKFのTFと別にする。

実機nodding_lidar/nav_launchはAMCLによる2D自己位置推定を使い、simulationの3D RTAB-Map構成とは異なる。首振りによるスキャン面の傾斜が照合へ与える影響は、通信分離とは別に実測評価が必要。

共有するnodding_lidarのscan_to_cloudはセンサ処理の実行ファイルであり、simulationから実機用ドライバlaunchを呼ばない。coverageもsimulationからenable_mirs=falseで呼び、Nav2設定を明示的に渡す。実機のEKF設定を編集してもsimulationのEKF設定には反映されない。

ROSドメインはトピック・TFの通信を分離するが、TCPポート、Bluetooth、シリアル機器、ファイル、Gazebo Transportは分離しない。Composeのsimulationサービスは実機サービスのデバイス・ホスト資源へのアクセス設定も継承している。同一ドメインで二台分のlaunchを同時に起動する構成には対応していない。

## 検証と限界

- test/test_stack_isolation.pyはノードを起動せず自作launchを展開し、ドメイン、LiDARドライバ数、安全ゲート数、TFの担当、Nav2設定、BLE・ポートの既定値、二重位置推定の拒否を検査する。外部launchは渡す引数までを検査する。
- mirs/test/scan_safety_integration.pyは専用ドメインと固有トピックで安全ゲートを実行し、開始時停止、障害物停止、0.35m／0.50mの停止・解除条件、スキャン・指令の0.5s途絶後の停止を検査する。モーターへの指令は出さない。
- nodding_lidarのgeometry、motion_window、laser_projection、motion_cloud_rosを再実行する。
- 読み取り専用ノードでDomain 90・91を確認した時点では、走行ノード、TF、速度指令の配信者を発見しなかった。実走行中の配信者重複やTFの飛びは未確認。
- 実機走行、停止距離の実測、カバレッジ完走、ICP精度、Humbleでの動作は検証していない。首振り点群の同期・特徴不足による照合不安定は、この構成分離だけでは解決を保証できない。

## 残る実機用launchの問題

mapping-3d/launch/rtabmap.launch.pyは、mirsに存在しないurdf/mirs.urdf.xacro、config/ekf_params.yamlと、ビルド対象にないodometry_publisher実行ファイルを参照する。この入口は稼働可能な実機構成として扱わない。simulationのモデルで代用すると実機の寸法・取付位置を誤用するため、自動的な代用は行わない。実測した車体・カメラ取付モデルの提供と、実機用起動APIへの接続整理が必要。

mapping-3dとservo_lidar_3dの独立SLAM入口も、一台分のEKF・RTAB-Map・LiDARを自分で起動する場合がある。mirs/noddingの完全なbringupへ追加して同時起動しない。
