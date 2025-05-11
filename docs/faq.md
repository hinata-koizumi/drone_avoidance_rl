# FAQ

## Q. Apple Silicon(M1/M2)で動く？
A. Dockerfileはarm64/amd64両対応。Gazebo/PX4/ROS 2の公式バイナリ状況に依存。

## Q. CI/CDでGazeboが止まる場合は？
A. timeout/exit code監視を導入済み。詳細はトラブルシューティング参照。 