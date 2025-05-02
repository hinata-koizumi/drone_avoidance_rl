# drone_avoidance_rl ― PX4 × ROS 2 × RL 参考環境

このリポジトリは **「ドローンモデルを差し込めば即動く」** ことを目的にした最小構成です。  
再現性を担保するため **日時固定の Ubuntu/ROS/PyPI スナップショット** を使用し、  
PX4‐SITL と RL エージェントを **マルチステージ Docker** で分離しています。

## クイックスタート

# 1. 取得
git clone https://github.com/yourname/drone_avoidance_rl.git
cd drone_avoidance_rl

# 2. 独自 SDF / airframe 挿入（任意）
cp -r ~/my_drone_sdf custom_model/drone_model
cp ~/4500_my_drone.json custom_airframes/

# 3. Docker Desktop を起動し buildx を arm64 に
docker buildx create --name arm_builder --driver docker-container --use || true

# 4. ホスト ↔ コンテナ UDP14556 を forward（Docker Desktop 設定 → Resources → Networking）
#    例: host-port 14556 ↔ container-port 14556/udp

# 5. フルスタックをビルド & 起動（CPU/MPS）
docker compose --profile cpu up -d --build
#    GPU(M-series) の場合は --profile gpu

# 6. 学習ログは logs/ 以下。停止は:
docker compose down


>>>>>>> f3a5ee7 (Initial commit)
