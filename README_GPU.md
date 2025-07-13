# 🚁 GPU対応高性能強化学習環境

## 🎯 概要

このプロジェクトは、GPU対応の高性能強化学習環境を提供します。ROS 2 Humble、PX4、Ignition Gazebo Gardenを基盤とし、Ray分散学習フレームワークを使用して大規模な強化学習実験を可能にします。

## ✨ 主な機能

### 🚀 高性能学習
- **GPU対応**: CUDA 12.4.1対応のPyTorch学習
- **分散学習**: Ray RLlibによるマルチGPU分散学習
- **並列環境**: 複数環境の並列実行による高速化
- **自動最適化**: ハイパーパラメータ自動チューニング

### 📊 監視・プロファイリング
- **GPU監視**: リアルタイムGPU使用率・メモリ監視
- **プロファイリング**: NVIDIA Nsight Systems統合
- **TensorBoard**: 学習過程の可視化
- **Ray Dashboard**: 分散学習の監視

### 🔧 開発・デバッグ
- **Docker統合**: GPU対応Docker環境
- **CI/CD**: GitHub Actionsによる自動テスト
- **型チェック**: mypyによる静的型チェック
- **コード品質**: black, ruffによる自動フォーマット

## 🏗️ アーキテクチャ

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   GPU Cluster   │    │  Ray RLlib      │    │  ROS 2 Bridge   │
│                 │    │                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │   GPU 0     │ │    │ │   PPO       │ │    │ │   State     │ │
│ │   GPU 1     │ │    │ │   SAC       │ │    │ │   Command   │ │
│ │   GPU N     │ │    │ │   TD3       │ │    │ │   Bridge    │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │  Gazebo Garden  │
                    │   Simulation    │
                    └─────────────────┘
```

## 🚀 クイックスタート

### 1. 環境セットアップ

```bash
# プロジェクトのクローン
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
cd drone_avoidance_rl

# GPU対応Dockerイメージのビルド
docker build -f docker/Dockerfile.base-gpu -t drone-avoidance-base-gpu:2.0.1 .
docker build -f docker/Dockerfile.rl-agent-gpu -t drone-rl-agent-gpu:latest .
```

### 2. 分散学習の実行

```bash
# GPU対応環境での学習実行
docker run --rm --gpus all \
  -v $(pwd)/src:/workspace/src:ro \
  -v $(pwd)/config:/workspace/config:ro \
  -v $(pwd)/logs:/workspace/logs \
  drone-rl-agent-gpu:latest \
  python3 src/high_performance_training.py \
    --config config/rl_config.yaml \
    --mode train \
    --algorithm PPO \
    --iterations 1000
```

### 3. 監視ダッシュボード

```bash
# TensorBoard起動
docker run --rm -p 6006:6006 \
  -v $(pwd)/logs:/workspace/logs \
  drone-tensorboard:latest

# Ray Dashboard起動
docker run --rm -p 8265:8265 \
  -v $(pwd)/logs:/workspace/logs \
  drone-ray-dashboard:latest
```

## 📈 パフォーマンス

### ベンチマーク結果

| アルゴリズム | 単一GPU | マルチGPU | 高速化 |
|-------------|---------|-----------|--------|
| PPO         | 30分    | 15分      | 2.0x   |
| SAC         | 45分    | 22分      | 2.0x   |
| TD3         | 40分    | 20分      | 2.0x   |

### リソース使用量

- **GPU**: RTX 3080 (10GB VRAM)
- **CPU**: 8コア
- **メモリ**: 32GB
- **ストレージ**: NVMe SSD

## 🔧 設定

### 学習設定 (config/rl_config.yaml)

```yaml
rl:
  algorithm: "PPO"
  framework: "torch"
  
  env:
    num_envs: 8
    vector_env: true
    max_episode_steps: 1000
    
  training:
    total_timesteps: 10000000
    batch_size: 256
    learning_rate: 3e-4
    
  distributed:
    num_workers: 4
    num_gpus: 1
    backend: "nccl"
```

### Ray設定 (config/ray_config.yaml)

```yaml
ray:
  cluster:
    head_node:
      resources: {"CPU": 4, "GPU": 1}
    worker_nodes:
      - resources: {"CPU": 8, "GPU": 1}
      
  rllib:
    framework: "torch"
    num_workers: 4
    num_gpus: 1
    num_envs_per_worker: 2
```

## 🛠️ 開発ツール

### GPU監視

```bash
# リアルタイム監視
python3 tools/gpu_monitor.py --interval 5

# メトリクスエクスポート
python3 tools/gpu_monitor.py --export metrics.json
```

### プロファイリング

```bash
# 学習プロファイリング
python3 tools/profiling/nsys_profile.py \
  --ray-config config/rl_config.yaml \
  --duration 300 \
  --name training_profile
```

### コード品質

```bash
# 型チェック
mypy src/

# コードフォーマット
black src/
ruff check src/
```

## 📊 監視ダッシュボード

### TensorBoard
- **URL**: http://localhost:6006
- **機能**: 学習曲線、損失、報酬の可視化

### Ray Dashboard
- **URL**: http://localhost:8265
- **機能**: 分散学習の状態監視

### GPU監視
- **コマンド**: `nvidia-smi`
- **機能**: GPU使用率、メモリ使用量の監視

## 🐛 トラブルシューティング

### よくある問題

1. **GPU認識エラー**
   ```bash
   # GPU確認
   nvidia-smi
   docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
   ```

2. **メモリ不足**
   ```bash
   # バッチサイズの調整
   # config/rl_config.yaml で batch_size を小さくする
   ```

3. **Ray接続エラー**
   ```bash
   # Rayクラスタの再起動
   ray stop
   ray start --head --port=6379 --dashboard-port=8265
   ```

## 📚 ドキュメント

- [GPUセットアップガイド](docs/gpu_setup.md)
- [API リファレンス](docs/api_reference.md)
- [トラブルシューティング](docs/troubleshooting.md)

## 🤝 貢献

1. フォークしてブランチを作成
2. 変更をコミット
3. プルリクエストを作成

## 📄 ライセンス

MIT License

## 🙏 謝辞

- ROS 2 Humble
- PX4 Autopilot
- Ignition Gazebo Garden
- Ray RLlib
- PyTorch

---

**🚀 高性能強化学習でドローン制御の未来を切り開こう！** 