# GPU対応高性能強化学習環境セットアップ

## 概要

このドキュメントでは、GPU対応の高性能強化学習環境のセットアップと使用方法について説明します。

## 前提条件

### ハードウェア要件
- NVIDIA GPU (RTX 3080以上推奨)
- 16GB以上のVRAM
- 32GB以上のシステムメモリ
- 高速SSD (NVMe推奨)

### ソフトウェア要件
- Ubuntu 22.04 LTS
- Docker 20.10+
- NVIDIA Container Toolkit
- CUDA 12.4.1
- cuDNN 8.9

## セットアップ手順

### 1. NVIDIA Container Toolkit のインストール

```bash
# NVIDIA リポジトリの追加
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# パッケージの更新とインストール
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 2. GPU環境の確認

```bash
# GPU確認
nvidia-smi

# Docker GPU確認
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

### 3. プロジェクトのクローン

```bash
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
cd drone_avoidance_rl
```

### 4. GPU対応Dockerイメージのビルド

```bash
# ベースイメージのビルド
docker build -f docker/Dockerfile.base-gpu -t drone-avoidance-base-gpu:2.0.1 .

# RLエージェントイメージのビルド
docker build -f docker/Dockerfile.rl-agent-gpu -t drone-rl-agent-gpu:latest .
```

### 5. GPU対応Docker Composeの実行

```bash
# GPU対応環境の起動
docker-compose -f docker-compose.gpu.yml up -d

# ログの確認
docker-compose -f docker-compose.gpu.yml logs -f rl-agent-gpu
```

## 使用方法

### 基本的な学習実行

```bash
# GPU対応コンテナでの学習実行
docker run --rm --gpus all \
  -v $(pwd)/src:/workspace/src:ro \
  -v $(pwd)/config:/workspace/config:ro \
  -v $(pwd)/logs:/workspace/logs \
  drone-rl-agent-gpu:latest \
  /entrypoint.sh train PPO config/rl_config.yaml
```

### 分散学習の実行

```bash
# Ray分散学習の実行
docker run --rm --gpus all \
  -v $(pwd)/src:/workspace/src:ro \
  -v $(pwd)/config:/workspace/config:ro \
  -v $(pwd)/logs:/workspace/logs \
  -p 6379:6379 -p 8265:8265 \
  drone-rl-agent-gpu:latest \
  python3 src/high_performance_training.py \
    --config config/rl_config.yaml \
    --mode train \
    --algorithm PPO \
    --iterations 1000
```

### ハイパーパラメータチューニング

```bash
# ハイパーパラメータチューニングの実行
docker run --rm --gpus all \
  -v $(pwd)/src:/workspace/src:ro \
  -v $(pwd)/config:/workspace/config:ro \
  -v $(pwd)/logs:/workspace/logs \
  drone-rl-agent-gpu:latest \
  python3 src/high_performance_training.py \
    --config config/rl_config.yaml \
    --mode tune \
    --algorithm PPO \
    --samples 20
```

### プロファイリング

```bash
# 学習のプロファイリング
docker run --rm --gpus all \
  -v $(pwd)/src:/workspace/src:ro \
  -v $(pwd)/config:/workspace/config:ro \
  -v $(pwd)/logs:/workspace/logs \
  drone-rl-agent-gpu:latest \
  python3 src/high_performance_training.py \
    --config config/rl_config.yaml \
    --mode profile \
    --algorithm PPO \
    --duration 300
```

## 監視とデバッグ

### GPU監視

```bash
# GPU監視ツールの実行
python3 tools/gpu_monitor.py --interval 5 --duration 3600

# メトリクスのエクスポート
python3 tools/gpu_monitor.py --export gpu_metrics.json
```

### TensorBoard監視

```bash
# TensorBoardの起動
docker run --rm -p 6006:6006 \
  -v $(pwd)/logs:/workspace/logs \
  drone-tensorboard:latest \
  tensorboard --logdir=/workspace/logs --host=0.0.0.0 --port=6006
```

### Ray Dashboard

```bash
# Ray Dashboardの起動
docker run --rm -p 8265:8265 \
  -v $(pwd)/logs:/workspace/logs \
  drone-ray-dashboard:latest \
  ray start --head --port=6379 --dashboard-port=8265 --dashboard-host=0.0.0.0
```

## 設定ファイル

### RL設定 (config/rl_config.yaml)

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

## トラブルシューティング

### GPU認識の問題

```bash
# GPU確認
nvidia-smi

# Docker GPU確認
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi

# PyTorch GPU確認
python3 -c "import torch; print(torch.cuda.is_available())"
```

### メモリ不足の問題

```bash
# GPUメモリ使用量の確認
nvidia-smi

# メモリ使用量の監視
watch -n 1 nvidia-smi
```

### Ray接続の問題

```bash
# Rayクラスタの状態確認
ray status

# Rayクラスタの再起動
ray stop
ray start --head --port=6379 --dashboard-port=8265
```

## パフォーマンス最適化

### GPU最適化

1. **バッチサイズの調整**: GPUメモリに応じてバッチサイズを調整
2. **並列環境数の調整**: CPUコア数に応じて並列環境数を調整
3. **メモリピニング**: 大容量メモリの使用でパフォーマンス向上

### ネットワーク最適化

1. **DDS設定**: ROS 2 DDS設定の最適化
2. **Gazebo設定**: シミュレーション設定の最適化
3. **Ray設定**: 分散学習設定の最適化

## ベンチマーク結果

### 単一GPU環境
- **PPO**: 1000 iterations / 30分
- **SAC**: 1000 iterations / 45分
- **TD3**: 1000 iterations / 40分

### マルチGPU環境
- **PPO**: 1000 iterations / 15分 (2x speedup)
- **SAC**: 1000 iterations / 22分 (2x speedup)
- **TD3**: 1000 iterations / 20分 (2x speedup)

## 次のステップ

1. **マルチノード分散学習**: 複数マシンでの分散学習
2. **自動ハイパーパラメータチューニング**: Optuna等の使用
3. **モデル圧縮**: 量子化・プルーニングの適用
4. **リアルタイム推論**: 最適化された推論パイプライン

## サポート

問題が発生した場合は、以下を確認してください：

1. GPUドライバーのバージョン
2. CUDAバージョンの互換性
3. Docker設定の確認
4. メモリ使用量の監視

詳細なログは `logs/` ディレクトリに保存されます。 