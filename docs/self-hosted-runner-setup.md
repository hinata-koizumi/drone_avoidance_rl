# セルフホストGPUランナー設定ガイド

## 概要

GitHub ActionsでGPUテストを実行するには、セルフホストランナーが必要です。

## セットアップ手順

### 1. セルフホストランナーの作成

#### GitHubリポジトリでの設定
1. リポジトリの **Settings** → **Actions** → **Runners**
2. **New self-hosted runner** をクリック
3. **Linux** を選択
4. 表示されるコマンドをコピー

#### サーバーでの実行
```bash
# 1. ランナーのダウンロード
curl -o actions-runner-linux-x64-2.311.0.tar.gz -L https://github.com/actions/runner/releases/download/v2.311.0/actions-runner-linux-x64-2.311.0.tar.gz

# 2. 解凍
tar xzf ./actions-runner-linux-x64-2.311.0.tar.gz

# 3. 設定
./config.sh --url https://github.com/hinata-koizumi/drone_avoidance_rl --token YOUR_TOKEN

# 4. ラベル設定
./config.sh --labels gpu,self-hosted,linux,x64

# 5. サービスとして起動
sudo ./svc.sh install
sudo ./svc.sh start
```

### 2. GPU環境の準備

#### NVIDIAドライバーとDocker
```bash
# NVIDIAドライバー確認
nvidia-smi

# Docker GPU対応確認
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

#### 必要なパッケージのインストール
```bash
# システムパッケージ
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    docker.io \
    docker-compose

# Pythonパッケージ
pip3 install torch==2.1.2+cu121 torchvision==0.16.2+cu121 torchaudio==2.1.2+cu121 --index-url https://download.pytorch.org/whl/cu121
pip3 install ray[rllib]==2.8.0
pip3 install -r requirements-gpu.txt
```

### 3. ランナー設定ファイル

#### `.github/runners/self-hosted/run.sh`
```bash
#!/bin/bash

# GPU環境変数
export CUDA_VISIBLE_DEVICES=0
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=compute,utility

# Docker権限
sudo usermod -aG docker $USER

# ランナー起動
./run.sh
```

### 4. ワークフロー設定の確認

#### `.github/workflows/gpu-ci.yml`
```yaml
jobs:
  gpu-check:
    runs-on: [self-hosted, gpu]  # セルフホストGPUランナー
    steps:
      - name: Check GPU availability
        run: |
          nvidia-smi
          echo "GPU available"
```

## トラブルシューティング

### よくある問題

#### 1. GPU認識エラー
```bash
# 解決策
sudo nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

#### 2. Docker権限エラー
```bash
# 解決策
sudo usermod -aG docker $USER
newgrp docker
```

#### 3. メモリ不足
```bash
# 解決策
# /etc/docker/daemon.json
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```

## セキュリティ考慮事項

### 1. アクセス制御
- セルフホストランナーは信頼できる環境でのみ使用
- プライベートリポジトリでの使用を推奨

### 2. リソース管理
- GPU使用率の監視
- メモリ使用量の制限
- 同時実行数の制御

### 3. ログ管理
- 実行ログの保存
- エラーログの監視
- セキュリティログの確認

## パフォーマンス最適化

### 1. GPU設定
```bash
# GPUメモリ設定
nvidia-smi -c 3  # 計算モード
nvidia-smi -pm 1  # パフォーマンスモード
```

### 2. Docker設定
```bash
# Dockerデーモン設定
# /etc/docker/daemon.json
{
  "default-shm-size": "2G",
  "storage-driver": "overlay2"
}
```

### 3. システム設定
```bash
# カーネルパラメータ
echo 'vm.max_map_count=262144' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## 監視とメンテナンス

### 1. ヘルスチェック
```bash
# ランナー状態確認
./run.sh --help
./run.sh --version

# GPU状態確認
nvidia-smi
nvidia-smi -l 1
```

### 2. ログ監視
```bash
# ランナーログ
tail -f _diag/run-*.log

# Dockerログ
docker logs CONTAINER_ID
```

### 3. 定期メンテナンス
```bash
# 週次メンテナンス
sudo apt-get update && sudo apt-get upgrade
docker system prune -f
nvidia-smi --gpu-reset
``` 