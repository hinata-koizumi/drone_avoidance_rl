#!/bin/bash
set -e

# GPU環境の確認
echo "=== GPU Environment Check ==="
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA-SMI available:"
    nvidia-smi
else
    echo "Warning: NVIDIA-SMI not available"
fi

# CUDA環境の確認
echo "=== CUDA Environment Check ==="
if [ -n "$CUDA_VISIBLE_DEVICES" ]; then
    echo "CUDA_VISIBLE_DEVICES: $CUDA_VISIBLE_DEVICES"
else
    echo "CUDA_VISIBLE_DEVICES not set"
fi

# PyTorch GPU確認
echo "=== PyTorch GPU Check ==="
python3 -c "
import torch
print(f'PyTorch version: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'CUDA version: {torch.version.cuda}')
    print(f'GPU count: {torch.cuda.device_count()}')
    for i in range(torch.cuda.device_count()):
        print(f'GPU {i}: {torch.cuda.get_device_name(i)}')
"

# ROS 2環境の設定
source /opt/ros/humble/setup.sh
if [ -f "/workspace/install/setup.sh" ]; then
    source /workspace/install/setup.sh
fi

# Ray環境の設定
export RAY_DISABLE_IMPORT_WARNING=1
export RAY_DEDUP_LOGS=0

# 作業ディレクトリの設定
cd /workspace

# GPU監視の開始（バックグラウンド）
echo "=== Starting GPU Monitor ==="
python3 tools/gpu_monitor.py --interval 10 &
GPU_MONITOR_PID=$!

# 分散学習の実行
echo "=== Starting Distributed RL Training ==="

# 引数の解析
MODE=${1:-"train"}
ALGORITHM=${2:-"PPO"}
CONFIG_FILE=${3:-"config/rl_config.yaml"}

case $MODE in
    "train")
        echo "Starting training with algorithm: $ALGORITHM"
        
        # Ray クラスタの開始
        ray start --head --port=6379 --dashboard-port=8265 --dashboard-host=0.0.0.0
        
        # 学習の実行
        python3 -m ray.rllib.train \
            --algorithm $ALGORITHM \
            --config $CONFIG_FILE \
            --stop '{"training_iteration": 1000}' \
            --local-dir /workspace/logs/ray_results
        ;;
        
    "evaluate")
        echo "Starting evaluation"
        CHECKPOINT_PATH=${4:-"/workspace/checkpoints/latest"}
        
        python3 -m ray.rllib.evaluate \
            --algorithm $ALGORITHM \
            --config $CONFIG_FILE \
            --checkpoint $CHECKPOINT_PATH \
            --local-dir /workspace/logs/evaluation
        ;;
        
    "tune")
        echo "Starting hyperparameter tuning"
        
        python3 -m ray.tune \
            --config $CONFIG_FILE \
            --local-dir /workspace/logs/tune_results \
            --num_samples 10
        ;;
        
    "profile")
        echo "Starting profiling"
        DURATION=${4:-300}
        
        python3 tools/profiling/nsys_profile.py \
            --ray-config $CONFIG_FILE \
            --duration $DURATION \
            --name "ray_training_profile"
        ;;
        
    "monitor")
        echo "Starting monitoring only"
        # GPU監視のみ実行
        wait $GPU_MONITOR_PID
        ;;
        
    *)
        echo "Usage: $0 {train|evaluate|tune|profile|monitor} [algorithm] [config_file] [checkpoint_path|duration]"
        echo "Examples:"
        echo "  $0 train PPO config/rl_config.yaml"
        echo "  $0 evaluate SAC /workspace/checkpoints/latest"
        echo "  $0 profile 300"
        exit 1
        ;;
esac

# GPU監視の停止
if [ -n "$GPU_MONITOR_PID" ]; then
    echo "Stopping GPU monitor..."
    kill $GPU_MONITOR_PID 2>/dev/null || true
fi

echo "=== RL Agent GPU Execution Complete ===" 