# [English README](README.md)

# drone-rl

ドローン用強化学習環境（OpenAI Gym API・ROS2連携・ドメインランダム化対応）

## 概要

本リポジトリは、ドローン自律制御・回避・ナビゲーションのための強化学習（RL）環境を提供します。
- **OpenAI Gymnasium API完全互換**
- **Ray RLlibによる分散学習・ベクトル化**
- **ROS 2 Humble/シミュレータ連携**
- **ドメインランダム化（初期位置・目標・質量・風）**
- **テスト・CI・Docker対応**

## 主要ファイル
- `drone_sim_env.py`: RL用Gym環境本体（reset/step/render/close, ROS2連携, ランダム化）
- `gym_env.py`: 環境登録・生成ユーティリティ
- `ros_interface.py`: ROS2トピック連携シングルトン
- `train_ray.py`: Ray RLlib用トレーニングスクリプト（PPO, env_config対応）
- `vector_env_example.py`: ベクトル化環境サンプル
- `tests/`: API・仕様テスト

## 環境仕様

### Observation Space
```python
Box(low=-inf, high=inf, shape=(10,), dtype=np.float32)
# [x, y, z, roll, pitch, yaw, vx, vy, vz, battery]
```

### Action Space
```python
Box(low=[0,-1,-1,-1], high=[1,1,1,1], dtype=np.float32)
# [thrust, roll, pitch, yaw_rate]
```

### Reward/Done
- 報酬: `-distance_to_target - 0.1*speed + 0.05*height`
- 終了: 距離<0.5m, 高度<0.2, バッテリ<0.05, 速度>25, 範囲外, ステップ上限
- info: `{"distance", "speed", "battery", "height", "step"}`

### ドメインランダム化
- `randomization_params`で初期位置・目標・質量・風を乱数サンプリング
- 例: `{"init_pos_range": [[-2,-2,3.5],[2,2,4.5]], ...}`

## 使い方

### 依存関係
- Python 3.10+
- gymnasium==1.0.0, numpy, torch>=2.0, ray[rllib]==2.47.1, tensorboard, pytest
- ROS2 (オプション)

### 環境登録・テスト
```bash
pip install -r requirements.txt
python -c "from gym_env import register_drone_env; register_drone_env()"
pytest tests/
```

### 単体実行例
```python
from gym_env import create_env
env = create_env(target_pos=[5,0,4], init_pos=[0,0,4], use_ros=False)
obs, info = env.reset()
done = False
while not done:
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    done = terminated or truncated
env.close()
```

### Ray RLlibトレーニング
```bash
python train_ray.py --num-workers 2 --train-iterations 10 \
  --target-pos 5 0 4 --init-pos 0 0 4 --randomize
```

### ベクトル化環境
```python
from gymnasium.vector import AsyncVectorEnv
from drone_sim_env import DroneSimEnv

def make_env(i):
    return lambda: DroneSimEnv(instance_id=i, use_ros=False)
num_envs = 4
env = AsyncVectorEnv([make_env(i) for i in range(num_envs)])
obs = env.reset()
...
```

### Docker
```bash
docker build -f docker/rl-agent/Dockerfile -t drone-rl:gpu .
docker run --gpus all -v $(pwd)/results:/workspace/results drone-rl:gpu \
  python train_ray.py --num-workers 4 --train-iterations 100
```

## ROS2連携
- `/drone0/state` (DroneState), `/drone0/control` (DroneControlCommand) でシミュレータと通信
- `use_ros=False`でスタンドアロン動作も可能
- `ros_interface.py`でpublish/subscribeを抽象化

## テスト
- `pytest tests/` でAPI・仕様・ランダム化・終了条件・ダミー動作を網羅的に検証

## 拡張例
- 画像センサ・複数ドローン・カスタム報酬・CI自動検証も容易に追加可能

## 貢献
- Pull Request歓迎。新規アルゴリズム・エージェント・ベンチマーク等の追加も歓迎。 

## セキュリティスキャン（CodeQL）について

- 本リポジトリでは **GitHub ActionsによるカスタムCodeQLワークフロー（.github/workflows/codeql-analysis.yml）** のみを有効化しています。
- **GitHubの「デフォルトセットアップ」CodeQLは無効化** しています（両者の併用はできません）。
- 今後も「カスタムワークフローのみ」を維持してください。
- Trivyによる脆弱性スキャンも併用していますが、カテゴリ分けされているため競合しません。 