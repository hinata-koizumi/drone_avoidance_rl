#!/usr/bin/env python3
"""
分散学習対応の高性能RL環境
"""

import numpy as np
import torch
import gymnasium as gym
from gymnasium import spaces
from typing import Dict, List, Optional, Tuple, Any
import logging
from concurrent.futures import ThreadPoolExecutor
import threading
import time

from drone_sim_env import DroneSimEnv

class DistributedDroneEnv(gym.Env):
    """分散学習対応のドローン環境"""
    
    def __init__(self, 
                 num_envs: int = 8,
                 use_gpu: bool = True,
                 device: Optional[str] = None):
        super().__init__()
        
        self.num_envs = num_envs
        self.use_gpu = use_gpu
        
        # デバイス設定
        if device is None:
            self.device = torch.device("cuda" if use_gpu and torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
            
        # 環境の初期化
        self.envs = []
        self._init_environments()
        
        # 観測・行動空間の設定
        self.observation_space = self._get_observation_space()
        self.action_space = self._get_action_space()
        
        # ロガーの設定
        self.logger = self._setup_logger()
        
        # 統計情報
        self.episode_rewards = np.zeros(num_envs)
        self.episode_lengths = np.zeros(num_envs)
        self.episode_count = 0
        
    def _init_environments(self):
        """環境の初期化"""
        self.logger.info(f"Initializing {self.num_envs} environments on {self.device}")
        
        for i in range(self.num_envs):
            env = DroneSimEnv()
            self.envs.append(env)
            
        # GPU対応の観測・行動バッファ
        self.obs_buffer = torch.zeros(
            (self.num_envs, *self.envs[0].observation_space.shape),
            dtype=torch.float32,
            device=self.device
        )
        
        self.action_buffer = torch.zeros(
            (self.num_envs, *self.envs[0].action_space.shape),
            dtype=torch.float32,
            device=self.device
        )
        
    def _get_observation_space(self) -> spaces.Box:
        """観測空間の取得"""
        # 最初の環境から観測空間を取得
        obs_space = self.envs[0].observation_space
        
        # 複数環境対応の観測空間
        return spaces.Box(
            low=np.tile(obs_space.low, (self.num_envs, 1)),
            high=np.tile(obs_space.high, (self.num_envs, 1)),
            dtype=obs_space.dtype
        )
        
    def _get_action_space(self) -> spaces.Box:
        """行動空間の取得"""
        # 最初の環境から行動空間を取得
        action_space = self.envs[0].action_space
        
        # 複数環境対応の行動空間
        return spaces.Box(
            low=np.tile(action_space.low, (self.num_envs, 1)),
            high=np.tile(action_space.high, (self.num_envs, 1)),
            dtype=action_space.dtype
        )
        
    def _setup_logger(self) -> logging.Logger:
        """ロガーの設定"""
        logger = logging.getLogger(f'distributed_drone_env_{id(self)}')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            
        return logger
        
    def reset(self, seed: Optional[int] = None) -> Tuple[torch.Tensor, Dict]:
        """環境のリセット"""
        if seed is not None:
            np.random.seed(seed)
            
        # 全環境のリセット
        observations = []
        infos = []
        
        for i, env in enumerate(self.envs):
            obs, info = env.reset()
            observations.append(obs)
            infos.append(info)
            
        # GPU対応の観測テンソル
        obs_tensor = torch.tensor(observations, dtype=torch.float32, device=self.device)
        
        # 統計情報のリセット
        self.episode_rewards = np.zeros(self.num_envs)
        self.episode_lengths = np.zeros(self.num_envs)
        
        return obs_tensor, {"infos": infos}
        
    def step(self, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, Dict]:
        """環境のステップ実行（並列化版）"""
        # 行動をCPUに移動（ROS 2との互換性のため）
        if actions.device != torch.device("cpu"):
            actions = actions.cpu().numpy()
        else:
            actions = actions.numpy()
            
        # 並列環境ステップ実行
        observations = []
        rewards = []
        dones = []
        infos = []
        
        # ThreadPoolExecutorを使用した並列実行
        with ThreadPoolExecutor(max_workers=min(self.num_envs, 8)) as executor:
            # 各環境のステップを並列実行
            future_to_env = {
                executor.submit(self._step_single_env, i, actions[i]): i 
                for i in range(self.num_envs)
            }
            
            # 結果を収集
            for future in future_to_env:
                try:
                    obs, reward, done, info = future.result(timeout=5.0)
                    observations.append(obs)
                    rewards.append(reward)
                    dones.append(done)
                    infos.append(info)
                except Exception as e:
                    self.logger.error(f"Environment step failed: {e}")
                    # エラー時はデフォルト値を設定
                    observations.append(np.zeros(self.envs[0].observation_space.shape))
                    rewards.append(0.0)
                    dones.append(True)
                    infos.append({})
                
        # GPU対応のテンソルに変換
        obs_tensor = torch.tensor(observations, dtype=torch.float32, device=self.device)
        reward_tensor = torch.tensor(rewards, dtype=torch.float32, device=self.device)
        done_tensor = torch.tensor(dones, dtype=torch.bool, device=self.device)
        
        return obs_tensor, reward_tensor, done_tensor, done_tensor, {"infos": infos}
        
    def _step_single_env(self, env_idx: int, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """単一環境のステップ実行"""
        try:
            obs, reward, done, truncated, info = self.envs[env_idx].step(action)
            
            # 統計情報の更新
            self.episode_rewards[env_idx] += reward
            self.episode_lengths[env_idx] += 1
            
            # エピソード終了時の統計
            if done or truncated:
                self.episode_count += 1
                self.logger.info(f"Episode {self.episode_count}: "
                               f"Reward={self.episode_rewards[env_idx]:.2f}, "
                               f"Length={self.episode_lengths[env_idx]}")
                
            return obs, reward, done or truncated, info
            
        except Exception as e:
            self.logger.error(f"Environment {env_idx} step error: {e}")
            return np.zeros(self.envs[env_idx].observation_space.shape), 0.0, True, {}
        
    def render(self):
        """描画（分散環境では無効）"""
        pass
        
    def close(self):
        """環境の終了"""
        for env in self.envs:
            env.close()
            
    def get_statistics(self) -> Dict[str, float]:
        """統計情報の取得"""
        return {
            "episode_count": self.episode_count,
            "mean_episode_reward": np.mean(self.episode_rewards),
            "std_episode_reward": np.std(self.episode_rewards),
            "mean_episode_length": np.mean(self.episode_lengths),
            "std_episode_length": np.std(self.episode_lengths)
        }

class VectorizedDroneEnv:
    """ベクトル化されたドローン環境（Ray RLlib対応）"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.num_envs = config.get("num_envs", 8)
        self.use_gpu = config.get("use_gpu", True)
        
        # 環境の初期化
        self.env = DistributedDroneEnv(
            num_envs=self.num_envs,
            use_gpu=self.use_gpu
        )
        
    def reset(self):
        """環境のリセット"""
        return self.env.reset()
        
    def step(self, actions):
        """環境のステップ実行"""
        return self.env.step(actions)
        
    @property
    def observation_space(self):
        return self.env.observation_space
        
    @property
    def action_space(self):
        return self.env.action_space
        
    def close(self):
        self.env.close()

# Ray RLlib用の環境登録
def register_drone_env():
    """ドローン環境の登録"""
    from ray.rllib.env.env_context import EnvContext
    
    def env_creator(env_config: EnvContext) -> VectorizedDroneEnv:
        return VectorizedDroneEnv(env_config)
        
    # 環境の登録
    try:
        from ray.rllib.env import register_env
        register_env("DroneAvoidance-v0", env_creator)
        print("Registered DroneAvoidance-v0 environment with Ray RLlib")
    except ImportError:
        print("Ray RLlib not available, skipping environment registration")

if __name__ == "__main__":
    # テスト実行
    register_drone_env()
    
    # 環境のテスト
    env = DistributedDroneEnv(num_envs=4, use_gpu=True)
    obs, info = env.reset()
    
    print(f"Observation shape: {obs.shape}")
    print(f"Device: {obs.device}")
    
    for i in range(100):
        actions = torch.randn(4, env.action_space.shape[0], device=obs.device)
        obs, rewards, dones, truncated, info = env.step(actions)
        
        if torch.any(dones):
            print(f"Episode completed at step {i}")
            break
            
    env.close()
    print("Test completed successfully") 