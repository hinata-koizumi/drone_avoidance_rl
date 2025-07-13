#!/usr/bin/env python3
"""
高性能分散学習スクリプト
"""

import os
import sys
import yaml
import argparse
import logging
from typing import Dict, Any, Optional
import torch
import numpy as np

# Ray関連のインポート
try:
    import ray
    from ray import tune
    from ray.rllib.algorithms.ppo import PPOConfig
    from ray.rllib.algorithms.sac import SACConfig
    from ray.rllib.algorithms.td3 import TD3Config
    from ray.rllib.algorithms.dqn import DQNConfig
    RAY_AVAILABLE = True
except ImportError:
    RAY_AVAILABLE = False
    print("Warning: Ray not available. Install with: pip install ray[rllib]")

# 環境の登録
from distributed_rl_env import register_drone_env

class HighPerformanceTrainer:
    """高性能分散学習トレーナー"""
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self._load_config()
        self.logger = self._setup_logger()
        
        # Ray初期化
        if RAY_AVAILABLE:
            self._init_ray()
        else:
            raise RuntimeError("Ray is required for high-performance training")
            
        # 環境の登録
        register_drone_env()
        
    def _load_config(self) -> Dict[str, Any]:
        """設定ファイルの読み込み"""
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)
            
    def _setup_logger(self) -> logging.Logger:
        """ロガーの設定"""
        logger = logging.getLogger('high_performance_trainer')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            
        return logger
        
    def _init_ray(self):
        """Rayの初期化"""
        # Ray設定
        ray_config = self.config.get("ray", {})
        
        # クラスタ設定
        if "cluster" in ray_config:
            cluster_config = ray_config["cluster"]
            head_resources = cluster_config.get("head_node", {}).get("resources", {})
            
            # ヘッドノードのリソース設定
            num_cpus = head_resources.get("CPU", 4)
            num_gpus = head_resources.get("GPU", 1)
            
            # Ray初期化
            ray.init(
                num_cpus=num_cpus,
                num_gpus=num_gpus,
                local_mode=False,
                ignore_reinit_error=True
            )
        else:
            # デフォルト設定
            ray.init(
                num_cpus=8,
                num_gpus=1,
                local_mode=False,
                ignore_reinit_error=True
            )
            
        self.logger.info("Ray initialized successfully")
        
    def _get_algorithm_config(self, algorithm: str) -> Any:
        """アルゴリズム設定の取得"""
        rllib_config = self.config.get("ray", {}).get("rllib", {})
        algorithms_config = rllib_config.get("algorithms", {})
        
        if algorithm == "PPO":
            config = PPOConfig()
            ppo_config = algorithms_config.get("ppo", {})
            config = config.training(**ppo_config)
            
        elif algorithm == "SAC":
            config = SACConfig()
            sac_config = algorithms_config.get("sac", {})
            config = config.training(**sac_config)
            
        elif algorithm == "TD3":
            config = TD3Config()
            td3_config = algorithms_config.get("td3", {})
            config = config.training(**td3_config)
            
        elif algorithm == "DQN":
            config = DQNConfig()
            dqn_config = algorithms_config.get("dqn", {})
            config = config.training(**dqn_config)
            
        else:
            raise ValueError(f"Unsupported algorithm: {algorithm}")
            
        # 環境設定
        env_config = rllib_config.get("env", {})
        config = config.environment(
            "DroneAvoidance-v0",
            env_config=env_config
        )
        
        # 混合精度学習設定
        mixed_precision_config = self.config.get("rl", {}).get("mixed_precision", {})
        if mixed_precision_config.get("enabled", True):
            config = config.training(
                framework="torch",
                torch_gpu=True,
                torch_compile_AMP=True,
                fp16=mixed_precision_config.get("fp16", True),
                bf16=mixed_precision_config.get("bf16", False)
            )
            
        # 分散学習設定
        distributed_config = rllib_config.get("distributed", {})
        if distributed_config:
            config = config.resources(
                num_gpus=distributed_config.get("num_gpus", 1),
                num_cpus_per_worker=distributed_config.get("num_cpus_per_worker", 1),
                num_gpus_per_worker=distributed_config.get("num_gpus_per_worker", 0.25)
            )
            
        # チェックポイント設定
        checkpoint_config = rllib_config.get("checkpoint", {})
        if checkpoint_config:
            config = config.checkpointing(
                checkpoint_freq=checkpoint_config.get("frequency", 10),
                checkpoint_at_end=checkpoint_config.get("checkpoint_at_end", True)
            )
            
        # 監視設定
        monitoring_config = rllib_config.get("monitoring", {})
        if monitoring_config:
            config = config.reporting(
                log_level=monitoring_config.get("log_level", "INFO"),
                metrics_smoothing_episodes=monitoring_config.get("metrics_smoothing_episodes", 100)
            )
            
        return config
        
    def train(self, algorithm: str = "PPO", num_iterations: int = 1000):
        """学習の実行"""
        self.logger.info(f"Starting training with algorithm: {algorithm}")
        
        # アルゴリズム設定の取得
        config = self._get_algorithm_config(algorithm)
        
        # 学習の実行
        results = tune.run(
            algorithm,
            config=config,
            stop={"training_iteration": num_iterations},
            local_dir="./logs/ray_results",
            checkpoint_freq=10,
            keep_checkpoints_num=5,
            verbose=1
        )
        
        self.logger.info("Training completed successfully")
        return results
        
    def evaluate(self, checkpoint_path: str, algorithm: str = "PPO"):
        """評価の実行"""
        self.logger.info(f"Starting evaluation with checkpoint: {checkpoint_path}")
        
        # アルゴリズム設定の取得
        config = self._get_algorithm_config(algorithm)
        
        # エージェントの復元
        agent = config.build()
        agent.restore(checkpoint_path)
        
        # 評価の実行
        evaluation_results = agent.evaluate()
        
        self.logger.info("Evaluation completed")
        return evaluation_results
        
    def hyperparameter_tuning(self, algorithm: str = "PPO", num_samples: int = 10):
        """ハイパーパラメータチューニング"""
        self.logger.info(f"Starting hyperparameter tuning for {algorithm}")
        
        # アルゴリズム設定の取得
        config = self._get_algorithm_config(algorithm)
        
        # ハイパーパラメータ空間の定義
        if algorithm == "PPO":
            param_space = {
                "lr": tune.loguniform(1e-5, 1e-3),
                "train_batch_size": tune.choice([256, 512, 1024, 2048]),
                "num_sgd_iter": tune.choice([5, 10, 15, 20]),
                "sgd_minibatch_size": tune.choice([64, 128, 256]),
                "lambda": tune.uniform(0.9, 1.0),
                "kl_coeff": tune.uniform(0.1, 0.3),
            }
        elif algorithm == "SAC":
            param_space = {
                "learning_rate": tune.loguniform(1e-5, 1e-3),
                "train_batch_size": tune.choice([256, 512, 1024]),
                "tau": tune.uniform(0.001, 0.01),
                "target_network_update_freq": tune.choice([1, 2, 4]),
            }
        else:
            param_space = {}
            
        # チューニングの実行
        results = tune.run(
            algorithm,
            config=config,
            param_space=param_space,
            num_samples=num_samples,
            local_dir="./logs/tune_results",
            verbose=1
        )
        
        self.logger.info("Hyperparameter tuning completed")
        return results
        
    def profile_training(self, algorithm: str = "PPO", duration: int = 300):
        """学習のプロファイリング"""
        self.logger.info(f"Starting training profiling for {algorithm}")
        
        # プロファイリングツールの実行
        from tools.profiling.nsys_profile import NSysProfiler
        
        profiler = NSysProfiler()
        
        # 学習コマンドのプロファイリング
        profile_file = profiler.profile_ray_training(
            config_file=self.config_path,
            duration=duration,
            name=f"{algorithm}_training_profile"
        )
        
        if profile_file:
            # レポートの生成
            report_file = profiler.generate_report(profile_file)
            self.logger.info(f"Profiling completed: {profile_file}")
            if report_file:
                self.logger.info(f"Report generated: {report_file}")
                
        return profile_file

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description="High-Performance RL Training")
    parser.add_argument("--config", type=str, default="config/rl_config.yaml",
                       help="Configuration file path")
    parser.add_argument("--mode", type=str, default="train",
                       choices=["train", "evaluate", "tune", "profile"],
                       help="Training mode")
    parser.add_argument("--algorithm", type=str, default="PPO",
                       choices=["PPO", "SAC", "TD3", "DQN"],
                       help="RL algorithm")
    parser.add_argument("--iterations", type=int, default=1000,
                       help="Number of training iterations")
    parser.add_argument("--checkpoint", type=str,
                       help="Checkpoint path for evaluation")
    parser.add_argument("--samples", type=int, default=10,
                       help="Number of samples for hyperparameter tuning")
    parser.add_argument("--duration", type=int, default=300,
                       help="Profiling duration in seconds")
    
    args = parser.parse_args()
    
    # トレーナーの初期化
    trainer = HighPerformanceTrainer(args.config)
    
    # モードに応じた実行
    if args.mode == "train":
        results = trainer.train(args.algorithm, args.iterations)
        print(f"Training completed. Best checkpoint: {results.best_checkpoint}")
        
    elif args.mode == "evaluate":
        if not args.checkpoint:
            print("Error: Checkpoint path required for evaluation")
            return
        results = trainer.evaluate(args.checkpoint, args.algorithm)
        print(f"Evaluation completed: {results}")
        
    elif args.mode == "tune":
        results = trainer.hyperparameter_tuning(args.algorithm, args.samples)
        print(f"Tuning completed. Best config: {results.best_config}")
        
    elif args.mode == "profile":
        profile_file = trainer.profile_training(args.algorithm, args.duration)
        print(f"Profiling completed: {profile_file}")
        
    else:
        print(f"Unknown mode: {args.mode}")

if __name__ == "__main__":
    main() 