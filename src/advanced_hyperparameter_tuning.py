#!/usr/bin/env python3
"""
Optuna統合の高度なハイパーパラメータチューニング
"""

import optuna
import yaml
import logging
from typing import Dict, Any
import ray
from ray import tune
from ray.tune.schedulers import PopulationBasedTraining
from ray.tune.search.optuna import OptunaSearch

class AdvancedHyperparameterTuner:
    """高度なハイパーパラメータチューニングクラス"""
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self._load_config()
        self.logger = self._setup_logger()
        
    def _load_config(self) -> Dict[str, Any]:
        """設定ファイルの読み込み"""
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)
            
    def _setup_logger(self) -> logging.Logger:
        """ロガーの設定"""
        logger = logging.getLogger('advanced_hyperparameter_tuner')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            
        return logger
        
    def create_optuna_search_space(self, algorithm: str) -> Dict[str, Any]:
        """Optuna検索空間の作成"""
        
        if algorithm == "PPO":
            return {
                "lr": tune.loguniform(1e-5, 1e-3),
                "train_batch_size": tune.choice([256, 512, 1024, 2048]),
                "num_sgd_iter": tune.choice([5, 10, 15, 20, 30]),
                "sgd_minibatch_size": tune.choice([32, 64, 128, 256]),
                "lambda": tune.uniform(0.9, 1.0),
                "kl_coeff": tune.uniform(0.1, 0.3),
                "clip_param": tune.uniform(0.1, 0.3),
                "vf_clip_param": tune.uniform(5.0, 15.0),
                "entropy_coeff": tune.loguniform(1e-4, 1e-2),
                "vf_loss_coeff": tune.uniform(0.3, 0.7),
            }
            
        elif algorithm == "SAC":
            return {
                "learning_rate": tune.loguniform(1e-5, 1e-3),
                "train_batch_size": tune.choice([256, 512, 1024]),
                "tau": tune.uniform(0.001, 0.01),
                "target_network_update_freq": tune.choice([1, 2, 4]),
                "initial_alpha": tune.loguniform(1e-3, 1e-1),
                "target_entropy": tune.uniform(-10, -1),
                "buffer_size": tune.choice([100000, 500000, 1000000]),
                "learning_starts": tune.choice([100, 500, 1000]),
            }
            
        elif algorithm == "TD3":
            return {
                "learning_rate": tune.loguniform(1e-5, 1e-3),
                "train_batch_size": tune.choice([256, 512, 1024]),
                "tau": tune.uniform(0.001, 0.01),
                "policy_delay": tune.choice([1, 2, 4]),
                "noise_clip": tune.uniform(0.1, 0.5),
                "noise": tune.uniform(0.1, 0.5),
                "buffer_size": tune.choice([100000, 500000, 1000000]),
                "learning_starts": tune.choice([100, 500, 1000]),
            }
            
        else:
            return {}
            
    def create_pbt_scheduler(self) -> PopulationBasedTraining:
        """PBTスケジューラーの作成"""
        return PopulationBasedTraining(
            time_attr="training_iteration",
            metric="episode_reward_mean",
            mode="max",
            perturbation_interval=10,
            resample_probability=0.25,
            quantile_fraction=0.25,
            hyperparam_mutations={
                "lr": tune.loguniform(1e-5, 1e-3),
                "train_batch_size": [256, 512, 1024, 2048],
                "num_sgd_iter": [5, 10, 15, 20, 30],
            }
        )
        
    def run_optuna_tuning(self, algorithm: str = "PPO", num_trials: int = 50):
        """Optunaチューニングの実行"""
        self.logger.info(f"Starting Optuna tuning for {algorithm}")
        
        # 検索空間の作成
        search_space = self.create_optuna_search_space(algorithm)
        
        # Optuna検索アルゴリズム
        search_alg = OptunaSearch(
            metric="episode_reward_mean",
            mode="max",
            points_to_evaluate=[],
            evaluated_rewards=[],
        )
        
        # PBTスケジューラー
        scheduler = self.create_pbt_scheduler()
        
        # チューニングの実行
        results = tune.run(
            algorithm,
            config=search_space,
            search_alg=search_alg,
            scheduler=scheduler,
            num_samples=num_trials,
            local_dir="./logs/optuna_results",
            verbose=1,
            stop={"training_iteration": 100},
            checkpoint_freq=10,
            keep_checkpoints_num=3,
        )
        
        self.logger.info("Optuna tuning completed")
        return results
        
    def run_bayesian_optimization(self, algorithm: str = "PPO", num_trials: int = 30):
        """ベイズ最適化の実行"""
        self.logger.info(f"Starting Bayesian optimization for {algorithm}")
        
        # 検索空間の作成
        search_space = self.create_optuna_search_space(algorithm)
        
        # ベイズ最適化用のOptuna設定
        search_alg = OptunaSearch(
            metric="episode_reward_mean",
            mode="max",
            sampler=optuna.samplers.TPESampler(seed=42),
        )
        
        # チューニングの実行
        results = tune.run(
            algorithm,
            config=search_space,
            search_alg=search_alg,
            num_samples=num_trials,
            local_dir="./logs/bayesian_results",
            verbose=1,
            stop={"training_iteration": 100},
        )
        
        self.logger.info("Bayesian optimization completed")
        return results
        
    def run_multi_objective_optimization(self, algorithm: str = "PPO", num_trials: int = 40):
        """多目的最適化の実行"""
        self.logger.info(f"Starting multi-objective optimization for {algorithm}")
        
        # 検索空間の作成
        search_space = self.create_optuna_search_space(algorithm)
        
        # 多目的最適化用のOptuna設定
        search_alg = OptunaSearch(
            metric=["episode_reward_mean", "episode_len_mean"],
            mode=["max", "min"],
            sampler=optuna.samplers.NSGAIISampler(seed=42),
        )
        
        # チューニングの実行
        results = tune.run(
            algorithm,
            config=search_space,
            search_alg=search_alg,
            num_samples=num_trials,
            local_dir="./logs/multi_objective_results",
            verbose=1,
            stop={"training_iteration": 100},
        )
        
        self.logger.info("Multi-objective optimization completed")
        return results
        
    def analyze_results(self, results):
        """結果の分析"""
        self.logger.info("Analyzing optimization results...")
        
        # 最良の結果を取得
        best_trial = results.get_best_trial("episode_reward_mean", "max")
        
        self.logger.info(f"Best trial: {best_trial}")
        self.logger.info(f"Best config: {best_trial.config}")
        self.logger.info(f"Best reward: {best_trial.last_result['episode_reward_mean']}")
        
        # 結果を保存
        with open("./logs/optimization_results.yaml", "w") as f:
            yaml.dump({
                "best_trial": best_trial.config,
                "best_reward": best_trial.last_result["episode_reward_mean"],
                "all_trials": [trial.config for trial in results.trials]
            }, f)
            
        return best_trial

def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Advanced Hyperparameter Tuning")
    parser.add_argument("--config", type=str, default="config/rl_config.yaml",
                       help="Configuration file path")
    parser.add_argument("--algorithm", type=str, default="PPO",
                       choices=["PPO", "SAC", "TD3"],
                       help="RL algorithm")
    parser.add_argument("--method", type=str, default="optuna",
                       choices=["optuna", "bayesian", "multi_objective", "pbt"],
                       help="Optimization method")
    parser.add_argument("--trials", type=int, default=50,
                       help="Number of trials")
    
    args = parser.parse_args()
    
    # チューナーの初期化
    tuner = AdvancedHyperparameterTuner(args.config)
    
    # メソッドに応じた実行
    if args.method == "optuna":
        results = tuner.run_optuna_tuning(args.algorithm, args.trials)
    elif args.method == "bayesian":
        results = tuner.run_bayesian_optimization(args.algorithm, args.trials)
    elif args.method == "multi_objective":
        results = tuner.run_multi_objective_optimization(args.algorithm, args.trials)
    else:
        print(f"Unknown method: {args.method}")
        return
        
    # 結果の分析
    best_trial = tuner.analyze_results(results)
    print(f"Best configuration: {best_trial.config}")

if __name__ == "__main__":
    main() 