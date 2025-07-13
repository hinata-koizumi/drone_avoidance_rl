#!/usr/bin/env python3
"""
NVIDIA Nsight Systems プロファイリングツール
"""

import subprocess
import os
import time
import argparse
from typing import List, Optional

class NSysProfiler:
    """NVIDIA Nsight Systems プロファイラー"""
    
    def __init__(self, output_dir: str = "profiles"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
    def check_nsys_available(self) -> bool:
        """nsysコマンドの可用性チェック"""
        try:
            result = subprocess.run(['nsys', '--version'], 
                                  capture_output=True, text=True)
            return result.returncode == 0
        except FileNotFoundError:
            return False
            
    def profile_command(self, 
                       command: List[str],
                       duration: Optional[float] = None,
                       name: str = "profile",
                       additional_args: List[str] = None) -> str:
        """コマンドのプロファイリング実行"""
        
        if not self.check_nsys_available():
            raise RuntimeError("nsys command not found. Install NVIDIA Nsight Systems.")
            
        # 出力ファイル名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(self.output_dir, f"{name}_{timestamp}")
        
        # nsysコマンドの構築
        nsys_cmd = [
            "nsys", "profile",
            "-o", output_file,
            "--stats=true",
            "--force-overwrite=true"
        ]
        
        # 追加引数
        if additional_args:
            nsys_cmd.extend(additional_args)
            
        # 実行時間制限
        if duration:
            nsys_cmd.extend(["--duration", str(duration)])
            
        # 対象コマンド
        nsys_cmd.extend(["--"] + command)
        
        print(f"Running: {' '.join(nsys_cmd)}")
        
        # プロファイリング実行
        try:
            result = subprocess.run(nsys_cmd, 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=duration + 30 if duration else None)
            
            if result.returncode != 0:
                print(f"Profile command failed: {result.stderr}")
                return None
                
            print(f"Profile completed: {output_file}")
            return output_file
            
        except subprocess.TimeoutExpired:
            print("Profile command timed out")
            return None
        except Exception as e:
            print(f"Profile error: {e}")
            return None
            
    def profile_python_script(self,
                             script_path: str,
                             duration: Optional[float] = None,
                             name: str = "python_profile",
                             additional_args: List[str] = None) -> str:
        """Pythonスクリプトのプロファイリング"""
        
        cmd = ["python3", script_path]
        return self.profile_command(cmd, duration, name, additional_args)
        
    def profile_ray_training(self,
                            config_file: str,
                            duration: Optional[float] = None,
                            name: str = "ray_training") -> str:
        """Ray分散学習のプロファイリング"""
        
        cmd = ["python3", "-m", "ray.rllib.train", 
               "--config", config_file]
        
        additional_args = [
            "--capture-range=cuda",
            "--capture-range=osrt"
        ]
        
        return self.profile_command(cmd, duration, name, additional_args)
        
    def generate_report(self, profile_file: str) -> str:
        """プロファイリングレポートの生成"""
        
        if not profile_file or not os.path.exists(profile_file):
            return None
            
        # レポートファイル名
        report_file = profile_file + "_report.txt"
        
        # レポート生成コマンド
        report_cmd = [
            "nsys", "stats",
            "--report", "gputrace",
            "--report", "cudaapisum",
            "--report", "osrt",
            profile_file
        ]
        
        try:
            with open(report_file, 'w') as f:
                result = subprocess.run(report_cmd, 
                                      stdout=f, 
                                      stderr=subprocess.PIPE,
                                      text=True)
                                      
            if result.returncode == 0:
                print(f"Report generated: {report_file}")
                return report_file
            else:
                print(f"Report generation failed: {result.stderr}")
                return None
                
        except Exception as e:
            print(f"Report generation error: {e}")
            return None

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description="NVIDIA Nsight Systems Profiler")
    parser.add_argument("--command", nargs="+", help="Command to profile")
    parser.add_argument("--script", type=str, help="Python script to profile")
    parser.add_argument("--ray-config", type=str, help="Ray config file to profile")
    parser.add_argument("--duration", type=float, help="Profile duration in seconds")
    parser.add_argument("--name", type=str, default="profile", help="Profile name")
    parser.add_argument("--output-dir", type=str, default="profiles", help="Output directory")
    parser.add_argument("--generate-report", action="store_true", help="Generate report")
    
    args = parser.parse_args()
    
    profiler = NSysProfiler(args.output_dir)
    
    profile_file = None
    
    if args.command:
        profile_file = profiler.profile_command(args.command, args.duration, args.name)
    elif args.script:
        profile_file = profiler.profile_python_script(args.script, args.duration, args.name)
    elif args.ray_config:
        profile_file = profiler.profile_ray_training(args.ray_config, args.duration, args.name)
    else:
        print("Please specify --command, --script, or --ray-config")
        return
        
    if args.generate_report and profile_file:
        profiler.generate_report(profile_file)

if __name__ == "__main__":
    main() 