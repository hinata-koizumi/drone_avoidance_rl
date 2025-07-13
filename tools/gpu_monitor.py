#!/usr/bin/env python3
"""
GPU監視・プロファイリングツール
"""

import time
import psutil
import subprocess
import json
from datetime import datetime
from typing import Dict, List, Optional
import logging

try:
    import pynvml
    NVML_AVAILABLE = True
except ImportError:
    NVML_AVAILABLE = False
    print("Warning: pynvml not available. Install with: pip install nvidia-ml-py3")

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

class GPUMonitor:
    """GPU監視クラス"""
    
    def __init__(self, log_interval: float = 1.0):
        self.log_interval = log_interval
        self.logger = self._setup_logger()
        
        if NVML_AVAILABLE:
            pynvml.nvmlInit()
            self.device_count = pynvml.nvmlDeviceGetCount()
        else:
            self.device_count = 0
            
    def _setup_logger(self) -> logging.Logger:
        """ロガーの設定"""
        logger = logging.getLogger('gpu_monitor')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            
        return logger
        
    def get_gpu_info(self) -> Dict:
        """GPU情報の取得"""
        if not NVML_AVAILABLE:
            return {"error": "NVML not available"}
            
        gpu_info = {}
        
        for i in range(self.device_count):
            handle = pynvml.nvmlDeviceGetHandleByIndex(i)
            
            # 基本情報
            name = pynvml.nvmlDeviceGetName(handle).decode('utf-8')
            memory_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
            utilization = pynvml.nvmlDeviceGetUtilizationRates(handle)
            temperature = pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU)
            
            # 電力情報
            try:
                power = pynvml.nvmlDeviceGetPowerUsage(handle) / 1000.0  # mW to W
            except:
                power = 0
                
            gpu_info[f"gpu_{i}"] = {
                "name": name,
                "memory_total": memory_info.total,
                "memory_used": memory_info.used,
                "memory_free": memory_info.free,
                "memory_utilization": (memory_info.used / memory_info.total) * 100,
                "gpu_utilization": utilization.gpu,
                "memory_utilization_rate": utilization.memory,
                "temperature": temperature,
                "power_usage": power
            }
            
        return gpu_info
        
    def get_system_info(self) -> Dict:
        """システム情報の取得"""
        return {
            "cpu_percent": psutil.cpu_percent(interval=1),
            "memory_percent": psutil.virtual_memory().percent,
            "disk_percent": psutil.disk_usage('/').percent,
            "timestamp": datetime.now().isoformat()
        }
        
    def get_torch_gpu_info(self) -> Dict:
        """PyTorch GPU情報の取得"""
        if not TORCH_AVAILABLE:
            return {"error": "PyTorch not available"}
            
        torch_info = {}
        
        if torch.cuda.is_available():
            torch_info["cuda_available"] = True
            torch_info["device_count"] = torch.cuda.device_count()
            torch_info["current_device"] = torch.cuda.current_device()
            torch_info["device_name"] = torch.cuda.get_device_name()
            
            # メモリ情報
            torch_info["memory_allocated"] = torch.cuda.memory_allocated()
            torch_info["memory_reserved"] = torch.cuda.memory_reserved()
            torch_info["memory_cached"] = torch.cuda.memory_reserved()
            
        else:
            torch_info["cuda_available"] = False
            
        return torch_info
        
    def monitor_loop(self, duration: Optional[float] = None):
        """監視ループ"""
        start_time = time.time()
        
        self.logger.info("Starting GPU monitoring...")
        
        try:
            while True:
                # GPU情報取得
                gpu_info = self.get_gpu_info()
                system_info = self.get_system_info()
                torch_info = self.get_torch_gpu_info()
                
                # ログ出力
                self.logger.info(f"System: CPU={system_info['cpu_percent']:.1f}%, "
                               f"Memory={system_info['memory_percent']:.1f}%")
                
                for gpu_id, info in gpu_info.items():
                    if "error" not in info:
                        self.logger.info(
                            f"{gpu_id}: GPU={info['gpu_utilization']}%, "
                            f"Memory={info['memory_utilization']:.1f}%, "
                            f"Temp={info['temperature']}°C, "
                            f"Power={info['power_usage']:.1f}W"
                        )
                
                # 終了条件チェック
                if duration and (time.time() - start_time) > duration:
                    break
                    
                time.sleep(self.log_interval)
                
        except KeyboardInterrupt:
            self.logger.info("Monitoring stopped by user")
        except Exception as e:
            self.logger.error(f"Monitoring error: {e}")
            
    def export_metrics(self, output_file: str):
        """メトリクスのエクスポート"""
        metrics = {
            "gpu": self.get_gpu_info(),
            "system": self.get_system_info(),
            "torch": self.get_torch_gpu_info()
        }
        
        with open(output_file, 'w') as f:
            json.dump(metrics, f, indent=2)
            
        self.logger.info(f"Metrics exported to {output_file}")

def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="GPU Monitoring Tool")
    parser.add_argument("--duration", type=float, help="Monitoring duration in seconds")
    parser.add_argument("--interval", type=float, default=1.0, help="Log interval in seconds")
    parser.add_argument("--export", type=str, help="Export metrics to file")
    
    args = parser.parse_args()
    
    monitor = GPUMonitor(log_interval=args.interval)
    
    if args.export:
        monitor.export_metrics(args.export)
    else:
        monitor.monitor_loop(duration=args.duration)

if __name__ == "__main__":
    main() 