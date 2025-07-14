#!/bin/bash

# ディスク容量監視スクリプト
# GitHub Actionsでディスク容量不足を防ぐためのスクリプト

set -e

# 設定
THRESHOLD_GB=5  # 5GB未満で警告
CRITICAL_GB=2   # 2GB未満で緊急クリーンアップ

echo "=== Disk Monitor Started ==="
echo "Threshold: ${THRESHOLD_GB}GB, Critical: ${CRITICAL_GB}GB"

# ディスク使用量を取得
get_disk_usage() {
    df / | tail -1 | awk '{print $4}' | sed 's/[^0-9]//g'
}

# ディスク使用量を表示
show_disk_usage() {
    echo "=== Current Disk Usage ==="
    df -h
    echo "=== Docker System Usage ==="
    docker system df || echo "Docker not available"
}

# 基本クリーンアップ
basic_cleanup() {
    echo "=== Performing Basic Cleanup ==="
    docker system prune -f || true
    docker volume prune -f || true
    docker builder prune -f || true
    sudo rm -rf /tmp/* /var/tmp/* /root/.cache/* || true
    sudo apt-get clean || true
    sudo apt-get autoremove -y || true
}

# 緊急クリーンアップ
emergency_cleanup() {
    echo "=== Performing Emergency Cleanup ==="
    docker system prune -af || true
    docker volume prune -f || true
    docker builder prune -af || true
    docker rmi $(docker images -q) || true
    sudo rm -rf /tmp/* /var/tmp/* /root/.cache/* || true
    sudo apt-get clean || true
    sudo apt-get autoremove -y || true
    sudo find /var/log -type f -name "*.log" -delete || true
    sudo find /var/cache -type f -delete || true
    sudo journalctl --vacuum-time=1s || true
    sudo journalctl --vacuum-size=1M || true
}

# メイン処理
main() {
    show_disk_usage
    
    # 利用可能容量を取得（KB単位）
    available_kb=$(get_disk_usage)
    available_gb=$((available_kb / 1024 / 1024))
    
    echo "Available disk space: ${available_gb}GB"
    
    if [ "$available_gb" -lt "$CRITICAL_GB" ]; then
        echo "CRITICAL: Very low disk space (${available_gb}GB < ${CRITICAL_GB}GB)"
        emergency_cleanup
        show_disk_usage
    elif [ "$available_gb" -lt "$THRESHOLD_GB" ]; then
        echo "WARNING: Low disk space (${available_gb}GB < ${THRESHOLD_GB}GB)"
        basic_cleanup
        show_disk_usage
    else
        echo "OK: Sufficient disk space (${available_gb}GB >= ${THRESHOLD_GB}GB)"
    fi
}

# スクリプト実行
main "$@" 