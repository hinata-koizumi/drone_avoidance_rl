#!/usr/bin/env bash
set -euxo pipefail

# rosdep初期化（既にされていればスキップ）
sudo rosdep init || true
rosdep update

# local rosdep yamlを追加
for yaml in rosdep/*.yaml; do
  fname=$(basename "$yaml")
  sudo cp "$yaml" "/etc/ros/rosdep/"
  echo "yaml file:///etc/ros/rosdep/$fname" | sudo tee "/etc/ros/rosdep/sources.list.d/10-local-$fname.list"
done

echo "[setup_rosdep_local] Local rosdep YAMLs added." 