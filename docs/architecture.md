# Architecture & Project Strategy

## 概要
このプロジェクトはROS 2 Humble, PX4, Gazebo Garden, Docker, RL(Gym API)を統合したドローン自律制御・回避RLスタックです。

## 主要構成
- ROS 2 Humble
- PX4 Autopilot
- Gazebo Garden (Ignition)
- Docker (multi-stage)
- OpenAI Gym API
- CI/CD (GitHub Actions)

## 運用・開発ルール
- Semantic Versioning
- CI/CD自動化
- ドキュメント自動生成
- コード品質ゲート
- E2Eテスト・可観測性

## 目標
- 高い再現性・保守性・拡張性
- OSS/チーム開発のしやすさ
- RL/制御/シミュレーションの一体運用 