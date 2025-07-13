# Drone Avoidance RL

**統合されたROS 2 + PX4 + Ignition Gazebo + 強化学習環境**

[![CI/CD](https://github.com/hinata-koizumi/drone_avoidance_rl/workflows/Optimized%20CI%2FCD%20Pipeline/badge.svg)](https://github.com/hinata-koizumi/drone_avoidance_rl/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-brightgreen.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-orange.svg)](https://gazebosim.org/)

## 概要

このプロジェクトは、**単一の統合環境**で以下を提供します：

- **PX4 SITL** + **Ignition Gazebo Garden** による高精度シミュレーション
- **強化学習環境** (SAC, PPO, DDPG対応)
- **手動制御インターフェース** (Web UI)
- **統合CI/CD** (自動テスト、パフォーマンス測定、セキュリティスキャン)
- **最適化されたパフォーマンス** (GPU対応、Fast DDS最適化)

## クイックスタート

### 1. 環境セットアップ

```bash
# リポジトリクローン
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
cd drone_avoidance_rl

# 環境設定
make setup
make build
```

### 2. シミュレーション実行

```bash
# シミュレーション環境起動
make sim

# 状態確認
make status
```

### 3. 強化学習開始

```bash
# GPU環境でRL学習開始
make train

# ログ確認
make logs
```

### 4. 手動制御

```bash
# 手動制御環境起動
make manual

# Web UI: http://localhost:8080
```

## 利用可能なコマンド

| コマンド | 説明 | 用途 |
|---------|------|------|
| `make build` | 全Dockerイメージをビルド | 初回セットアップ |
| `make sim` | シミュレーション環境起動 | 開発・テスト |
| `make train` | RL学習開始（GPU対応） | 強化学習 |
| `make manual` | 手動制御環境起動 | デバッグ・検証 |
| `make test` | 全テスト実行 | CI/CD |
| `make test-fast` | 軽量テスト実行 | 開発中 |
| `make test-gpu` | GPU環境テスト | パフォーマンス検証 |
| `make clean` | 全コンテナ・イメージ削除 | クリーンアップ |
| `make logs` | ログ表示 | デバッグ |
| `make status` | サービス状態確認 | 監視 |

## アーキテクチャ

### 統合Docker環境

```
┌─────────────────────────────────────────────────────────────┐
│                    Drone Avoidance RL                      │
├─────────────────────────────────────────────────────────────┤
│  Docker Compose (Profiles)                                 │
│  ├── default: 本番環境 (sim + bridge + manual-control)    │
│  ├── test: テスト環境 (軽量シミュレーション)               │
│  └── gpu: GPU学習環境 (sim + bridge + rl-agent)          │
├─────────────────────────────────────────────────────────────┤
│  最適化機能                                                │
│  ├── BuildKit キャッシュ (40-60%高速化)                   │
│  ├── Fast DDS QoS最適化 (20-30%通信効率向上)              │
│  ├── GPU対応 (NCCL最適化)                                 │
│  └── 並列ビルド・テスト                                    │
└─────────────────────────────────────────────────────────────┘
```

### サービス構成

| サービス | 役割 | プロファイル | 最適化 |
|---------|------|-------------|--------|
| `msgs` | ROS 2メッセージ定義 | 全プロファイル | キャッシュ共有 |
| `sim` | PX4 + Gazebo シミュレーション | default, test, gpu | ヘッドレス最適化 |
| `bridge` | ROS 2 ↔ Gazebo ブリッジ | default, test, gpu | Fast DDS最適化 |
| `rl-agent` | 強化学習エージェント | gpu | GPU対応 |
| `manual-control` | 手動制御Web UI | default | 軽量Webサーバー |

## パフォーマンス最適化

### ビルド最適化
- **BuildKit キャッシュ**: 40-60% ビルド時間短縮
- **並列ビルド**: マルチステージDockerfile
- **レイヤ共有**: 共通ベースイメージの再利用

### ランタイム最適化
- **Fast DDS QoS**: テスト環境は `BEST_EFFORT`、本番は `RELIABLE`
- **GPU最適化**: NCCL環境変数、CUDAメモリ管理
- **リソース制限**: メモリ・CPU使用量の最適化

### CI/CD最適化
- **並列テスト**: マトリックス戦略でテスト時間短縮
- **キャッシュ戦略**: Docker layer、pip、apt キャッシュ
- **セキュリティスキャン**: Trivyによる脆弱性検出

## テスト戦略

### テスト階層

```
┌─────────────────────────────────────────────────────────────┐
│                    テストピラミッド                          │
├─────────────────────────────────────────────────────────────┤
│  単体テスト (Unit Tests)                                   │
│  ├── Bridge Nodes テスト                                   │
│  ├── Gym API テスト                                       │
│  └── 共通ユーティリティテスト                              │
├─────────────────────────────────────────────────────────────┤
│  統合テスト (Integration Tests)                            │
│  ├── Docker Compose テスト                                 │
│  ├── ROS 2 通信テスト                                     │
│  └── Gazebo シミュレーションテスト                        │
├─────────────────────────────────────────────────────────────┤
│  E2Eテスト (End-to-End Tests)                             │
│  ├── 完全シミュレーション環境                             │
│  ├── RL学習フロー                                         │
│  └── 手動制御フロー                                       │
└─────────────────────────────────────────────────────────────┘
```

### テスト実行

```bash
# 全テスト実行
make test

# 軽量テスト（開発中）
make test-fast

# GPU環境テスト
make test-gpu

# パフォーマンステスト
make perf-test
```

## 監視・メトリクス

### パフォーマンス指標

| 指標 | 目標値 | 測定方法 |
|------|--------|----------|
| ビルド時間 | < 15分 | CI/CD パイプライン |
| テスト実行時間 | < 10分 | pytest + Docker |
| メモリ使用量 | < 4GB | docker stats |
| GPU使用率 | > 80% | nvidia-smi |
| 通信レイテンシ | < 10ms | Fast DDS QoS |

### ログ・監視

```bash
# リアルタイムログ
make logs

# サービス状態
make status

# パフォーマンス監視
docker stats
```

## 開発環境

### 前提条件

- **Docker**: 20.10+
- **Docker Compose**: 2.0+
- **GPU**: NVIDIA GPU (オプション)
- **メモリ**: 8GB+ (推奨16GB)
- **ストレージ**: 20GB+ 空き容量

### 開発ワークフロー

```bash
# 1. 環境セットアップ
make setup
make build

# 2. 開発・テスト
make sim          # シミュレーション起動
make test-fast    # 軽量テスト
make logs         # ログ確認

# 3. 強化学習
make train        # GPU学習開始
make test-gpu     # GPUテスト

# 4. クリーンアップ
make clean        # 環境クリーンアップ
```

## 詳細ドキュメント

- [**開発ガイド**](docs/development_guide.md) - 開発環境セットアップ
- [**API仕様**](docs/api_specification.md) - Gym API詳細
- [**CI/CD設計**](docs/ci_cd.md) - パイプライン詳細
- [**パフォーマンス最適化**](docs/performance_optimization.md) - 最適化手法
- [**トラブルシューティング**](docs/troubleshooting.md) - よくある問題と解決策

## コントリビューション

### 開発フロー

1. **Fork** リポジトリ
2. **Feature branch** 作成 (`git checkout -b feature/amazing-feature`)
3. **Commit** 変更 (`git commit -m 'Add amazing feature'`)
4. **Push** ブランチ (`git push origin feature/amazing-feature`)
5. **Pull Request** 作成

### 品質ゲート

- **静的解析**: ruff, mypy
- **テストカバレッジ**: 80%以上
- **CI/CD**: 全テスト通過
- **ドキュメント**: API仕様更新

## ライセンス

このプロジェクトは [Apache License 2.0](LICENSE) の下で公開されています。

## 謝辞

- [ROS 2](https://docs.ros.org/) - ロボティクスミドルウェア
- [PX4](https://px4.io/) - オープンソースフライトコントローラー
- [Ignition Gazebo](https://gazebosim.org/) - 物理シミュレーター
- [Stable Baselines3](https://stable-baselines3.readthedocs.io/) - 強化学習ライブラリ


