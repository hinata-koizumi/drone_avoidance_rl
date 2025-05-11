# ドローン回避RL Wiki（日本語）

[English version here](wiki_en.md)

---

## プロジェクト概要

- PX4 SITL + ROS 2 Humble + Gazebo Garden + RL (Gym API) 統合スタック
- 完全再現性: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden, multi-stage Docker
- CI/CD自動化: GitHub Actionsでビルド・テスト・静的解析・E2E・カバレッジ・セキュリティ・リリースノート自動生成
- カスタムモデル/エアフレーム差し替え対応
- 型安全・コード品質ゲート・ドキュメント自動生成

---

## ディレクトリ構成

（詳細はREADME.ja.md参照）

---

## 開発フロー
- ブランチ: main, feature/*, fix/*
- PRテンプレート・CONTRIBUTING.md必須
- すべてのコードはCI/CD（ビルド・テスト・Lint・静的解析）をパスすること

---

## カスタマイズ
- 報酬重み: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH`（`src/gym_env.py`参照）
- ドメインランダム化: `DroneSimEnv._randomize_world()`を拡張
- PX4パラメータ: `custom_airframes/`のJSON編集
- テレメトリ: UDP 14550をQGroundControl等に転送

---

## CI/CD・テスト
- GitHub Actions: ビルド・テスト・Lint・E2E・カバレッジ・セキュリティ・リリースノート自動化
- ローカルテスト例:
  ```bash
  tools/clean_workspace.sh
  docker build -t drone_rl:unified -f docker/Dockerfile.unified .
  docker run --rm drone_rl:unified ros2 launch sim_launch sim_all.launch.py
  python3 -m pip install pytest gymnasium numpy pyyaml lark
  cd src && PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
  python3 -m pip install ruff mypy
  ruff src/ tests/
  mypy src/ tests/
  ```

---

## ドキュメント
- 自動生成ドキュメント: [docs/](../docs/)
- mkdocs + GitHub Pages
- アーキテクチャ・開発フロー・トラブルシューティング・FAQ等

---

## FAQ
- Q. Apple Silicon(M1/M2)で動く？  
  A. arm64/x86_64両対応（公式バイナリ依存）
- Q. CIでGazeboが止まる場合は？  
  A. timeout/exit code監視済み。トラブルシューティング参照。

---

## ライセンス
Apache License 2.0 