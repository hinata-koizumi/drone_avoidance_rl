# README.ja.md（日本語版）

## 概要

* **ドローンモデルを差し込めば即動く** ― `custom_model/` と `custom_airframes/` を入れ替えるだけで新機体を学習対象にできる最小構成。
* **再現性の担保** ― Ubuntu 22.04 + ROS 2 Humble + Ignition Fortress + PX4‑SITL 1.15 を *日付固定スナップショット* で取得し、バージョン揺れを排除。
* **マルチステージ Docker** ― `px4‑simulator` / `px4‑bridge` / `rl‑agent` を分離し、SIM ↔ RL をホットリロード。Apple Silicon M‑series GPU (`--profile gpu`) にも対応。
* **最小 CI** ― GitHub Actions で `docker compose up` + `pytest` を 10 分以内に完走し、PR ごとに動作保証。

## 📘 [English version is available (README.md)](README.md)

## 📂 ディレクトリ構成

```
drone_avoidance_rl/
├── docker/            # 3 サービス用 Dockerfile & Entrypoint
├── src/               # ROS 2 ノード & Gym 環境
├── drone_msgs/        # カスタム ROS 2 メッセージ
├── custom_model/      # 差し替え SDF モデル
├── custom_airframes/  # PX4 airframe JSON
├── tests/             # pytest で最低限の結合テスト
└── logs/              # ulog / TensorBoard 等を自動保存
```

## ⚙️ 前提条件

* Docker Desktop ≥ 4.30（BuildKit 有効）
* 12 GB 以上の空き RAM
* macOS 12+  / Linux / Windows (WSL2)
* オプション: Apple Silicon GPU (`--profile gpu`) または NVIDIA CUDA 12

## 🚀 クイックスタート

```bash
# 1) Clone
$ git clone https://github.com/yourname/drone_avoidance_rl.git
$ cd drone_avoidance_rl

# 2) (任意) 機体モデル / エアフレームを差し替え
$ cp -r ~/my_drone_sdf      custom_model/drone_model
$ cp    ~/4500_my_drone.json custom_airframes/

# 3) Apple Silicon の場合は buildx を arm64 へ
$ docker buildx create --name arm_builder --driver docker-container --use || true

# 4) UDP 14556/11345 をホスト↔コンテナでフォワード
#    Docker Desktop › Settings › Resources › Networking で追加

# 5) ビルド & 起動
$ docker compose --profile cpu up -d --build          # CPU
$ docker compose --profile gpu up -d --build          # Apple GPU

# 6) 停止
$ docker compose down
```

*ログ/モデル/ulog は `logs/` に自動保存されます。*

## 🛠️ カスタマイズ

| 項目            | 方法                                                                             |
| ------------- | ------------------------------------------------------------------------------ |
| **報酬関数**      | 環境変数 `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` を変更<br>(`src/gym_env.py` の先頭) |
| **物理乱数**      | `DroneSimEnv._randomize_world()` を編集して風・質量などを追加変更                              |
| **PX4 パラメータ** | `custom_airframes/*.json` に追記し、`SYS_AUTOSTART` を合わせる                           |
| **テレメトリ**     | `docker-compose.yml` にポート 14550/udp を追記すると QGroundControl 接続可                  |

## 🧪 テスト & CI

```bash
# ローカルで結合テストを実行
$ docker compose exec rl-agent pytest -q /work/tests
```

CI ワークフローは `.drone.github/workflows/ci.yml` を参照。

## 📜 ライセンス

Apache License 2.0 — `LICENSE` を参照。

---

*貢献・Issue は大歓迎です！*
