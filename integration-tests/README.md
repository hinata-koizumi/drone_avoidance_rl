# drone-avoidance-integration

統合テスト専用リポジトリ（またはサブディレクトリ）です。 

- **目的**: `drone-msgs`、`drone-sim-core`、`drone-rl` の 3 つの独立リポジトリを組み合わせた E2E（エンドツーエンド）テストを実行する。
- **実行タイミング**: 手動 (`workflow_dispatch`) または週次スケジュール。CI 容量節約のためプルリクごとには走らせません。
- **前提**: 3 つのリポジトリをサブモジュールとして追加していること。

```bash
# 初回セットアップ
git submodule add https://github.com/your-org/drone-msgs.git external/drone-msgs
git submodule add https://github.com/your-org/drone-sim-core.git external/drone-sim-core
git submodule add https://github.com/your-org/drone-rl.git external/drone-rl

# 更新
git submodule update --remote --init --recursive
```

## フォルダ構成

```
integration-tests/
├── external/              # 3 つのサブモジュール
│   ├── drone-msgs/
│   ├── drone-sim-core/
│   └── drone-rl/
├── docker-compose.yml     # 統合テスト用 Compose
├── .github/workflows/
│   └── ci.yml             # 統合CI (E2E)
└── tests/
    └── test_e2e.py        # pytest などで実装
```

## 実行方法（ローカル）

```bash
# サブモジュール取得
git submodule update --init --recursive

# Docker イメージビルド
docker compose build

# 統合環境起動
docker compose up -d sim bridge rl-agent

# テストスクリプト実行
pytest tests/test_e2e.py -v
```

## 今後の拡張
- GPU ノードを含めた分散テスト
- Chaos Engineering による耐障害性テスト
- 長期安定稼働テスト (soak test) 