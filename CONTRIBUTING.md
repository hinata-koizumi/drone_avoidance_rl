# Contributing Guide

## コントリビュートの流れ
1. Issue作成・相談
2. フォーク＆ブランチ作成
3. PR作成（テンプレート必須）
4. CI/CDパス・レビュー

## コード品質
- ruff, mypy, ament_lint_auto必須
- テスト・ドキュメント必須

## サブモジュールの初期化について

このリポジトリは src/px4_msgs をサブモジュールとして利用しています。
クローン直後やpull直後、docker build/CI/CD前には必ず以下を実行してください:

```sh
git submodule update --init --recursive
```

これを怠るとビルドやテストが失敗します。 