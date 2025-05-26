# custom_model/drone_model

## 概要
災害救助用クアッド＋デュアルダクトファンドローンのSDFモデルです。

- 4つのローター（rotor_1〜4）
- 2つのダクトファン（ducted_fan_1, 2）
- PX4による姿勢制御＋ダクトファンによる微調整・排熱

## ファイル構成
- `model.sdf`: 本体・ローター・ダクトファンの物理・ビジュアル・コリジョン定義
- `model.config`: モデルメタ情報（著者・バージョン・説明）

## 設計意図・拡張方針
- **拡張性重視**: センサ追加、可動ジョイント化、プラグイン追加など容易
- **Ignition Gazebo (Garden) 対応**: SDF 1.9形式
- **パラメータ外部化**: 今後YAML等でのパラメータ管理も検討

## CI/CD・テスト例
- SDF/ConfigのバリデーションをCIで自動実行
- 例: GitHub Actionsでign sdf -kやxmllintによる構文チェック

```yaml
# .github/workflows/model_validation.yaml
name: Validate SDF Model
on: [push, pull_request]
jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Install Ignition Tools
        run: sudo apt-get update && sudo apt-get install -y ignition-tools
      - name: Validate SDF
        run: ign sdf -k custom_model/drone_model/model.sdf
      - name: Validate XML
        run: xmllint --noout custom_model/drone_model/model.config
```

## 著者
- Koizumi Hinata (hkoizumi1123@gmail.com) 