# CI/CD設計・再現・トラブルシュート

本プロジェクトのCI/CDは、再現性・自動化・依存性一貫性を重視して設計されています。

## CI/CD全体像
- **GitHub Actions** で全自動ビルド・テスト・静的解析・E2E・カバレッジ・セキュリティ・リリースノート生成
- **マルチステージDocker**で本番・テスト環境を完全再現
- **ローカル再現手順**もCI/CDと同一

## 主なCIジョブ
- **build-msgs/bridge/sim**: 各レイヤーのDockerイメージをmulti-archでビルド
- **lint/static-analysis**: ruff, mypy, ament_lint_auto
- **e2e**: docker composeによる統合テスト
- **rosdep-check**: rosdep依存解決の自動検証（ローカルYAML対応）
- **check-package-versions**: package.xmlバージョン一貫性チェック
- **check-tag-version**: タグリリース時にpackage.xmlバージョンとタグの一致を自動検証
- **gym-api-test**: Gym API互換性テスト
- **validate-model**: SDF/Configの構文・仕様チェック
- **coverage/security-scan**: カバレッジ・脆弱性スキャン

## バージョン・依存性一貫性の自動チェック
- **package.xmlバージョン**: `check_package_versions.sh` で全package.xmlのバージョン一貫性を自動チェック
- **タグ連携**: タグリリース時にpackage.xmlバージョンとタグの一致を自動検証（不一致ならCI失敗）
- **rosdep/requirements.txt依存性**: Dependabot（`.github/dependabot.yml`）で自動監視・PR作成
- **rosdep YAML**: `tools/check_rosdep_consistency.py` でローカルYAMLの一貫性をCIで自動検証

## ローカルでCI/CDと同じテストを再現する手順
1. Dockerとdocker composeでビルド・テスト
   ```sh
   docker compose build --no-cache
   bash tools/setup_rosdep_local.sh
   docker compose -f tests/ci-compose.yml up --abort-on-container-exit
   ```
2. Gym APIテストや静的解析もCI/CDと同じコマンドで実行
   ```sh
   python3 -m pip install --upgrade pip
   python3 -m pip install pytest gymnasium numpy pyyaml lark ruff mypy
   cd src
   PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
   ruff src/ tests/
   mypy src/ tests/
   ```
3. rosdep依存解決は必ずlocal rosdep yamlを反映
   ```sh
   bash tools/setup_rosdep_local.sh
   ```
4. PythonバージョンはCIと同じ3.10を推奨
   ```sh
   pyenv install 3.10.12
   pyenv local 3.10.12
   ```

## トラブルシュート
- **rosdep update失敗**: ネットワークやYAMLの記述ミスを確認
- **バージョン不一致でCI失敗**: package.xmlやタグ名を修正し再push
- **依存性PRが自動で来ない**: `.github/dependabot.yml`の記述や権限を確認
- **Docker build失敗**: .envやサブモジュール初期化漏れに注意

## 参考
- `.github/workflows/ci.yml`（全体CI定義）
- `check_package_versions.sh`（バージョン一貫性スクリプト）
- `tools/check_rosdep_consistency.py`（rosdep YAML一貫性）
- `.github/dependabot.yml`（依存性自動監視） 