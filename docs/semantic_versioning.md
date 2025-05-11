# Semantic Versioning 運用ルール

- すべてのpackage.xmlは同一バージョンを維持
- タグはvMAJOR.MINOR.PATCH形式
- タグpush時にCIでバージョン一致を自動チェック
- 破壊的変更はMAJOR、後方互換追加はMINOR、バグ修正はPATCH 