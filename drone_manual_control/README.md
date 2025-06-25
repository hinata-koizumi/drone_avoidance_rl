# Drone Manual Control Environment

この環境は、事前定義された行動をドローンに実行させるための手動制御システムです。
元の強化学習環境から再利用可能なコンポーネントを移行して構築されています。

## 特徴

- **事前定義行動実行**: ホバリング、離陸、着陸、軌道追従などの基本動作
- **ROS 2 Humble対応**: 最新のROS 2フレームワークを使用
- **Ignition Gazebo (Garden)**: 高精度な物理シミュレーション
- **Docker統合**: 再現可能な開発環境
- **モジュラー設計**: 既存のブリッジコンポーネントを再利用

## 移行されたコンポーネント

### 基盤コンポーネント
- `common/` - BridgeBaseクラスとユーティリティ
- `drone_msgs/` - カスタムメッセージ定義
- `px4_msgs/` - PX4メッセージ定義

### シミュレーション環境
- `sim_launch/` - Gazebo Sim起動設定
- `models/` - ドローンモデル
- `custom_airframes/` - エアフレーム設定

### ブリッジノード
- `command_bridge/` - 制御コマンド変換
- `state_bridge/` - 状態情報変換
- `angle_bridge/` - 角度制御
- `outer_motor_bridge/` - 外部モーター制御

## 新規追加コンポーネント

### 手動制御システム
- `manual_control/` - 事前定義行動実行ノード
- `action_sequences/` - 行動シーケンス定義
- `control_interface/` - 制御インターフェース

## セットアップ手順

### 1. 環境の初期化
```bash
# 既存環境からコンポーネントをコピー
./scripts/setup_environment.sh
```

### 2. 環境のビルド
```bash
# Dockerコンテナをビルド
docker-compose build
```

### 3. デモの実行
```bash
# 自動デモ実行
./scripts/run_demo.sh
```

### 4. 手動実行（オプション）
```bash
# シミュレーション起動
docker-compose up -d simulator

# ブリッジノード起動
docker-compose up -d bridge

# 手動制御ノード起動
docker-compose up -d manual_control
```

## 使用方法

### 基本的な使用方法
1. **環境構築**
```bash
cd drone_manual_control
./scripts/setup_environment.sh
docker-compose build
```

2. **シミュレーション起動**
```bash
docker-compose up -d
```

3. **手動制御実行**
```bash
docker-compose up -d manual_control
```

### ログの確認
```bash
# 手動制御ノードのログ
docker-compose logs -f manual_control

# 全ノードのログ
docker-compose logs -f
```

### 環境の停止
```bash
docker-compose down
```

## 事前定義行動

### 基本動作
- **Hover**: ホバリング維持（10秒間）
- **Takeoff**: 離陸シーケンス（5秒間）
- **Landing**: 着陸シーケンス（8秒間）

### 移動動作
- **Waypoint Forward**: 前方5m移動（15秒間）
- **Waypoint Backward**: 後方5m移動（15秒間）
- **Waypoint Right**: 右方5m移動（12秒間）
- **Waypoint Left**: 左方5m移動（12秒間）

### パターン飛行
- **Circle Flight**: 円形飛行（半径5m、20秒間）
- **Square Pattern**: 四角形パターン飛行（40秒間）

### 複合シーケンス
- **Takeoff and Hover**: 離陸→ホバリング
- **Exploration Sequence**: 離陸→前方移動
- **Return to Base**: 基地帰還→着陸

## 設定

### 行動シーケンスの変更
`config/action_sequences.yaml` を編集して行動をカスタマイズ：

```yaml
action_sequences:
  - name: "custom_hover"
    action_type: "hover"
    duration: 15.0  # 15秒間
    parameters:
      target_altitude: 5.0  # 5m高度
    next_action: "landing"  # 次の行動
```

### 制御パラメータの調整
```yaml
control_parameters:
  position_p_gain: 1.0
  position_i_gain: 0.1
  position_d_gain: 0.05
  max_velocity: 5.0  # m/s
```

### 安全設定
```yaml
safety_parameters:
  min_altitude: 0.5  # m
  max_altitude: 50.0  # m
  max_distance_from_base: 100.0  # m
```

## トラブルシューティング

### よくある問題

1. **シミュレーションが起動しない**
   ```bash
   # X11ディスプレイの確認
   echo $DISPLAY
   
   # 権限の確認
   xhost +local:docker
   ```

2. **ブリッジノードが接続できない**
   ```bash
   # ネットワークの確認
   docker network ls
   
   # ログの確認
   docker-compose logs bridge
   ```

3. **手動制御が動作しない**
   ```bash
   # 設定ファイルの確認
   cat config/action_sequences.yaml
   
   # ノードの再起動
   docker-compose restart manual_control
   ```

### デバッグ方法

1. **個別ノードのログ確認**
   ```bash
   docker-compose logs -f [service_name]
   ```

2. **ROS 2トピックの確認**
   ```bash
   docker-compose exec manual_control ros2 topic list
   docker-compose exec manual_control ros2 topic echo /drone/control_command
   ```

3. **ノードの状態確認**
   ```bash
   docker-compose exec manual_control ros2 node list
   docker-compose exec manual_control ros2 node info /action_executor_node
   ```

## 開発者向け情報

### 新しい行動の追加
1. `ActionType` 列挙型に新しい行動を追加
2. `_generate_command()` メソッドに制御ロジックを実装
3. `action_sequences.yaml` に設定を追加

### カスタムブリッジの追加
1. 既存のブリッジノードを参考に新しいノードを作成
2. `docker-compose.yml` にサービスを追加
3. 依存関係を適切に設定

### テストの実行
```bash
# 単体テスト
docker-compose exec manual_control python3 -m pytest

# 統合テスト
./scripts/test_integration.sh
```

## ライセンス

MIT License

## 貢献

プルリクエストやイシューの報告を歓迎します。 