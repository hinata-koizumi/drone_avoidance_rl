# ドローン手動制御環境

> **English**: [README.en.md](README.en.md)

この環境は、事前定義された行動をドローンに実行させるための手動制御システムです。
元の強化学習環境から再利用可能なコンポーネントを移行して構築されています。

## 専用ドローン機体情報

### Aether-SL

手動制御環境専用に設計された高性能ドローン機体です。

#### **基本仕様**
- **機体タイプ**: ストリームライン型クアッドコプター
- **重量**: 0.65kg
- **サイズ**: 32.5cm × 32.5cm × 7cm
- **最大速度**: 23.6 m/s（85 km/h）
- **最大高度**: 50m
- **飛行時間**: 約25分（シミュレーション）
- **最大航続距離**: 20km

#### **推進システム**
- **メインローター**: 4基（6インチプロペラ、最大11000rpm）
- **制御チャンネル**: 4チャンネル（4ローター）
- **推力定数**: 1.6（高効率設計）

#### **センサーシステム**
- **IMU**: 9軸（500Hz更新、高精度）
- **GPS**: デュアルバンドGNSS（10Hz更新）
- **バロメータ**: 高度計測（60Hz更新）
- **磁気センサ**: 3軸（50Hz更新）

#### **安全機能**
- **ジオフェンス**: 水平20km、垂直50m制限
- **フェイルセーフ**: RC信号喪失時の自動帰還
- **速度制限**: 水平23.6m/s、垂直8m/s
- **加速度制限**: 最大12m/s²

#### **制御パラメータ**
- **姿勢制御**: PID制御（Roll/Pitch: P=4.8, I=0.18, D=0.09）
- **位置制御**: PID制御（XY: P=1.0, I=0.1, D=0.05）
- **手動制御**: 傾斜角制限80%、ヨー角制限80%

#### **フライトモード**
- **STABILIZE**: 姿勢安定化モード
- **ALT_HOLD**: 高度保持モード
- **LOITER**: 位置保持モード
- **AUTO**: 自動飛行モード
- **RTL**: 自動帰還モード
- **MANUAL**: 手動制御モード

詳細仕様は `config/drone_specs.yaml` を参照してください。

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