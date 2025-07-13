# Drone Avoidance RL - Makefile
# 統一された開発・テスト・実行コマンド

.PHONY: help build test sim train manual clean logs gpu-test

# デフォルトターゲット
help:
	@echo "Drone Avoidance RL - 利用可能なコマンド:"
	@echo ""
	@echo "開発コマンド:"
	@echo "  make build     - 全Dockerイメージをビルド"
	@echo "  make sim       - シミュレーション環境を起動"
	@echo "  make train     - RL学習を開始（GPU対応）"
	@echo "  make manual    - 手動制御環境を起動"
	@echo ""
	@echo "テストコマンド:"
	@echo "  make test      - 全テストを実行"
	@echo "  make test-fast - 軽量テストを実行"
	@echo "  make test-gpu  - GPU環境テストを実行"
	@echo ""
	@echo "管理コマンド:"
	@echo "  make clean     - 全コンテナ・イメージを削除"
	@echo "  make logs      - ログを表示"
	@echo "  make status    - サービス状態を確認"
	@echo ""

# ビルド
build:
	@echo "🔨 Dockerイメージをビルド中..."
	docker compose build --no-cache
	@echo "✅ ビルド完了"

# シミュレーション環境
sim:
	@echo "🚁 シミュレーション環境を起動中..."
	docker compose --profile default up -d sim bridge
	@echo "✅ シミュレーション環境起動完了"
	@echo "📊 状態確認: make status"

# RL学習
train:
	@echo "🧠 RL学習を開始中..."
	docker compose --profile gpu up -d
	@echo "✅ RL学習開始"
	@echo "📊 ログ確認: make logs"

# 手動制御
manual:
	@echo "🎮 手動制御環境を起動中..."
	docker compose --profile default up -d manual-control
	@echo "✅ 手動制御環境起動完了"
	@echo "🌐 Web UI: http://localhost:8080"

# テスト実行
test:
	@echo "🧪 全テストを実行中..."
	docker compose --profile test up --abort-on-container-exit --exit-code-from test-bridge
	@echo "✅ テスト完了"

# 軽量テスト
test-fast:
	@echo "⚡ 軽量テストを実行中..."
	PYTHONPATH=$(PWD):$(PWD)/src python3 -m pytest tests/test_gym_api.py -v
	@echo "✅ 軽量テスト完了"

# GPU環境テスト
test-gpu:
	@echo "🎮 GPU環境テストを実行中..."
	docker compose --profile gpu up --abort-on-container-exit
	@echo "✅ GPU環境テスト完了"

# クリーンアップ
clean:
	@echo "🧹 クリーンアップ中..."
	docker compose down --volumes --remove-orphans
	docker system prune -af
	docker volume prune -f
	@echo "✅ クリーンアップ完了"

# ログ表示
logs:
	@echo "📋 ログを表示中..."
	docker compose logs -f

# 状態確認
status:
	@echo "📊 サービス状態:"
	docker compose ps
	@echo ""
	@echo "🔍 ヘルスチェック:"
	docker compose exec sim ros2 topic list 2>/dev/null || echo "シミュレーション未起動"
	docker compose exec bridge ros2 node list 2>/dev/null || echo "Bridge未起動"

# 開発環境セットアップ
setup:
	@echo "🔧 開発環境をセットアップ中..."
	@if [ ! -f .env ]; then \
		echo "ROS_DISTRO=humble" > .env; \
		echo "IGNITION_VERSION=garden" >> .env; \
		echo "GPU_COUNT=1" >> .env; \
		echo "✅ .envファイルを作成しました"; \
	else \
		echo "✅ .envファイルは既に存在します"; \
	fi
	@echo "✅ セットアップ完了"

# パフォーマンステスト
perf-test:
	@echo "⚡ パフォーマンステストを実行中..."
	docker compose --profile test up -d test-sim test-bridge
	@echo "⏱️  60秒間のパフォーマンス測定..."
	timeout 60 docker compose logs -f test-sim test-bridge || true
	@echo "✅ パフォーマンステスト完了"

# ヘルプ（詳細版）
help-detailed:
	@echo "詳細な使用方法:"
	@echo ""
	@echo "1. 初回セットアップ:"
	@echo "   make setup"
	@echo "   make build"
	@echo ""
	@echo "2. シミュレーション実行:"
	@echo "   make sim"
	@echo "   make status"
	@echo ""
	@echo "3. RL学習実行:"
	@echo "   make train"
	@echo "   make logs"
	@echo ""
	@echo "4. テスト実行:"
	@echo "   make test      # 統合テスト"
	@echo "   make test-fast # 軽量テスト"
	@echo "   make test-gpu  # GPUテスト"
	@echo ""
	@echo "5. クリーンアップ:"
	@echo "   make clean"
	@echo ""
	@echo "環境変数設定 (.env):"
	@echo "   ROS_DISTRO=humble"
	@echo "   IGNITION_VERSION=garden"
	@echo "   GPU_COUNT=1"
	@echo "   CUDA_VISIBLE_DEVICES=0" 