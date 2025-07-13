# syntax=docker/dockerfile:1.4
ARG GITHUB_REPOSITORY_OWNER=hinata-koizumi
ARG BASE_IMAGE=ghcr.io/${GITHUB_REPOSITORY_OWNER}/drone-avoidance-base:2.0.1

# ベースステージ
FROM ${BASE_IMAGE} AS base

# ビルドステージ
FROM base AS builder

WORKDIR /drone_ws

# Python依存パッケージのインストール（キャッシュ最適化）
COPY requirements.txt requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip install --no-cache-dir -r requirements.txt && \
    rm -rf /tmp/* /var/tmp/*

# RL関連のソースコードをコピー
COPY src/drone_sim_env.py src/drone_sim_env.py
COPY src/vector_env_example.py src/vector_env_example.py
COPY tests/ tests/
COPY src/sample_agent/ src/sample_agent/

# ランタイムステージ
FROM base AS runtime

WORKDIR /drone_ws

# ビルドステージからPythonパッケージをコピー
COPY --from=builder /usr/local/lib/python3.10/site-packages /usr/local/lib/python3.10/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# RL関連のソースコードをコピー
COPY src/drone_sim_env.py src/drone_sim_env.py
COPY src/vector_env_example.py src/vector_env_example.py
COPY tests/ tests/
COPY src/sample_agent/ src/sample_agent/

# Entrypoint（正しいパスでコピー）
COPY docker/rl-agent/entrypoint-rl-agent.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# PYTHONPATHの安全な設定
ENV PYTHONPATH=/drone_ws/src:${PYTHONPATH:-}
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"] 