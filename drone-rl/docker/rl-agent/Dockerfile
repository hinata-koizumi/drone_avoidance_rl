# syntax=docker/dockerfile:1.4
ARG GITHUB_REPOSITORY_OWNER=hinata-koizumi
ARG BASE_IMAGE=ghcr.io/${GITHUB_REPOSITORY_OWNER}/drone-avoidance-base:2.0.1

FROM ${BASE_IMAGE}

WORKDIR /drone_ws

# Python依存パッケージのインストール（キャッシュ最適化）
COPY drone-rl/requirements.txt ./requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip install --no-cache-dir -r requirements.txt && \
    rm -rf /tmp/* /var/tmp/* /root/.cache/* && \
    find /usr -name "*.pyc" -delete && \
    find /usr -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true

# RL関連のソースコードをコピー（最小限）
COPY drone-rl/gym_env.py gym_env.py
COPY drone-rl/environments/ environments/
COPY drone-rl/agents/ agents/
COPY drone-rl/utils/ utils/
COPY drone-rl/scripts/ scripts/
COPY drone-rl/tests/ tests/

# Entrypoint（正しいパスでコピー）
COPY drone-rl/docker/rl-agent/entrypoint-rl-agent.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# PYTHONPATHの安全な設定
ENV PYTHONPATH=/drone_ws/src:${PYTHONPATH:-}
ENV ROS_DOMAIN_ID=0

# クリーンアップ
RUN rm -rf /tmp/* /var/tmp/* /root/.cache/* /usr/share/doc/* /usr/share/man/* /var/lib/apt/lists/* || true

ENTRYPOINT ["/entrypoint.sh"] 