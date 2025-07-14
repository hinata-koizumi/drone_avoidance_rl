# syntax=docker/dockerfile:1.4
ARG GITHUB_REPOSITORY_OWNER=hinata-koizumi
ARG BASE_IMAGE=ghcr.io/${GITHUB_REPOSITORY_OWNER}/drone-avoidance-base:2.0.1

FROM ${BASE_IMAGE}

WORKDIR /drone_ws

# Python依存パッケージのインストール（キャッシュ最適化）
COPY requirements.txt ./requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip install --no-cache-dir -r requirements.txt && \
    rm -rf /tmp/* /var/tmp/*

# RL関連のソースコードをコピー
COPY drone_sim_env.py drone_sim_env.py
COPY gym_env.py gym_env.py
COPY train_ray.py train_ray.py
COPY vector_env_example.py vector_env_example.py
COPY tests/ tests/
COPY sample_agent/ sample_agent/

# Entrypoint（正しいパスでコピー）
COPY docker/rl-agent/entrypoint-rl-agent.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# PYTHONPATHの安全な設定
ENV PYTHONPATH=/drone_ws/src:${PYTHONPATH:-}
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"] 