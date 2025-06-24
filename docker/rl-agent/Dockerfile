ARG GITHUB_REPOSITORY_OWNER=hinata-koizumi
ARG BASE_IMAGE=ghcr.io/${GITHUB_REPOSITORY_OWNER}/drone-avoidance-base:2.0.1
FROM ${BASE_IMAGE}

WORKDIR /drone_ws

# Python依存パッケージのインストール
COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt
RUN rm -rf /tmp/* /var/tmp/* /root/.cache/* || true

# RL関連のソースコードをコピー
COPY src/drone_sim_env.py src/drone_sim_env.py
COPY src/vector_env_example.py src/vector_env_example.py
COPY tests/ tests/
COPY src/sample_agent/ src/sample_agent/

# Entrypoint
COPY docker/rl-agent/entrypoint-rl-agent.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV PYTHONPATH=/drone_ws/src:$PYTHONPATH
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"] 