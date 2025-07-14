FROM python:3.10-slim

# 軽量CPU依存パッケージのみインストール
WORKDIR /rl_ws

# 必要最小限のPythonライブラリ
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip install --no-cache-dir torch==2.0.1+cpu -f https://download.pytorch.org/whl/cpu/torch_stable.html gymnasium==1.1.1 numpy && \
    rm -rf /root/.cache /tmp/* /var/tmp/*

# デフォルトコマンド：5ステップだけ回して終了
CMD ["python3", "- <<'PY'", "import gymnasium as gym, torch, numpy as np; env=gym.make('CartPole-v1'); obs,_=env.reset(seed=0); [env.step(env.action_space.sample()) for _ in range(5)]; print('RL mini test ok');", "PY"]