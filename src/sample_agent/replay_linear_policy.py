
import gymnasium as gym
import numpy as np
from sample_cartpole_agent import LinearPolicyAgent, load_state

MODEL_PATH = "linear_policy.pt"
STATE_PATH = "linear_policy_state.pkl"
EPISODES = 5
RENDER = True

def main():
    # 状態・シードの復元
    state = load_state(STATE_PATH)
    seed = state['seed'] if state else 42
    print(f"[Replay] Using seed: {seed}")
    env = gym.make('CartPole-v1', render_mode='human' if RENDER else None)
    env.reset(seed=seed)
    agent = LinearPolicyAgent(env.observation_space, env.action_space)
    agent.load_model(MODEL_PATH)
    rewards = []
    for ep in range(EPISODES):
        obs, info = env.reset()
        total_reward = 0
        done = False
        while not done:
            if RENDER:
                env.render()
            action = agent.act(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            total_reward += reward
        rewards.append(total_reward)
        print(f"[Replay] Episode {ep+1}: reward={total_reward}")
    env.close()
    print(f"[Replay] Mean reward over {EPISODES} episodes: {np.mean(rewards):.2f}")

if __name__ == "__main__":
    main() 