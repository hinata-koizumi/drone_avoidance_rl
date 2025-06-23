import os
import pickle

import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter


class RandomPolicyAgent:
    def __init__(self, action_space):
        self.action_space = action_space
    def act(self, observation):
        return self.action_space.sample()

class AlwaysRightAgent:
    def __init__(self, action_space):
        self.action_space = action_space
    def act(self, observation):
        return 1  # always push cart to the right

class LinearPolicyAgent:
    def __init__(self, obs_space, action_space, lr=1e-2):
        self.obs_space = obs_space
        self.action_space = action_space
        self.model = nn.Linear(obs_space.shape[0], action_space.n)
        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)
        self.criterion = nn.CrossEntropyLoss()

    def act(self, observation):
        obs = torch.tensor(observation, dtype=torch.float32)
        logits = self.model(obs)
        action = torch.argmax(logits).item()
        return action

    def update(self, obs, action, reward):
        obs = torch.tensor(obs, dtype=torch.float32)
        action = torch.tensor([action], dtype=torch.long)
        logits = self.model(obs)
        loss = self.criterion(logits.unsqueeze(0), action)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def save_model(self, path):
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
        }, path)

    def load_model(self, path):
        checkpoint = torch.load(path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])

def run_episode(env, agent, render=False):
    obs, info = env.reset()
    total_reward = 0
    done = False
    while not done:
        if render:
            env.render()
        action = agent.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        total_reward += reward
    return total_reward

def train_cartpole_agent(episodes=100):
    env = gym.make('CartPole-v1')
    agent = RandomPolicyAgent(env.action_space)
    rewards = []
    writer = SummaryWriter(log_dir="runs/cartpole_random_policy")
    for ep in range(episodes):
        ep_reward = run_episode(env, agent)
        rewards.append(ep_reward)
        print(f"[RandomPolicy] Episode {ep+1}: reward={ep_reward}")
        writer.add_scalar("Reward/Episode", ep_reward, ep)
    writer.add_scalar("Reward/Mean", np.mean(rewards), episodes)
    writer.close()
    env.close()
    print(f"RandomPolicy: mean reward over {episodes} episodes: {np.mean(rewards):.2f}")

def test_always_right_agent(episodes=10):
    env = gym.make('CartPole-v1')
    agent = AlwaysRightAgent(env.action_space)
    rewards = []
    writer = SummaryWriter(log_dir="runs/cartpole_always_right")
    for ep in range(episodes):
        ep_reward = run_episode(env, agent)
        rewards.append(ep_reward)
        print(f"[AlwaysRight] Episode {ep+1}: reward={ep_reward}")
        writer.add_scalar("Reward/Episode", ep_reward, ep)
    writer.add_scalar("Reward/Mean", np.mean(rewards), episodes)
    writer.close()
    env.close()
    print(f"AlwaysRight: mean reward over {episodes} episodes: {np.mean(rewards):.2f}")

def save_state(seed, episode, path="linear_policy_state.pkl"):
    state = {
        'seed': seed,
        'episode': episode
    }
    with open(path, 'wb') as f:
        pickle.dump(state, f)

def load_state(path="linear_policy_state.pkl"):
    if not os.path.exists(path):
        return None
    with open(path, 'rb') as f:
        state = pickle.load(f)
    return state

def train_linear_policy_agent(
    episodes=100,
    save_path="linear_policy.pt",
    resume=False,
    state_path="linear_policy_state.pkl"
):
    env = gym.make('CartPole-v1')
    agent = LinearPolicyAgent(env.observation_space, env.action_space)
    start_ep = 0
    seed = 42
    if resume:
        agent.load_model(save_path)
        state = load_state(state_path)
        if state is not None:
            seed = state['seed']
            start_ep = state['episode']
            print(f"[LinearPolicy] Resumed from {save_path}, episode {start_ep}, seed {seed}")
    np.random.seed(seed)
    torch.manual_seed(seed)
    env.reset(seed=seed)
    rewards = []
    writer = SummaryWriter(log_dir="runs/cartpole_linear_policy")
    for ep in range(start_ep, episodes):
        obs, info = env.reset()
        total_reward = 0
        done = False
        while not done:
            action = agent.act(obs)
            next_obs, reward, terminated, truncated, info = env.step(action)
            agent.update(obs, action, reward)
            obs = next_obs
            done = terminated or truncated
            total_reward += reward
        rewards.append(total_reward)
        print(f"[LinearPolicy] Episode {ep+1}: reward={total_reward}")
        writer.add_scalar("Reward/Episode", total_reward, ep)
        # Save checkpoint every 10 episodes
        if (ep+1) % 10 == 0:
            agent.save_model(save_path)
            save_state(seed, ep+1, state_path)
            print(f"[LinearPolicy] Checkpoint saved at episode {ep+1}")
    writer.add_scalar("Reward/Mean", np.mean(rewards), episodes)
    writer.close()
    env.close()
    agent.save_model(save_path)
    save_state(seed, episodes, state_path)
    print(f"LinearPolicy: mean reward over {episodes} episodes: {np.mean(rewards):.2f}")
    print(f"Model saved to {save_path}")
    print(f"State saved to {state_path}")

if __name__ == "__main__":
    print("=== RandomPolicyAgent (CartPole-v1) ===")
    train_cartpole_agent(episodes=10)
    print("\n=== AlwaysRightAgent (CartPole-v1) ===")
    test_always_right_agent(episodes=5)
    print("\n=== LinearPolicyAgent (CartPole-v1) ===")
    train_linear_policy_agent(episodes=20, save_path="linear_policy.pt", resume=False) 