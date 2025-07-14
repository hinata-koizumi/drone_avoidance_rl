"""
Sample agents test suite for local GitHub Actions testing.
Tests the sample agent implementations.
"""
import os
import tempfile

import numpy as np
import pytest

# Import sample agents
from src.sample_agent.sample_cartpole_agent import (
    AlwaysRightAgent,
    LinearPolicyAgent,
    RandomPolicyAgent,
    load_state,
    run_episode,
    save_state,
)


class TestSampleAgents:
    """Test suite for sample agent implementations."""
    
    def test_random_policy_agent_creation(self):
        """Test RandomPolicyAgent can be created."""
        action_space = type('MockActionSpace', (), {'sample': lambda: 0})()
        agent = RandomPolicyAgent(action_space)
        assert agent is not None
        assert hasattr(agent, 'act')
    
    def test_always_right_agent_creation(self):
        """Test AlwaysRightAgent can be created."""
        action_space = type('MockActionSpace', (), {'sample': lambda: 0})()
        agent = AlwaysRightAgent(action_space)
        assert agent is not None
        assert hasattr(agent, 'act')
    
    def test_always_right_agent_action(self):
        """Test AlwaysRightAgent always returns action 1."""
        action_space = type('MockActionSpace', (), {'sample': lambda: 0})()
        agent = AlwaysRightAgent(action_space)
        
        # Test multiple observations
        for _ in range(10):
            obs = np.random.random(4)
            action = agent.act(obs)
            assert action == 1, "AlwaysRightAgent should always return action 1"
    
    def test_linear_policy_agent_creation(self):
        """Test LinearPolicyAgent can be created."""
        obs_space = type('MockObsSpace', (), {'shape': (4,)})()
        action_space = type('MockActionSpace', (), {'n': 2})()
        agent = LinearPolicyAgent(obs_space, action_space)
        assert agent is not None
        assert hasattr(agent, 'act')
        assert hasattr(agent, 'update')
        assert hasattr(agent, 'save_model')
        assert hasattr(agent, 'load_model')
    
    def test_linear_policy_agent_action(self):
        """Test LinearPolicyAgent can generate actions."""
        obs_space = type('MockObsSpace', (), {'shape': (4,)})()
        action_space = type('MockActionSpace', (), {'n': 2})()
        agent = LinearPolicyAgent(obs_space, action_space)
        
        obs = np.random.random(4)
        action = agent.act(obs)
        assert isinstance(action, int)
        assert 0 <= action < 2
    
    def test_linear_policy_agent_update(self):
        """Test LinearPolicyAgent can update its policy."""
        obs_space = type('MockObsSpace', (), {'shape': (4,)})()
        action_space = type('MockActionSpace', (), {'n': 2})()
        agent = LinearPolicyAgent(obs_space, action_space)
        
        obs = np.random.random(4)
        action = 0
        reward = 1.0
        
        # Should not raise exception
        agent.update(obs, action, reward)
    
    def test_linear_policy_agent_save_load(self):
        """Test LinearPolicyAgent can save and load models."""
        obs_space = type('MockObsSpace', (), {'shape': (4,)})()
        action_space = type('MockActionSpace', (), {'n': 2})()
        agent = LinearPolicyAgent(obs_space, action_space)
        
        with tempfile.NamedTemporaryFile(suffix='.pt', delete=False) as tmp_file:
            model_path = tmp_file.name
        
        try:
            # Save model
            agent.save_model(model_path)
            assert os.path.exists(model_path)
            
            # Create new agent and load model
            new_agent = LinearPolicyAgent(obs_space, action_space)
            new_agent.load_model(model_path)
            
            # Test that both agents produce same action for same observation
            obs = np.random.random(4)
            action1 = agent.act(obs)
            action2 = new_agent.act(obs)
            assert action1 == action2, "Loaded model should produce same actions"
            
        finally:
            if os.path.exists(model_path):
                os.unlink(model_path)
    
    def test_save_load_state(self):
        """Test state save and load functionality."""
        with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as tmp_file:
            state_path = tmp_file.name
        
        try:
            # Test save state
            save_state(seed=42, episode=10, path=state_path)
            assert os.path.exists(state_path)
            
            # Test load state
            state = load_state(state_path)
            assert state is not None
            assert state['seed'] == 42
            assert state['episode'] == 10
            
            # Test load non-existent state
            non_existent_state = load_state("non_existent.pkl")
            assert non_existent_state is None
            
        finally:
            if os.path.exists(state_path):
                os.unlink(state_path)
    
    def test_run_episode_with_mock_env(self):
        """Test run_episode function with a mock environment."""
        class MockEnv:
            def __init__(self):
                self.reset_called = False
                self.step_called = False
                self.render_called = False
                self.close_called = False
            
            def reset(self):
                self.reset_called = True
                return np.random.random(4), {}
            
            def step(self, action):
                self.step_called = True
                return np.random.random(4), 1.0, True, False, {}
            
            def render(self):
                self.render_called = True
            
            def close(self):
                self.close_called = True
        
        mock_env = MockEnv()
        action_space = type('MockActionSpace', (), {'sample': lambda: 0})()
        agent = RandomPolicyAgent(action_space)
        
        total_reward = run_episode(mock_env, agent, render=True)
        
        assert mock_env.reset_called
        assert mock_env.step_called
        assert mock_env.render_called
        assert isinstance(total_reward, (int, float))


if __name__ == "__main__":
    pytest.main([__file__]) 