"""
Simulation launch test suite for local GitHub Actions testing.
Tests the simulation launch files and configuration.
"""
import pytest
import yaml
import os
from pathlib import Path


class TestSimulationLaunch:
    """Test suite for simulation launch files."""
    
    def test_sim_params_yaml_exists(self):
        """Test that sim_params.yaml exists and is valid YAML."""
        config_path = Path("src/sim_launch/config/sim_params.yaml")
        assert config_path.exists(), "sim_params.yaml does not exist"
        
        with open(config_path, 'r') as f:
            try:
                params = yaml.safe_load(f)
                assert isinstance(params, dict), "sim_params.yaml should be a dictionary"
            except yaml.YAMLError as e:
                pytest.fail(f"sim_params.yaml is not valid YAML: {e}")
    
    def test_sim_params_required_keys(self):
        """Test that sim_params.yaml contains required configuration keys."""
        config_path = Path("src/sim_launch/config/sim_params.yaml")
        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)
        
        # Required keys for bridge topics
        assert 'bridge_topics' in params, "bridge_topics key is required"
        assert isinstance(params['bridge_topics'], list), "bridge_topics should be a list"
        
        # Check bridge topic structure
        for bridge in params['bridge_topics']:
            assert 'topic' in bridge, "Each bridge should have a topic"
            assert 'ros_type' in bridge, "Each bridge should have a ros_type"
            assert 'ign_type' in bridge, "Each bridge should have an ign_type"
    
    def test_sim_all_launch_exists(self):
        """Test that sim_all.launch.py exists."""
        launch_path = Path("src/sim_launch/launch/sim_all.launch.py")
        assert launch_path.exists(), "sim_all.launch.py does not exist"
    
    def test_bridge_launch_exists(self):
        """Test that bridge_launch.py exists."""
        launch_path = Path("src/sim_launch/launch/bridge_launch.py")
        assert launch_path.exists(), "bridge_launch.py does not exist"
    
    def test_gz_sim_launch_exists(self):
        """Test that gz_sim.launch.py exists."""
        launch_path = Path("src/sim_launch/launch/gz_sim.launch.py")
        assert launch_path.exists(), "gz_sim.launch.py does not exist"
    
    def test_launch_files_importable(self):
        """Test that launch files can be imported without syntax errors."""
        # Test sim_all.launch.py
        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "sim_all_launch", 
                "src/sim_launch/launch/sim_all.launch.py"
            )
            if spec is None or spec.loader is None:
                pytest.fail("Failed to create module spec for sim_all.launch.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            assert hasattr(module, 'generate_launch_description'), \
                "sim_all.launch.py should have generate_launch_description function"
        except Exception as e:
            pytest.fail(f"sim_all.launch.py import failed: {e}")
        
        # Test bridge_launch.py
        try:
            spec = importlib.util.spec_from_file_location(
                "bridge_launch", 
                "src/sim_launch/launch/bridge_launch.py"
            )
            if spec is None or spec.loader is None:
                pytest.fail("Failed to create module spec for bridge_launch.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except Exception as e:
            pytest.fail(f"bridge_launch.py import failed: {e}")
        
        # Test gz_sim.launch.py
        try:
            spec = importlib.util.spec_from_file_location(
                "gz_sim_launch", 
                "src/sim_launch/launch/gz_sim.launch.py"
            )
            if spec is None or spec.loader is None:
                pytest.fail("Failed to create module spec for gz_sim.launch.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except Exception as e:
            pytest.fail(f"gz_sim.launch.py import failed: {e}")
    
    def test_sim_params_bridge_topics_structure(self):
        """Test that bridge_topics have correct structure."""
        config_path = Path("src/sim_launch/config/sim_params.yaml")
        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)
        
        for bridge in params['bridge_topics']:
            # Check required fields
            assert 'topic' in bridge, "Bridge missing topic"
            assert 'ros_type' in bridge, "Bridge missing ros_type"
            assert 'ign_type' in bridge, "Bridge missing ign_type"
            
            # Check topic format
            assert bridge['topic'].startswith('/'), "Topic should start with /"
            
            # Check direction if present
            if 'direction' in bridge:
                assert bridge['direction'] in ['bidirectional', 'ros_to_ign', 'ign_to_ros'], \
                    "Direction should be one of: bidirectional, ros_to_ign, ign_to_ros"


if __name__ == "__main__":
    pytest.main([__file__]) 