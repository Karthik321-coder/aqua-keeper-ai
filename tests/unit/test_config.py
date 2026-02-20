"""
tests/unit/test_config.py
Unit tests for the configuration loader.
"""

import pytest

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.utils.config import load_config, _deep_merge


class TestDeepMerge:
    def test_simple_override(self):
        base = {"a": 1, "b": 2}
        override = {"b": 99}
        result = _deep_merge(base, override)
        assert result["a"] == 1
        assert result["b"] == 99

    def test_nested_merge(self):
        base = {"camera": {"source": 0, "width": 1280}}
        override = {"camera": {"width": 640}}
        result = _deep_merge(base, override)
        assert result["camera"]["source"] == 0
        assert result["camera"]["width"] == 640

    def test_does_not_mutate_base(self):
        base = {"x": {"y": 1}}
        override = {"x": {"y": 99}}
        _deep_merge(base, override)
        assert base["x"]["y"] == 1

    def test_adds_new_key(self):
        base = {"a": 1}
        override = {"b": 2}
        result = _deep_merge(base, override)
        assert result["b"] == 2


class TestLoadConfig:
    def test_defaults_loaded_without_file(self):
        cfg = load_config(None)
        assert "camera" in cfg
        assert "model" in cfg
        assert "motor" in cfg
        assert "geometry" in cfg
        assert "control" in cfg

    def test_default_values(self):
        cfg = load_config(None)
        assert cfg["geometry"]["goal_width"] == 2.4
        assert cfg["geometry"]["goal_height"] == 0.9
        assert cfg["motor"]["backend"] == "mock"
        assert cfg["control"]["kp"] == 5.0

    def test_file_overrides_defaults(self, tmp_path):
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text("geometry:\n  goal_width: 3.0\n")
        cfg = load_config(str(yaml_file))
        assert cfg["geometry"]["goal_width"] == 3.0
        # Other defaults preserved
        assert cfg["geometry"]["goal_height"] == 0.9

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_config(str(tmp_path / "nonexistent.yaml"))

    def test_empty_yaml_uses_defaults(self, tmp_path):
        yaml_file = tmp_path / "empty.yaml"
        yaml_file.write_text("")
        cfg = load_config(str(yaml_file))
        assert cfg["model"]["imgsz"] == 640
