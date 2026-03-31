#!/usr/bin/env python3
import json
import os

import yaml


def load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        raise FileNotFoundError(f"YAML 파일을 찾을 수 없습니다: {path}")
    with open(path, "r") as f:
        return yaml.safe_load(f)


def save_yaml(path: str, data: dict):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(data, f, allow_unicode=True, default_flow_style=False)


def save_json(path: str, data: dict):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)
