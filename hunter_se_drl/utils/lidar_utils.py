#!/usr/bin/env python3
import base64
import gzip

import numpy as np


def decode_lidar(b64_data: str) -> np.ndarray:
    """base64 + gzip으로 인코딩된 LiDAR 문자열을 float 배열로 디코딩한다."""
    try:
        raw = base64.b64decode(b64_data)
        decompressed = gzip.decompress(raw).decode("utf-8")
        values = [float(v) for v in decompressed.split("\n") if v.strip()]
        return np.array(values, dtype=np.float32)
    except Exception:
        return np.array([], dtype=np.float32)


def preprocess_lidar(
    ranges: np.ndarray,
    target_dim: int,
    max_range: float,
) -> np.ndarray:
    """디코딩된 LiDAR 배열을 신경망 입력에 맞게 전처리한다."""
    # 유효하지 않은 값 처리
    ranges = ranges.copy()
    ranges[ranges <= 0] = max_range
    ranges[~np.isfinite(ranges)] = max_range

    # target_dim으로 리샘플링
    if len(ranges) != target_dim:
        x_orig = np.linspace(0, 1, len(ranges))
        x_new = np.linspace(0, 1, target_dim)
        ranges = np.interp(x_new, x_orig, ranges)

    # 정규화
    return (ranges / max_range).astype(np.float32)
