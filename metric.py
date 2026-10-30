"""
Performance Metrics - PEI, tracking, proximity
"""

import numpy as np


def calculate_tracking_distance(ego_history, distance_history, threshold=15.0,
                                 dt=0.1):
    tracking_idx = np.where(distance_history < threshold)[0]
    if len(tracking_idx) == 0:
        return {'tracking_distance': 0, 'tracking_time': 0,
                'tracking_percentage': 0, 'total_distance': 0}

    tracking_dist = 0.0
    for i in range(len(tracking_idx) - 1):
        idx = tracking_idx[i]
        if idx < len(ego_history) - 1:
            tracking_dist += np.linalg.norm(
                ego_history[idx+1] - ego_history[idx])

    total_dist = np.sum(np.linalg.norm(np.diff(ego_history, axis=0), axis=1))

    return {
        'tracking_distance': tracking_dist,
        'total_distance': total_dist,
        'tracking_percentage': tracking_dist / max(total_dist, 1e-6) * 100,
        'tracking_time': len(tracking_idx) * dt
    }


def calculate_proximity_time(distance_history, dt=0.1):
    n = len(distance_history)
    return {
        'very_close': {
            'time': np.sum(distance_history < 5) * dt,
            'percentage': np.sum(distance_history < 5) / n * 100
        },
        'close': {
            'time': np.sum((distance_history >= 5) & (distance_history < 10)) * dt,
            'percentage': np.sum((distance_history >= 5) & (distance_history < 10)) / n * 100
        },
        'medium': {
            'time': np.sum((distance_history >= 10) & (distance_history < 20)) * dt,
            'percentage': np.sum((distance_history >= 10) & (distance_history < 20)) / n * 100
        },
        'far': {
            'time': np.sum(distance_history >= 20) * dt,
            'percentage': np.sum(distance_history >= 20) / n * 100
        }
    }


def calculate_pei(ego_history, target_history, distance_history, time_history):
    initial_dist = distance_history[0]
    final_dist = np.min(distance_history)
    total_time = time_history[-1]

    reduction_rate = (initial_dist - final_dist) / total_time
    tracking_time = np.sum(distance_history < 15) * 0.1
    tracking_score = tracking_time / total_time
    final_dist_score = np.exp(-final_dist / 10)

    ego_path = np.sum(np.linalg.norm(np.diff(ego_history, axis=0), axis=1))
    target_path = np.sum(np.linalg.norm(np.diff(target_history, axis=0), axis=1))
    path_efficiency = min(1, target_path / max(ego_path, 1e-6))

    overall = (0.3 * min(1, reduction_rate/2) +
               0.4 * tracking_score +
               0.2 * final_dist_score +
               0.1 * path_efficiency)

    return {
        'overall_score': overall,
        'reduction_rate': reduction_rate,
        'tracking_time': tracking_time,
        'tracking_ratio': tracking_score,
        'final_distance': final_dist,
        'path_efficiency': path_efficiency
    }