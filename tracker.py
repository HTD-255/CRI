"""
Simple Multi-Object Tracker
Quản lý khởi tạo, gán, xác nhận, xóa track
"""

import numpy as np
from ukf import UKF
from vehicle_model import wrap_to_pi


class SimpleTracker:
    def __init__(self, dt=0.1):
        self.tracks = []
        self.dt = dt

        # Nhiễu
        self.Q = np.diag([0.2, 0.2, 0.5, 1.0])
        self.R = np.diag([0.5, 0.5, 0.3, 0.1])
        self.P_init = np.diag([1.0, 1.0, 1.0, 0.5])

        # Tham số
        self.assignment_threshold = 15.0

    def step(self, detections):
        """
        Một bước tracker

        Parameters:
            detections: list of np.array [x, y, v, theta]

        Returns:
            confirmed_tracks: list of dict {State, Covariance, Age, TrackID}
        """
        # 1. PREDICT tất cả tracks
        for track in self.tracks:
            track.predict()

        # 2. DATA ASSOCIATION (Nearest Neighbor)
        n_tracks = len(self.tracks)
        n_dets = len(detections)
        assigned_tracks = [False] * n_tracks
        assigned_dets = [False] * n_dets

        if n_tracks > 0 and n_dets > 0:
            # Cost matrix (Mahalanobis)
            cost = np.full((n_tracks, n_dets), np.inf)

            for i in range(n_tracks):
                for j in range(n_dets):
                    z = np.array(detections[j])
                    z_pred = self.tracks[i].x
                    nu = z - z_pred
                    nu[3] = wrap_to_pi(nu[3])

                    S = self.tracks[i].P + self.tracks[i].R
                    try:
                        d = np.sqrt(nu @ np.linalg.inv(S) @ nu)
                    except np.linalg.LinAlgError:
                        d = np.inf

                    if d < self.assignment_threshold:
                        cost[i, j] = d

            # Greedy assignment
            for _ in range(min(n_tracks, n_dets)):
                min_val = np.min(cost)
                if min_val >= self.assignment_threshold:
                    break
                idx = np.unravel_index(np.argmin(cost), cost.shape)
                row, col = idx

                self.tracks[row].update(detections[col])
                assigned_tracks[row] = True
                assigned_dets[col] = True

                cost[row, :] = np.inf
                cost[:, col] = np.inf

        # 3. UNASSIGNED TRACKS
        for i in range(n_tracks):
            if not assigned_tracks[i]:
                self.tracks[i].missed()
            self.tracks[i].update_age()

        # 4. NEW TRACKS
        for j in range(n_dets):
            if not assigned_dets[j]:
                z = np.array(detections[j])
                new_track = UKF(z, self.P_init.copy(), self.Q.copy(),
                                self.R.copy(), self.dt)
                self.tracks.append(new_track)

        # 5. DELETE TRACKS
        self.tracks = [t for t in self.tracks if not t.should_delete()]

        # 6. RETURN CONFIRMED
        confirmed = []
        track_id = 1
        for track in self.tracks:
            if track.confirmed:
                confirmed.append({
                    'TrackID': track_id,
                    'State': track.x.copy(),
                    'Covariance': track.P.copy(),
                    'Age': track.age
                })
                track_id += 1

        return confirmed