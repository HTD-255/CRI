"""
Hysteresis Switching Mechanism
Chống chattering bằng dải trễ + dwell-time
"""


class HysteresisSwitcher:
    def __init__(self, theta_on=0.65, theta_off=0.45, n_dwell=2,
                 initial_mode='FALLBACK'):
        self.theta_on = theta_on
        self.theta_off = theta_off
        self.n_dwell = n_dwell

        self.current_mode = initial_mode
        self.dwell_counter = 0
        self.cri_history = []

    def decide(self, cri):
        """
        Quyết định mode dựa trên CRI

        Returns:
            mode: 'NMPC' hoặc 'FALLBACK'
        """
        self.cri_history.append(cri)
        self.dwell_counter += 1

        switch_allowed = (self.dwell_counter >= self.n_dwell)

        if self.current_mode == 'FALLBACK':
            if cri > self.theta_on and switch_allowed:
                self.current_mode = 'NMPC'
                self.dwell_counter = 0
        elif self.current_mode == 'NMPC':
            if cri < self.theta_off and switch_allowed:
                self.current_mode = 'FALLBACK'
                self.dwell_counter = 0

        return self.current_mode