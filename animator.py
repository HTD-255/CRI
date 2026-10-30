"""
Real-time Simulation Visualization bằng Pygame
Mượt, nhanh, tương tác được (zoom, pan, pause)
"""

import numpy as np
import pygame
import sys
import math


class Colors:
    """Bảng màu"""
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GRAY = (200, 200, 200)
    DARK_GRAY = (100, 100, 100)
    LIGHT_GRAY = (230, 230, 230)

    # Xe
    EGO_BLUE = (41, 128, 205)
    EGO_DARK = (25, 80, 140)
    TARGET_RED = (231, 76, 60)
    TARGET_DARK = (150, 40, 30)

    # Trail
    EGO_TRAIL = (100, 160, 230)
    TARGET_TRAIL = (230, 130, 120)

    # Mode
    NMPC_GREEN = (46, 204, 113)
    PP_ORANGE = (230, 126, 34)
    NO_TRACK_RED = (231, 76, 60)

    # CRI
    CRI_BLUE = (52, 152, 219)
    CRI_ON = (46, 204, 113)
    CRI_OFF = (231, 76, 60)
    CRI_DEAD = (241, 196, 15)

    # Misc
    ROAD_COLOR = (180, 180, 180)
    GRASS_COLOR = (144, 190, 109)
    INTERCEPT_GREEN = (0, 230, 64)
    GRID_COLOR = (220, 220, 220)
    HEADING_GREEN = (0, 200, 0)

    # Dashboard
    PANEL_BG = (245, 245, 250)
    PANEL_BORDER = (180, 180, 200)
    GRAPH_BG = (255, 255, 255)


class Camera:
    """Camera 2D: zoom, pan, follow"""

    def __init__(self, screen_w, screen_h, world_x=0, world_y=0, zoom=4.0):
        self.screen_w = screen_w
        self.screen_h = screen_h
        self.world_x = world_x  # tâm camera trong world
        self.world_y = world_y
        self.zoom = zoom         # pixels per meter
        self.target_zoom = zoom
        self.target_x = world_x
        self.target_y = world_y
        self.smooth = 0.08       # smoothing factor

    def world_to_screen(self, wx, wy):
        """Chuyển tọa độ world → screen"""
        sx = (wx - self.world_x) * self.zoom + self.screen_w / 2
        sy = self.screen_h / 2 - (wy - self.world_y) * self.zoom
        return int(sx), int(sy)

    def screen_to_world(self, sx, sy):
        """Chuyển tọa độ screen → world"""
        wx = (sx - self.screen_w / 2) / self.zoom + self.world_x
        wy = (self.screen_h / 2 - sy) / self.zoom + self.world_y
        return wx, wy

    def follow(self, ego_pos, target_pos, distance):
        """Camera tự động follow trung bình ego & target"""
        cx = (ego_pos[0] + target_pos[0]) / 2
        cy = (ego_pos[1] + target_pos[1]) / 2
        self.target_x = cx
        self.target_y = cy

        # Auto zoom theo khoảng cách
        desired_zoom = max(2.0, min(8.0, 300.0 / max(distance + 20, 30)))
        self.target_zoom = desired_zoom

        # Smooth interpolation
        self.world_x += (self.target_x - self.world_x) * self.smooth
        self.world_y += (self.target_y - self.world_y) * self.smooth
        self.zoom += (self.target_zoom - self.zoom) * self.smooth

    def manual_zoom(self, delta):
        """Zoom bằng scroll"""
        self.target_zoom *= (1.1 if delta > 0 else 0.9)
        self.target_zoom = max(1.0, min(20.0, self.target_zoom))


class DashboardGraph:
    """Đồ thị nhỏ trong dashboard"""

    def __init__(self, rect, title, y_label, y_range,
                 color=(52, 152, 219), h_lines=None):
        """
        rect: (x, y, w, h) trên screen
        h_lines: [(y_val, color, label), ...]
        """
        self.rect = pygame.Rect(rect)
        self.title = title
        self.y_label = y_label
        self.y_min, self.y_max = y_range
        self.color = color
        self.h_lines = h_lines or []
        self.data_x = []
        self.data_y = []
        self.max_points = 500

    def add_point(self, t, val):
        self.data_x.append(t)
        self.data_y.append(val)
        if len(self.data_x) > self.max_points:
            self.data_x.pop(0)
            self.data_y.pop(0)

    def draw(self, surface, font_small, font_tiny):
        r = self.rect

        # Background
        pygame.draw.rect(surface, Colors.GRAPH_BG, r)
        pygame.draw.rect(surface, Colors.PANEL_BORDER, r, 1)

        # Title
        title_surf = font_small.render(self.title, True, Colors.BLACK)
        surface.blit(title_surf, (r.x + 5, r.y + 2))

        # Plot area
        margin = 20
        plot_rect = pygame.Rect(r.x + margin + 25, r.y + margin,
                                 r.w - margin - 30, r.h - margin - 10)

        # Horizontal lines
        for y_val, col, label in self.h_lines:
            if self.y_min <= y_val <= self.y_max:
                frac = 1 - (y_val - self.y_min) / (self.y_max - self.y_min + 1e-10)
                py = int(plot_rect.y + frac * plot_rect.h)
                pygame.draw.line(surface, col, (plot_rect.x, py),
                                 (plot_rect.x + plot_rect.w, py), 1)
                lbl = font_tiny.render(label, True, col)
                surface.blit(lbl, (plot_rect.x + plot_rect.w - lbl.get_width(),
                                    py - 12))

        # Y axis labels
        for i in range(5):
            frac = i / 4
            y_val = self.y_min + frac * (self.y_max - self.y_min)
            py = int(plot_rect.y + (1 - frac) * plot_rect.h)
            lbl = font_tiny.render(f"{y_val:.0f}", True, Colors.DARK_GRAY)
            surface.blit(lbl, (r.x + 3, py - 6))

        # Data line
        if len(self.data_x) > 1:
            t_min = self.data_x[0]
            t_max = max(self.data_x[-1], t_min + 1)

            points = []
            for i in range(len(self.data_x)):
                fx = (self.data_x[i] - t_min) / (t_max - t_min)
                fy = 1 - (self.data_y[i] - self.y_min) / (self.y_max - self.y_min + 1e-10)
                fy = max(0, min(1, fy))
                px = int(plot_rect.x + fx * plot_rect.w)
                py = int(plot_rect.y + fy * plot_rect.h)
                points.append((px, py))

            if len(points) >= 2:
                pygame.draw.lines(surface, self.color, False, points, 2)

            # Current value
            val_text = f"{self.data_y[-1]:.1f}"
            val_surf = font_small.render(val_text, True, self.color)
            surface.blit(val_surf, (plot_rect.x + plot_rect.w - val_surf.get_width() - 5,
                                     plot_rect.y + 2))


class PygameAnimator:
    """
    Real-time simulation animator bằng Pygame

    Bao gồm:
    - Bird's Eye View: xe, trail, intercept, heading
    - Dashboard: distance, speed, CRI
    - Info panel: mode, counters, time
    - Tương tác: zoom (scroll), pause (space), quit (ESC)
    """

    def __init__(self, waypoints, ego_start, width=1400, height=800):
        pygame.init()
        pygame.display.set_caption("Vehicle Pursuit Simulation")

        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()

        self.waypoints = np.array(waypoints)
        self.paused = False
        self.running = True

        # Layout
        self.bev_width = width - 380
        self.dash_x = self.bev_width + 10
        self.dash_width = 360

        # Camera
        self.camera = Camera(self.bev_width, height,
                              ego_start[0], ego_start[1], zoom=4.0)

        # Fonts
        self.font_large = pygame.font.SysFont('consolas', 22, bold=True)
        self.font_medium = pygame.font.SysFont('consolas', 16, bold=True)
        self.font_small = pygame.font.SysFont('consolas', 13)
        self.font_tiny = pygame.font.SysFont('consolas', 11)

        # Dashboard graphs
        graph_w = self.dash_width - 20
        graph_h = 140
        gx = self.dash_x + 10

        self.graph_dist = DashboardGraph(
            (gx, 100, graph_w, graph_h),
            "DISTANCE (m)", "m", (0, 100),
            color=Colors.BLACK,
            h_lines=[(15, Colors.TARGET_RED, "track"),
                     (5, Colors.NMPC_GREEN, "close")]
        )
        self.graph_speed = DashboardGraph(
            (gx, 250, graph_w, graph_h),
            "SPEED (m/s)", "m/s", (0, 30),
            color=Colors.EGO_BLUE,
            h_lines=[(10, Colors.TARGET_RED, "target")]
        )
        self.graph_cri = DashboardGraph(
            (gx, 400, graph_w, graph_h),
            "CRI", "", (0, 1.05),
            color=Colors.CRI_BLUE,
            h_lines=[(0.65, Colors.CRI_ON, "θ_on"),
                     (0.45, Colors.CRI_OFF, "θ_off")]
        )

        # Trails (lưu screen points)
        self.ego_trail_world = []
        self.target_trail_world = []

    def handle_events(self):
        """Xử lý input: ESC, Space, scroll"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.camera.manual_zoom(1)
                elif event.key == pygame.K_MINUS:
                    self.camera.manual_zoom(-1)
            elif event.type == pygame.MOUSEWHEEL:
                self.camera.manual_zoom(event.y)

        return self.running

    def _draw_grid(self):
        """Vẽ lưới tọa độ"""
        cam = self.camera
        grid_spacing = 10  # meters

        # Tìm range visible
        wx_min, wy_max = cam.screen_to_world(0, 0)
        wx_max, wy_min = cam.screen_to_world(self.bev_width, self.height)

        # Vẽ grid lines
        x_start = int(wx_min / grid_spacing) * grid_spacing
        y_start = int(wy_min / grid_spacing) * grid_spacing

        for gx in np.arange(x_start, wx_max, grid_spacing):
            sx, _ = cam.world_to_screen(gx, 0)
            if 0 <= sx <= self.bev_width:
                pygame.draw.line(self.screen, Colors.GRID_COLOR,
                                  (sx, 0), (sx, self.height), 1)

        for gy in np.arange(y_start, wy_max, grid_spacing):
            _, sy = cam.world_to_screen(0, gy)
            if 0 <= sy <= self.height:
                pygame.draw.line(self.screen, Colors.GRID_COLOR,
                                  (0, sy), (self.bev_width, sy), 1)

    def _draw_waypoints(self):
        """Vẽ đường target (waypoints)"""
        cam = self.camera
        points = []
        for wp in self.waypoints:
            sx, sy = cam.world_to_screen(wp[0], wp[1])
            points.append((sx, sy))

        if len(points) >= 2:
            pygame.draw.lines(self.screen, (200, 100, 100), False, points, 2)

    def _draw_trail(self, trail_world, color, width=2):
        """Vẽ vết xe"""
        if len(trail_world) < 2:
            return

        cam = self.camera
        # Chỉ vẽ N điểm cuối để tối ưu
        trail = trail_world[-500:]
        points = []
        for pos in trail:
            sx, sy = cam.world_to_screen(pos[0], pos[1])
            points.append((sx, sy))

        if len(points) >= 2:
            pygame.draw.lines(self.screen, color, False, points, width)

    def _draw_vehicle(self, pos, yaw, color_body, color_border,
                       length=4.5, width=2.0, label=""):
        """Vẽ xe hình chữ nhật xoay"""
        cam = self.camera
        cx, cy = cam.world_to_screen(pos[0], pos[1])

        # 4 góc xe trong local frame
        hl = length / 2 * cam.zoom
        hw = width / 2 * cam.zoom

        corners_local = [
            (-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)
        ]

        # Xoay và dịch
        cos_a = math.cos(-yaw)  # âm vì screen y ngược
        sin_a = math.sin(-yaw)

        corners_screen = []
        for lx, ly in corners_local:
            rx = lx * cos_a - ly * sin_a + cx
            ry = lx * sin_a + ly * cos_a + cy
            corners_screen.append((rx, ry))

        # Vẽ body
        pygame.draw.polygon(self.screen, color_body, corners_screen)
        pygame.draw.polygon(self.screen, color_border, corners_screen, 2)

        # Vẽ mũi xe (tam giác nhỏ ở đầu)
        front_x = hl * 0.8
        nose = [
            (front_x, -hw * 0.3),
            (front_x + hw * 0.5, 0),
            (front_x, hw * 0.3)
        ]
        nose_screen = []
        for lx, ly in nose:
            rx = lx * cos_a - ly * sin_a + cx
            ry = lx * sin_a + ly * cos_a + cy
            nose_screen.append((rx, ry))
        pygame.draw.polygon(self.screen, color_border, nose_screen)

        # Label
        if label:
            lbl = self.font_tiny.render(label, True, color_border)
            self.screen.blit(lbl, (cx - lbl.get_width()//2, cy - hw - 15))

    def _draw_heading(self, pos, yaw, length=8):
        """Vẽ heading arrow"""
        cam = self.camera
        sx, sy = cam.world_to_screen(pos[0], pos[1])
        ex = pos[0] + length * math.cos(yaw)
        ey = pos[1] + length * math.sin(yaw)
        esx, esy = cam.world_to_screen(ex, ey)

        pygame.draw.line(self.screen, Colors.HEADING_GREEN,
                          (sx, sy), (esx, esy), 3)
        pygame.draw.circle(self.screen, Colors.HEADING_GREEN,
                            (esx, esy), 5)

    def _draw_intercept(self, ego_pos, intercept_point):
        """Vẽ intercept point và đường nối"""
        if intercept_point is None:
            return

        cam = self.camera
        esx, esy = cam.world_to_screen(ego_pos[0], ego_pos[1])
        isx, isy = cam.world_to_screen(intercept_point[0], intercept_point[1])

        # Đường nối dashed
        self._draw_dashed_line(self.screen, Colors.INTERCEPT_GREEN,
                                (esx, esy), (isx, isy), 2, 8)

        # Star marker
        self._draw_star(isx, isy, 12, Colors.INTERCEPT_GREEN)

    def _draw_dashed_line(self, surface, color, start, end, width, dash_len):
        """Vẽ đường đứt nét"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1:
            return

        dashes = int(dist / dash_len)
        for i in range(0, dashes, 2):
            f1 = i / dashes
            f2 = min((i + 1) / dashes, 1.0)
            p1 = (int(start[0] + f1*dx), int(start[1] + f1*dy))
            p2 = (int(start[0] + f2*dx), int(start[1] + f2*dy))
            pygame.draw.line(surface, color, p1, p2, width)

    def _draw_star(self, cx, cy, size, color):
        """Vẽ ngôi sao"""
        points = []
        for i in range(10):
            angle = math.pi/2 + i * math.pi / 5
            r = size if i % 2 == 0 else size * 0.4
            px = cx + r * math.cos(angle)
            py = cy - r * math.sin(angle)
            points.append((px, py))
        pygame.draw.polygon(self.screen, color, points)
        pygame.draw.polygon(self.screen, (0, 150, 0), points, 1)

    def _draw_sensor_fov(self, ego_pos, ego_yaw, radius=100, fov_deg=60, offset=1.2):
        """Vẽ hình nón cảm biến (FOV) gắn ở đầu xe, dạng quạt tròn"""
        cam = self.camera
        fov = math.radians(fov_deg)

        # Gốc cảm biến tại đầu xe
        sensor_x = ego_pos[0] + offset * math.cos(ego_yaw)
        sensor_y = ego_pos[1] + offset * math.sin(ego_yaw)
        cx, cy = cam.world_to_screen(sensor_x, sensor_y)

        # Tạo dải điểm trên cung tròn thay vì 2 cạnh thẳng
        num_arc_pts = 40  # càng lớn càng mịn
        points = [(cx, cy)]
        for i in range(num_arc_pts + 1):
            ang = ego_yaw - fov / 2 + i * (fov / num_arc_pts)
            wx = sensor_x + radius * math.cos(ang)
            wy = sensor_y + radius * math.sin(ang)
            points.append(cam.world_to_screen(wx, wy))

        # Vẽ quạt FOV bán trong suốt
        fov_surface = pygame.Surface((self.bev_width, self.height), pygame.SRCALPHA)
        pygame.draw.polygon(fov_surface, (180, 220, 180, 50), points)
        pygame.draw.lines(fov_surface, (120, 180, 120, 160), False, points, 2)
        self.screen.blit(fov_surface, (0, 0))

    def update(self, step_data):
        """
        Cập nhật 1 frame

        Parameters:
            step_data: dict
                time, ego_pos, ego_yaw, ego_speed,
                target_pos, target_yaw,
                distance, intercept_point,
                cri, mode,
                nlmpc_count, pp_count, no_track_count, ...

        Returns:
            True nếu tiếp tục, False nếu user thoát
        """
        if not self.handle_events():
            return False

        if self.paused:
            # Vẫn hiển thị nhưng không cập nhật data
            self.clock.tick(30)
            return True

        d = step_data

        # Lưu trail
        self.ego_trail_world.append(d['ego_pos'].copy())
        self.target_trail_world.append(d['target_pos'].copy())

        # Cập nhật graphs
        t = d.get('time', 0)
        self.graph_dist.add_point(t, d.get('distance', 0))
        self.graph_speed.add_point(t, d.get('ego_speed', 0))
        self.graph_cri.add_point(t, d.get('cri', 0))

        # Auto adjust graph Y range
        if len(self.graph_dist.data_y) > 0:
            self.graph_dist.y_max = max(np.max(self.graph_dist.data_y) * 1.2, 20)
        if len(self.graph_speed.data_y) > 0:
            self.graph_speed.y_max = max(np.max(self.graph_speed.data_y) * 1.2, 15)

        # Camera follow
        self.camera.follow(d['ego_pos'], d['target_pos'], d.get('distance', 50))

        # ═══ VẼ ═══
        self.screen.fill(Colors.GRASS_COLOR,
                          pygame.Rect(0, 0, self.bev_width, self.height))

        # Grid
        self._draw_grid()

        # Waypoints (đường target)
        self._draw_waypoints()

        # Sensor FOV (60°)
        self._draw_sensor_fov(d['ego_pos'], d['ego_yaw'])

        # Trails
        self._draw_trail(self.ego_trail_world, Colors.EGO_TRAIL, 3)
        self._draw_trail(self.target_trail_world, Colors.TARGET_TRAIL, 2)

        # Intercept point + đường nối
        self._draw_intercept(d['ego_pos'], d.get('intercept_point'))

        # Heading
        self._draw_heading(d['ego_pos'], d['ego_yaw'])

        # Xe target
        self._draw_vehicle(d['target_pos'], d['target_yaw'],
                            Colors.TARGET_RED, Colors.TARGET_DARK,
                            label="TARGET")

        # Xe ego
        self._draw_vehicle(d['ego_pos'], d['ego_yaw'],
                            Colors.EGO_BLUE, Colors.EGO_DARK,
                            label="EGO")

        # Dashboard
        self._draw_dashboard(step_data)

        # FPS
        fps = self.clock.get_fps()
        fps_text = self.font_tiny.render(f"FPS: {fps:.0f}", True,
                                          Colors.DARK_GRAY)
        self.screen.blit(fps_text, (5, self.height - 20))

        pygame.display.flip()
        self.clock.tick(60)  # 60 FPS max

        return True

    def _draw_dashboard(self, step_data):
        """Vẽ dashboard bên phải"""
        d = step_data

        # Background
        dash_rect = pygame.Rect(self.dash_x, 0, self.dash_width, self.height)
        pygame.draw.rect(self.screen, Colors.PANEL_BG, dash_rect)
        pygame.draw.line(self.screen, Colors.PANEL_BORDER,
                          (self.dash_x, 0), (self.dash_x, self.height), 2)

        # Title
        title = self.font_large.render("DASHBOARD", True, Colors.BLACK)
        self.screen.blit(title, (self.dash_x + 10, 10))

        # Mode indicator
        mode = d.get('mode', 'NO_TRACK')
        if mode == 'NMPC':
            mode_color = Colors.NMPC_GREEN
            mode_text = "● NMPC"
        elif mode == 'PP_FALLBACK':
            mode_color = Colors.PP_ORANGE
            mode_text = "● PURE PURSUIT"
        elif mode == 'SIMPLE_FALLBACK':
            mode_color = Colors.PP_ORANGE
            mode_text = "● SIMPLE FB"
        else:
            mode_color = Colors.NO_TRACK_RED
            mode_text = "● NO TRACK"

        mode_rect = pygame.Rect(self.dash_x + 10, 40, self.dash_width - 20, 30)
        pygame.draw.rect(self.screen, mode_color, mode_rect, border_radius=5)
        mode_surf = self.font_medium.render(mode_text, True, Colors.WHITE)
        self.screen.blit(mode_surf,
                          (mode_rect.centerx - mode_surf.get_width()//2,
                           mode_rect.centery - mode_surf.get_height()//2))

        # Time
        time_text = f"t = {d.get('time', 0):.1f} s"
        self.screen.blit(self.font_small.render(time_text, True, Colors.BLACK),
                          (self.dash_x + 10, 75))

        # Graphs
        self.graph_dist.draw(self.screen, self.font_small, self.font_tiny)
        self.graph_speed.draw(self.screen, self.font_small, self.font_tiny)
        self.graph_cri.draw(self.screen, self.font_small, self.font_tiny)

        # Counters
        cy = 560
        stats = [
            ("NMPC", d.get('nlmpc_count', 0), Colors.NMPC_GREEN),
            ("PP FB", d.get('pp_count', 0), Colors.PP_ORANGE),
            ("No Track", d.get('no_track_count', 0), Colors.NO_TRACK_RED),
        ]
        total = sum(s[1] for s in stats)

        self.screen.blit(
            self.font_medium.render("MODE DISTRIBUTION", True, Colors.BLACK),
            (self.dash_x + 10, cy))
        cy += 25

        for name, count, color in stats:
            pct = count / max(total, 1) * 100

            # Bar
            bar_w = int(pct / 100 * (self.dash_width - 100))
            bar_rect = pygame.Rect(self.dash_x + 80, cy + 2, bar_w, 16)
            pygame.draw.rect(self.screen, color, bar_rect, border_radius=3)

            # Text
            text = f"{name:9s} {count:4d} ({pct:4.0f}%)"
            self.screen.blit(
                self.font_tiny.render(text, True, Colors.BLACK),
                (self.dash_x + 10, cy + 2))
            cy += 22

        # Info
        cy += 10
        info_lines = [
            f"Distance : {d.get('distance', 0):6.1f} m",
            f"Speed    : {d.get('ego_speed', 0):6.1f} m/s",
            f"CRI      : {d.get('cri', 0):6.3f}",
            f"Min Dist : {d.get('min_distance', 999):6.1f} m",
        ]
        for line in info_lines:
            self.screen.blit(
                self.font_small.render(line, True, Colors.DARK_GRAY),
                (self.dash_x + 10, cy))
            cy += 18

        # Controls help
        cy = self.height - 60
        help_lines = [
            "SPACE: Pause | ESC: Quit",
            "Scroll: Zoom | +/-: Zoom"
        ]
        for line in help_lines:
            self.screen.blit(
                self.font_tiny.render(line, True, Colors.DARK_GRAY),
                (self.dash_x + 10, cy))
            cy += 15

    def wait_for_close(self):
        """Đợi user đóng cửa sổ sau khi mô phỏng xong"""
        waiting = True

        # Vẽ overlay "SIMULATION COMPLETE"
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 100))
        self.screen.blit(overlay, (0, 0))

        text = self.font_large.render("SIMULATION COMPLETE", True, Colors.WHITE)
        text2 = self.font_medium.render("Press ESC or close window to exit",
                                         True, Colors.LIGHT_GRAY)
        self.screen.blit(text,
                          (self.bev_width//2 - text.get_width()//2,
                           self.height//2 - 30))
        self.screen.blit(text2,
                          (self.bev_width//2 - text2.get_width()//2,
                           self.height//2 + 10))
        pygame.display.flip()

        while waiting:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    waiting = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        waiting = False
            self.clock.tick(30)

        pygame.quit()