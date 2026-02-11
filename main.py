import os
import sys
import math
import pygame
import numpy as np
import serial
import platform
import threading
import struct

# --- Configuration ---
BAUD_RATE = 230400  # Serial communication speed
TIMEOUT = 0.1       # Serial port timeout in seconds

# Automatically match port based on OS
os_name = platform.system()
if os_name == 'Windows':
    PORT_NAME = 'COM12'  # Default Windows port - adjust as needed
elif os_name == 'Darwin':  # macOS
    PORT_NAME = '/dev/tty.usbserial-1410'  # Default macOS port - adjust as needed
else:  # Linux
    PORT_NAME = '/dev/ttyUSB0'  # Default Linux port - adjust as needed

# SLAM / Map Settings
WINDOW_SIZE = (800, 800)  # Display window size in pixels
MAP_DIM = 2000             # Map dimensions in cells
CELL_SIZE_MM = 20          # Size of each cell in millimeters
POSE_OPTIMIZATION_ITERATIONS = 20  # Number of iterations for pose optimization
CONFIDENCE_FREE = (10, 10, 10)     # RGB values for free space visualization
CONFIDENCE_OCCUPIED = (50, 50, 50) # RGB values for occupied space visualization

# LD06 Performance Parameters
INITIAL_STEP_XY = 30       # Initial step size for XY translation in pose optimization (mm)
INITIAL_STEP_THETA = 0.5   # Initial step size for rotation in pose optimization (degrees)
MIN_OCCUPANCY_THRESHOLD = 0.6  # Threshold for considering a cell as occupied
WEIGHT_OCCUPIED_CELLS = 2.0      # Weight multiplier for occupied cells in scoring
MULTI_RESOLUTION_LEVELS = 3      # Number of resolution levels for multi-resolution optimization
MIN_VALID_POINTS_FOR_UPDATE = 5  # Minimum number of valid points required to update map

# --- LD06 Parser Class with Threading ---

class LD06Lidar:
    """LD06 LiDAR data parser class with background thread"""
    def __init__(self, port, baudrate=230400):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=TIMEOUT)
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            sys.exit()

        self.buffer = bytearray()
        self.latest_scan = []      # Store latest full scan data
        self.new_data_flag = False # Flag indicating if new data is available
        self.running = True
        self.lock = threading.Lock()

        # Start background reading thread
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        scan_data = []
        last_angle = 0
        while self.running:
            if self.ser.in_waiting > 0:
                self.buffer.extend(self.ser.read(self.ser.in_waiting))

            while len(self.buffer) >= 48:
                if self.buffer[0] == 0x54 and self.buffer[1] == 0x2C:
                    packet = self.buffer[:48]
                    del self.buffer[:48]

                    # Parse packet header
                    start_angle = struct.unpack("<H", packet[4:6])[0] / 100.0
                    end_angle = struct.unpack("<H", packet[42:44])[0] / 100.0
                    diff = (end_angle - start_angle) % 360
                    step = diff / 11.0

                    for i in range(12):
                        dist = struct.unpack("<H", packet[6 + i*3 : 8 + i*3])[0]
                        conf = packet[8 + i*3]
                        angle = (start_angle + step * i) % 360

                        # Detect angle wrap-around, indicating end of rotation
                        if angle < last_angle and len(scan_data) > 200:
                            with self.lock:
                                self.latest_scan = list(scan_data)
                                self.new_data_flag = True
                            scan_data = []

                        last_angle = angle
                        scan_data.append((conf, angle, dist))
                else:
                    self.buffer.pop(0)

    def get_scan(self):
        """Get latest data and reset flag"""
        with self.lock:
            if self.new_data_flag:
                self.new_data_flag = False
                return self.latest_scan
            return None

    def close(self):
        self.running = False
        self.ser.close()

# --- Pose Estimator Class ---

class PoseEstimator:
    """
    Estimates the robot's pose (position and orientation) based on LiDAR scan data
    and previous pose estimates using an iterative optimization approach.
    """
    def __init__(self, map_dim, cell_size_mm):
        self.map_w = map_dim
        self.map_h = map_dim
        self.cell_size = cell_size_mm
        self.reset()

    def reset(self):
        """Reset the pose estimator to initial position at center of map"""
        self.x = (self.map_w * self.cell_size) / 2
        self.y = (self.map_h * self.cell_size) / 2
        self.theta = 0.0
        self.last_dx = self.last_dy = self.last_dth = 0.0

    def get_pose(self):
        """Return current estimated pose (x, y, theta)"""
        return self.x, self.y, self.theta

    def optimize_pose(self, scan_points, grid_map):
        """
        Optimize the current pose estimate by maximizing agreement between
        the current scan and the existing map.
        
        Args:
            scan_points: List of (angle, distance) tuples from LiDAR scan
            grid_map: Occupancy grid map to compare against
        """
        if len(scan_points) == 0: return

        # Motion prediction based on previous movement
        self.x += self.last_dx
        self.y += self.last_dy
        self.theta += self.last_dth

        scan_arr = np.array(scan_points)
        angles, dists = scan_arr[:, 0], scan_arr[:, 1]
        local_x, local_y = dists * np.cos(angles), dists * np.sin(angles)

        prev_x, prev_y, prev_th = self.x, self.y, self.theta
        initial_step_xy = INITIAL_STEP_XY
        initial_step_th = np.radians(INITIAL_STEP_THETA * 2)

        for level in range(MULTI_RESOLUTION_LEVELS):
            step_xy = initial_step_xy / (2**level)
            step_th = initial_step_th / (2**level)
            best_score = -float('inf')
            best_pose = (self.x, self.y, self.theta)

            for dth in [-2*step_th, -step_th, 0, step_th, 2*step_th]:
                test_th = self.theta + dth
                cos_th, sin_th = np.cos(test_th), np.sin(test_th)
                for dx in [-step_xy, 0, step_xy]:
                    for dy in [-step_xy, 0, step_xy]:
                        test_x, test_y = self.x + dx, self.y + dy
                        gx = ((local_x * cos_th - local_y * sin_th) + test_x) / self.cell_size
                        gy = ((local_x * sin_th + local_y * cos_th) + test_y) / self.cell_size
                        idx_x, idx_y = gx.astype(int), gy.astype(int)

                        mask = (idx_x>=0)&(idx_x<self.map_w)&(idx_y>=0)&(idx_y<self.map_h)
                        if np.sum(mask) > MIN_VALID_POINTS_FOR_UPDATE:
                            scores = grid_map[idx_x[mask], idx_y[mask]]
                            weighted_score = np.sum(np.where(scores > MIN_OCCUPANCY_THRESHOLD,
                                                            scores * WEIGHT_OCCUPIED_CELLS, scores))
                            if weighted_score > best_score:
                                best_score = weighted_score
                                best_pose = (test_x, test_y, test_th)

            self.x, self.y, self.theta = best_pose

        # Apply damping to motion estimates
        self.last_dx = (self.x - prev_x) * 0.8
        self.last_dy = (self.y - prev_y) * 0.8
        self.last_dth = (self.theta - prev_th) * 0.8

# --- Main SLAM Loop ---

def run_fixed_map_slam():
    """
    Main SLAM loop that processes LiDAR data, estimates robot pose, 
    updates the occupancy grid map, and renders the visualization.
    """
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("LD06 Lidar SLAM (Multithreaded)")

    # Initialize map surface and occupancy grid
    view_surface = pygame.Surface((MAP_DIM, MAP_DIM))
    view_surface.fill((128, 128, 128))
    occupancy_grid = np.full((MAP_DIM, MAP_DIM), 0.5, dtype=np.float32)  # 0.5 = unknown
    trajectory_points = []

    # Camera controls
    zoom_factor, camera_offset_x, camera_offset_y = 1.0, 0, 0
    
    # Initialize pose estimator and LiDAR
    estimator = PoseEstimator(MAP_DIM, CELL_SIZE_MM)
    lidar = LD06Lidar(PORT_NAME, baudrate=BAUD_RATE)

    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)

    running = True
    try:
        while running:
            # 1. Handle keyboard and events (immediate response)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        view_surface.fill((128, 128, 128))
                        occupancy_grid.fill(0.5)
                        trajectory_points.clear()
                        estimator.reset()
                    if event.key == pygame.K_q: zoom_factor = max(zoom_factor * 0.9, 0.1)
                    if event.key == pygame.K_e: zoom_factor = min(zoom_factor * 1.1, 5.0)

            keys = pygame.key.get_pressed()
            pan = 15 / zoom_factor
            if keys[pygame.K_a]: camera_offset_x += pan
            if keys[pygame.K_d]: camera_offset_x -= pan
            if keys[pygame.K_w]: camera_offset_y += pan
            if keys[pygame.K_s]: camera_offset_y -= pan

            # 2. Get radar data parsed in background
            scan = lidar.get_scan()
            num_points = 0

            if scan:
                valid_points = []
                for (conf, angle, distance) in scan:
                    if distance <= 0 or conf < 20: continue
                    valid_points.append((math.radians(angle), distance))

                num_points = len(valid_points)
                if valid_points:
                    # 3. Perform pose estimation
                    estimator.optimize_pose(valid_points, occupancy_grid)
                    curr_x, curr_y, curr_th = estimator.get_pose()
                    rob_px, rob_py = int(curr_x / CELL_SIZE_MM), int(curr_y / CELL_SIZE_MM)
                    trajectory_points.append((rob_px, rob_py))

                    # 4. Update map
                    flash_surf = pygame.Surface((MAP_DIM, MAP_DIM))
                    hits_surf = pygame.Surface((MAP_DIM, MAP_DIM))
                    math_hits_x, math_hits_y = [], []
                    cos_th, sin_th = math.cos(curr_th), math.sin(curr_th)

                    for (angle_rad, dist) in valid_points:
                        gx_mm = (dist * math.cos(angle_rad) * cos_th - dist * math.sin(angle_rad) * sin_th) + curr_x
                        gy_mm = (dist * math.cos(angle_rad) * sin_th + dist * math.sin(angle_rad) * cos_th) + curr_y
                        px, py = int(gx_mm / CELL_SIZE_MM), int(gy_mm / CELL_SIZE_MM)

                        if 0 <= px < MAP_DIM and 0 <= py < MAP_DIM:
                            pygame.draw.line(flash_surf, CONFIDENCE_FREE, (rob_px, rob_py), (px, py), 1)
                            pygame.draw.circle(hits_surf, CONFIDENCE_OCCUPIED, (px, py), 1)
                            math_hits_x.append(px); math_hits_y.append(py)

                    view_surface.blit(flash_surf, (0,0), special_flags=pygame.BLEND_ADD)
                    view_surface.blit(hits_surf, (0,0), special_flags=pygame.BLEND_SUB)
                    if math_hits_x:
                        occupancy_grid[math_hits_x, math_hits_y] = np.minimum(1.0, occupancy_grid[math_hits_x, math_hits_y] + 0.1)

            # 5. Render screen
            screen.fill((40, 40, 40))
            sw, sh = int(MAP_DIM * zoom_factor), int(MAP_DIM * zoom_factor)
            scaled_view = pygame.transform.scale(view_surface, (sw, sh))
            ox = (WINDOW_SIZE[0] - sw) // 2 + camera_offset_x
            oy = (WINDOW_SIZE[1] - sh) // 2 + camera_offset_y
            screen.blit(scaled_view, (ox, oy))

            # Robot trajectory and position
            curr_x, curr_y, _ = estimator.get_pose()
            rob_px, rob_py = int(curr_x / CELL_SIZE_MM), int(curr_y / CELL_SIZE_MM)
            if len(trajectory_points) > 1:
                t_pts = [(int(x*zoom_factor + ox), int(y*zoom_factor + oy)) for x, y in trajectory_points[-500:]]
                pygame.draw.lines(screen, (0, 191, 255), False, t_pts, 2)

            rx, ry = int(rob_px*zoom_factor + ox), int(rob_py*zoom_factor + oy)
            pygame.draw.circle(screen, (255, 0, 0), (rx, ry), max(2, int(5*zoom_factor)))

            # Status display
            status_text = f"LD06 SLAM | FPS: {int(clock.get_fps())} | Points: {num_points} | Q/E: Zoom | WASD: Pan"
            text = font.render(status_text, True, (0, 255, 0))
            screen.blit(text, (10, 10))

            pygame.display.flip()
            clock.tick(60)
    finally:
        lidar.close()
        pygame.quit()

if __name__ == '__main__':
    run_fixed_map_slam()