import pygame
import math
import sys
from enum import Enum
import numpy as np
from queue import PriorityQueue

# Initialize Pygame
pygame.init()

# Constants
WIDTH = 1000
HEIGHT = 800
FPS = 60
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
GRAY = (128, 128, 128)
YELLOW = (255, 255, 0)

class PathState(Enum):
    MOVE_TO_GOAL = 1
    FOLLOW_OBSTACLE = 2
    REACHED_GOAL = 3

class FrontLoader:
    def __init__(self, x, y):
        # Basic dimensions and properties
        self.front_length = 100
        self.rear_length = 100
        self.width = 40
        self.x = x
        self.y = y
        self.heading = 0
        self.articulation = 0
        self.speed = 0
        self.max_speed = 4
        self.max_articulation = 45
        
        # Path planning properties
        self.path = []
        self.current_path_index = 0
        self.reached_goal = False
        self.target_point = None
        
    def update(self, target, obstacles):
        if not self.path:
            return
            
        if self.current_path_index >= len(self.path):
            self.speed = 0
            return
            
        # Get current target point from path
        target_x, target_y = self.path[self.current_path_index]
        
        # Calculate angle to target
        dx = target_x - self.x
        dy = target_y - self.y
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Calculate angle difference
        angle_diff = (target_angle - self.heading + 180) % 360 - 180
        
        # Adjust articulation based on angle difference
        self.articulation = max(-self.max_articulation, min(self.max_articulation, angle_diff))
        
        # Set speed based on distance to target
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 20:  # If close to current target point
            self.current_path_index += 1
            self.speed = 0
        else:
            self.speed = min(self.max_speed, distance/50)
        
        # Update position and heading
        if self.speed != 0:
            if self.articulation != 0:
                turn_radius = self.front_length / math.tan(math.radians(abs(self.articulation)))
                heading_change = math.degrees(self.speed / turn_radius)
                self.heading += heading_change if self.articulation > 0 else -heading_change
            
            self.x += self.speed * math.cos(math.radians(self.heading))
            self.y += self.speed * math.sin(math.radians(self.heading))
    
    def draw(self, screen):
        # Drawing code from previous implementation
        front_angle = self.heading + self.articulation/2
        rear_angle = self.heading - self.articulation/2
        
        front_x = self.x + self.front_length/2 * math.cos(math.radians(front_angle))
        front_y = self.y + self.front_length/2 * math.sin(math.radians(front_angle))
        
        rear_x = self.x - self.rear_length/2 * math.cos(math.radians(rear_angle))
        rear_y = self.y - self.rear_length/2 * math.sin(math.radians(rear_angle))
        
        def draw_segment(start_x, start_y, angle, length, color):
            half_width = self.width / 2
            cos_a = math.cos(math.radians(angle))
            sin_a = math.sin(math.radians(angle))
            
            corners = [
                (start_x - length/2 * cos_a - half_width * sin_a,
                 start_y - length/2 * sin_a + half_width * cos_a),
                (start_x + length/2 * cos_a - half_width * sin_a,
                 start_y + length/2 * sin_a + half_width * cos_a),
                (start_x + length/2 * cos_a + half_width * sin_a,
                 start_y + length/2 * sin_a - half_width * cos_a),
                (start_x - length/2 * cos_a + half_width * sin_a,
                 start_y - length/2 * sin_a - half_width * cos_a)
            ]
            
            pygame.draw.polygon(screen, color, corners)
            pygame.draw.polygon(screen, BLACK, corners, 2)
        
        draw_segment(rear_x, rear_y, rear_angle, self.rear_length, BLUE)
        draw_segment(front_x, front_y, front_angle, self.front_length, RED)
        
        pygame.draw.circle(screen, BLACK, (int(self.x), int(self.y)), 4)
        
        # Draw path if available
        if self.path:
            for i in range(len(self.path) - 1):
                pygame.draw.line(screen, GREEN, self.path[i], self.path[i + 1], 2)

class PathPlanner:
    def __init__(self):
        self.bug_state = PathState.MOVE_TO_GOAL
        self.closest_point = None
        self.start_follow_point = None
    
    def check_collision(self, point, obstacles):
        for obs in obstacles:
            if math.dist(point, obs.center) < obs.radius:
                return True
        return False
    
    def get_obstacle_boundary_point(self, point, obstacle, angle):
        x = obstacle.center[0] + obstacle.radius * math.cos(angle)
        y = obstacle.center[1] + obstacle.radius * math.sin(angle)
        return (x, y)
    
    def smooth_path(self, path, obstacles):
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            current = path[i]
            # Look ahead to find furthest valid direct connection
            for j in range(len(path)-1, i, -1):
                next_point = path[j]
                # Check if direct path is collision-free
                collision = False
                for obs in obstacles:
                    if self.line_intersects_circle(current, next_point, obs):
                        collision = True
                        break
                
                if not collision:
                    smoothed.append(next_point)
                    i = j
                    break
            i += 1
            
        return smoothed
    
    def line_intersects_circle(self, start, end, obstacle):
        # Vector from start to end
        d = (end[0] - start[0], end[1] - start[1])
        # Vector from start to circle center
        f = (start[0] - obstacle.center[0], start[1] - obstacle.center[1])
        
        a = d[0]**2 + d[1]**2
        b = 2 * (f[0]*d[0] + f[1]*d[1])
        c = f[0]**2 + f[1]**2 - obstacle.radius**2
        
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return False
            
        t1 = (-b + math.sqrt(discriminant))/(2*a)
        t2 = (-b - math.sqrt(discriminant))/(2*a)
        
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)
    
    def plan_path(self, start, goal, obstacles):
        if not obstacles:
            return [start, goal]
            
        path = [start]
        current = start
        self.bug_state = PathState.MOVE_TO_GOAL
        self.closest_point = None
        
        while math.dist(current, goal) > 20:  # Goal threshold
            if self.bug_state == PathState.MOVE_TO_GOAL:
                # Try moving directly to goal
                next_point = (
                    current[0] + 20 * (goal[0] - current[0]) / math.dist(current, goal),
                    current[1] + 20 * (goal[1] - current[1]) / math.dist(current, goal)
                )
                
                # Check for collision
                collision = False
                for obs in obstacles:
                    if math.dist(next_point, obs.center) < obs.radius + 10:
                        collision = True
                        self.bug_state = PathState.FOLLOW_OBSTACLE
                        self.start_follow_point = current
                        self.closest_point = current
                        break
                        
                if not collision:
                    current = next_point
                    path.append(current)
                    
            elif self.bug_state == PathState.FOLLOW_OBSTACLE:
                # Find closest obstacle
                closest_obs = min(obstacles, key=lambda o: math.dist(current, o.center))
                
                # Follow obstacle boundary
                angle = math.atan2(current[1] - closest_obs.center[1],
                                 current[0] - closest_obs.center[0])
                angle += math.pi/18  # 10 degrees step
                
                next_point = self.get_obstacle_boundary_point(current, closest_obs, angle)
                
                # Check if we can move to goal
                direct_to_goal = True
                for obs in obstacles:
                    if self.line_intersects_circle(next_point, goal, obs):
                        direct_to_goal = False
                        break
                
                if direct_to_goal:
                    self.bug_state = PathState.MOVE_TO_GOAL
                    
                current = next_point
                path.append(current)
                
                # Update closest point to goal if applicable
                if math.dist(current, goal) < math.dist(self.closest_point, goal):
                    self.closest_point = current
                
                # Check if we've circled the obstacle
                if math.dist(current, self.start_follow_point) < 20 and len(path) > 5:
                    current = self.closest_point
                    path.append(current)
                    self.bug_state = PathState.MOVE_TO_GOAL
        
        path.append(goal)
        return self.smooth_path(path, obstacles)

class Obstacle:
    def __init__(self, x, y, radius):
        self.center = (x, y)
        self.radius = radius
    
    def draw(self, screen):
        pygame.draw.circle(screen, GRAY, (int(self.center[0]), int(self.center[1])), self.radius)

def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Front Loader Path Planning")
    clock = pygame.time.Clock()
    
    # Create loader, planner, and obstacles
    loader = FrontLoader(100, HEIGHT//2)
    planner = PathPlanner()
    obstacles = [
        Obstacle(400, 400, 80),
        Obstacle(600, 300, 70),
        Obstacle(500, 600, 90)
    ]
    
    goal = None
    path = None
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    goal = event.pos
                    start = (loader.x, loader.y)
                    path = planner.plan_path(start, goal, obstacles)
                    loader.path = path
                    loader.current_path_index = 0
        
        # Update
        if goal:
            loader.update(goal, obstacles)
        
        # Draw
        screen.fill(WHITE)
        
        # Draw obstacles
        for obs in obstacles:
            obs.draw(screen)
        
        # Draw goal
        if goal:
            pygame.draw.circle(screen, GREEN, goal, 10)
        
        # Draw loader and path
        loader.draw(screen)
        
        # Draw instructions
        font = pygame.font.Font(None, 36)
        text = font.render("Click anywhere to set goal", True, BLACK)
        screen.blit(text, (10, 10))
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
