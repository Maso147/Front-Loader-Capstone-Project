import pygame
import math
import random
import numpy as np
from typing import List, Tuple

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
YELLOW = (255, 200, 0)
DARK_GRAY = (64, 64, 64)

class FrontLoader:
    def __init__(self):
        # Base position (pivot point of the arm)
        self.base_pos = (300, 500)
        
        # Dimensions
        self.arm_length = 180
        self.bucket_base = 100  # Base of right triangle
        self.bucket_height = 60  # Height of right triangle
        
        # Angles in degrees (0 is horizontal right)
        self.arm_angle = 30  # Main arm angle
        self.bucket_angle = 0  # Relative to arm
        
        # RRT path planning
        self.path: List[Tuple[float, float]] = []
        self.current_path_index = 0
        self.is_executing_path = False
        
        # Movement constraints
        self.arm_limits = (-30, 60)  # Realistic arm motion range
        self.bucket_limits = (-90, 90)  # Bucket rotation limits
        
        # Add vehicle body dimensions
        self.body_width = 200
        self.body_height = 100
        self.wheel_radius = 30

    def calculate_positions(self):
        # Convert angles to radians
        arm_rad = math.radians(self.arm_angle)
        total_bucket_rad = math.radians(self.arm_angle + self.bucket_angle)
        
        # Calculate arm end position (pivot point for bucket)
        arm_end_x = self.base_pos[0] + self.arm_length * math.cos(arm_rad)
        arm_end_y = self.base_pos[1] - self.arm_length * math.sin(arm_rad)
        
        # Calculate bucket points (right triangle)
        # Point 1: Arm end (pivot point)
        p1 = (arm_end_x, arm_end_y)
        
        # Point 2: Base of triangle
        p2 = (
            arm_end_x + self.bucket_base * math.cos(total_bucket_rad),
            arm_end_y - self.bucket_base * math.sin(total_bucket_rad)
        )
        
        # Point 3: Height of triangle (90-degree corner)
        p3 = (
            arm_end_x + self.bucket_height * math.cos(total_bucket_rad + math.pi/2),
            arm_end_y - self.bucket_height * math.sin(total_bucket_rad + math.pi/2)
        )
        
        return {
            'arm_end': (arm_end_x, arm_end_y),
            'bucket_points': [p1, p2, p3]  # Right triangle points
        }

    def plan_rrt_path(self, goal_state):
        """Generate RRT path from current state to goal state"""
        self.path = []
        current_state = (self.arm_angle, self.bucket_angle)
        
        # Simple RRT implementation
        for _ in range(15):  # Generate 15 waypoints
            if random.random() < 0.2:  # 20% chance to sample goal
                sample = goal_state
            else:
                sample = (
                    random.uniform(self.arm_limits[0], self.arm_limits[1]),
                    random.uniform(self.bucket_limits[0], self.bucket_limits[1])
                )
            self.path.append(sample)
        
        self.path.append(goal_state)
        self.current_path_index = 0
        self.is_executing_path = True

    def execute_path_step(self):
        """Execute one step of the planned path"""
        if not self.is_executing_path or self.current_path_index >= len(self.path):
            self.is_executing_path = False
            return
        
        target = self.path[self.current_path_index]
        
        # Smooth interpolation
        self.arm_angle = self.interpolate_angle(self.arm_angle, target[0], 2)
        self.bucket_angle = self.interpolate_angle(self.bucket_angle, target[1], 3)
        
        if self.is_at_target(target):
            self.current_path_index += 1

    def interpolate_angle(self, current: float, target: float, speed: float) -> float:
        diff = target - current
        if abs(diff) < speed:
            return target
        return current + speed * np.sign(diff)

    def is_at_target(self, target: Tuple[float, float]) -> bool:
        tolerance = 1.0
        return (abs(self.arm_angle - target[0]) < tolerance and
                abs(self.bucket_angle - target[1]) < tolerance)

    def draw(self, screen):
        # Draw vehicle body
        body_rect = pygame.Rect(
            self.base_pos[0] - self.body_width//2,
            self.base_pos[1] - self.body_height//2,
            self.body_width,
            self.body_height
        )
        pygame.draw.rect(screen, YELLOW, body_rect)
        
        # Draw wheels
        wheel_positions = [
            (self.base_pos[0] - self.body_width//3, self.base_pos[1] + self.body_height//2),
            (self.base_pos[0] + self.body_width//3, self.base_pos[1] + self.body_height//2)
        ]
        for pos in wheel_positions:
            pygame.draw.circle(screen, DARK_GRAY, pos, self.wheel_radius)
        
        # Calculate arm and bucket positions
        positions = self.calculate_positions()
        
        # Draw main arm (thicker line)
        pygame.draw.line(screen, DARK_GRAY, self.base_pos, 
                        positions['arm_end'], 12)
        
        # Draw hydraulic cylinder (simplified)
        cylinder_start = (
            self.base_pos[0] - 20,
            self.base_pos[1] - self.body_height//4
        )
        cylinder_end = (
            positions['arm_end'][0] - 30 * math.cos(math.radians(self.arm_angle)),
            positions['arm_end'][1] + 30 * math.sin(math.radians(self.arm_angle))
        )
        pygame.draw.line(screen, GRAY, cylinder_start, cylinder_end, 8)
        
        # Draw bucket (right triangle)
        pygame.draw.polygon(screen, YELLOW, positions['bucket_points'])
        pygame.draw.polygon(screen, DARK_GRAY, positions['bucket_points'], 2)
        
        # Draw pivot points
        pygame.draw.circle(screen, BLACK, self.base_pos, 6)
        pygame.draw.circle(screen, BLACK, positions['arm_end'], 5)
        
        # Draw 90-degree indicator in bucket (small arc)
        right_angle_pos = positions['bucket_points'][0]  # Pivot point
        arc_radius = 10
        arc_start_angle = self.arm_angle + self.bucket_angle + 90
        pygame.draw.arc(screen, BLACK,
                       (right_angle_pos[0] - arc_radius, right_angle_pos[1] - arc_radius,
                        arc_radius * 2, arc_radius * 2),
                       math.radians(arc_start_angle), math.radians(arc_start_angle + 90), 2)

def main():
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Front Loader Simulation")
    clock = pygame.time.Clock()
    
    loader = FrontLoader()
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    goal_state = (
                        random.uniform(loader.arm_limits[0], loader.arm_limits[1]),
                        random.uniform(loader.bucket_limits[0], loader.bucket_limits[1])
                    )
                    loader.plan_rrt_path(goal_state)
        
        # Handle continuous key presses for manual control
        keys = pygame.key.get_pressed()
        if not loader.is_executing_path:
            if keys[pygame.K_w]:
                loader.arm_angle = min(loader.arm_angle + 1, loader.arm_limits[1])
            if keys[pygame.K_s]:
                loader.arm_angle = max(loader.arm_angle - 1, loader.arm_limits[0])
            if keys[pygame.K_a]:
                loader.bucket_angle = max(loader.bucket_angle - 2, loader.bucket_limits[0])
            if keys[pygame.K_d]:
                loader.bucket_angle = min(loader.bucket_angle + 2, loader.bucket_limits[1])
        
        if loader.is_executing_path:
            loader.execute_path_step()
        
        # Drawing
        screen.fill(WHITE)
        loader.draw(screen)
        
        # Draw instructions
        font = pygame.font.Font(None, 24)
        instructions = [
            "W/S: Move arm up/down",
            "A/D: Rotate bucket",
            "SPACE: Plan random path"
        ]
        for i, text in enumerate(instructions):
            surface = font.render(text, True, BLACK)
            screen.blit(surface, (10, 10 + i * 25))
        
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()