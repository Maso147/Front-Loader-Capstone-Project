import pygame
import math
import sys

# Initialize Pygame
pygame.init()

# Constants
WIDTH = 800
HEIGHT = 600
FPS = 60
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

class FrontLoader:
    def __init__(self, x, y):
        # Basic dimensions
        self.front_length = 100  # Length of front segment
        self.rear_length = 100   # Length of rear segment
        self.width = 40         # Width of each segment
        
        # Position and angles
        self.x = x              # Pivot point x
        self.y = y              # Pivot point y
        self.heading = 0        # Overall heading in degrees
        self.articulation = 0   # Articulation angle in degrees
        
        # Movement
        self.speed = 0
        self.max_speed = 4
        self.max_articulation = 45
        
    def update(self, keys):
        # Steering
        if keys[pygame.K_LEFT]:
            self.articulation = min(self.articulation + 2, self.max_articulation)
        if keys[pygame.K_RIGHT]:
            self.articulation = max(self.articulation - 2, -self.max_articulation)
        
        # Speed control
        if keys[pygame.K_UP]:
            self.speed = min(self.speed + 0.2, self.max_speed)
        elif keys[pygame.K_DOWN]:
            self.speed = max(self.speed - 0.2, -self.max_speed)
        else:
            self.speed *= 0.95  # Friction
            
        # Move based on current speed and heading
        if self.speed != 0:
            # Update heading based on articulation
            if self.articulation != 0:
                # Calculate turning radius and heading change
                turn_radius = self.front_length / math.tan(math.radians(abs(self.articulation)))
                heading_change = math.degrees(self.speed / turn_radius)
                self.heading += heading_change if self.articulation > 0 else -heading_change
            
            # Update position
            self.x += self.speed * math.cos(math.radians(self.heading))
            self.y += self.speed * math.sin(math.radians(self.heading))
    
    def draw(self, screen):
        # Calculate angles for front and rear segments
        front_angle = self.heading + self.articulation/2
        rear_angle = self.heading - self.articulation/2
        
        # Calculate endpoints for front segment
        front_x = self.x + self.front_length/2 * math.cos(math.radians(front_angle))
        front_y = self.y + self.front_length/2 * math.sin(math.radians(front_angle))
        
        # Calculate endpoints for rear segment
        rear_x = self.x - self.rear_length/2 * math.cos(math.radians(rear_angle))
        rear_y = self.y - self.rear_length/2 * math.sin(math.radians(rear_angle))
        
        # Draw segments
        def draw_segment(start_x, start_y, angle, length, color):
            # Calculate corners of the rectangle
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
        
        # Draw both segments
        draw_segment(rear_x, rear_y, rear_angle, self.rear_length, BLUE)
        draw_segment(front_x, front_y, front_angle, self.front_length, RED)
        
        # Draw pivot point
        pygame.draw.circle(screen, BLACK, (int(self.x), int(self.y)), 4)
        
        # Draw direction indicator
        pygame.draw.line(screen, BLACK,
                        (self.x, self.y),
                        (self.x + 20 * math.cos(math.radians(self.heading)),
                         self.y + 20 * math.sin(math.radians(self.heading))), 2)

def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Front Loader Top-Down View")
    clock = pygame.time.Clock()
    
    loader = FrontLoader(WIDTH//2, HEIGHT//2)
    
    # Main game loop
    running = True
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Update
        keys = pygame.key.get_pressed()
        loader.update(keys)
        
        # Draw
        screen.fill(WHITE)
        loader.draw(screen)
        
        # Draw instructions
        font = pygame.font.Font(None, 36)
        text = font.render("up/down = move, left/right = steer", True, BLACK)
        screen.blit(text, (10, 10))
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()