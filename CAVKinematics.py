import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Enhanced Center Articulated Vehicle Simulation")

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

# Vehicle Parameters
l1 = 50  # distance from joint to front center
l2 = 50  # distance from joint to rear center

# Simulation Parameters
dt = 0.1  # time step
clock = pygame.time.Clock()

# Max Values
MAX_DEFLECTION_ANGLE = math.radians(38)  # articulation max in radians (±38 degrees)
MAX_SPEED = 20.0  # max forward speed
FRICTION = 0.02  # friction coefficient to gradually reduce speed

# Initial State
x1, y1 = WIDTH // 2, HEIGHT // 2  # front center position
theta1 = 0  # orientation angle of the front part
gamma = 0  # articulation angle
v = 0  # initial velocity
gamma_rate = 0  # initial rate of articulation angle change

# Control parameters
velocity_increment = 2.0  # acceleration rate for velocity
gamma_rate_increment = 0.1  # change rate for articulation angle

def draw_vehicle(x, y, theta, gamma):
    # Calculate positions of front and rear parts based on theta and gamma
    front_x = x + l1 * math.cos(theta)
    front_y = y + l1 * math.sin(theta)
    rear_x = x - l2 * math.cos(theta + gamma)
    rear_y = y - l2 * math.sin(theta + gamma)
    
    # Draw front and rear parts
    pygame.draw.line(screen, BLUE, (rear_x, rear_y), (x, y), 5)  # Rear segment
    pygame.draw.line(screen, RED, (x, y), (front_x, front_y), 5)  # Front segment
    pygame.draw.circle(screen, BLACK, (int(x), int(y)), 5)  # Joint

def update_state(x, y, theta, gamma, v, gamma_rate):
    # Apply friction to gradually reduce forward speed
    if abs(v) > 0.1:
        v -= FRICTION * v / abs(v)  # Reduce speed in proportion to its current value
    else:
        v = 0  # Stop the vehicle when speed is very low
    
    # Cap the forward velocity at max speed
    if v > MAX_SPEED:
        v = MAX_SPEED
    elif v < -MAX_SPEED:
        v = -MAX_SPEED
    
    # Calculate L (effective length)
    L = l2 + l1 * math.cos(gamma)
    
    # Update position and angle based on kinematics
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += (-v * math.sin(gamma)) / L * dt
    gamma += gamma_rate * dt
    
    # Apply max deflection angle constraint
    if gamma > MAX_DEFLECTION_ANGLE:
        gamma = MAX_DEFLECTION_ANGLE
    elif gamma < -MAX_DEFLECTION_ANGLE:
        gamma = -MAX_DEFLECTION_ANGLE
    
    return x, y, theta, gamma, v

# Main simulation loop
running = True
while running:
    screen.fill(WHITE)
    
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    # Key press handling
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:      # Increase forward velocity
        v += velocity_increment
    if keys[pygame.K_DOWN]:    # Decrease forward velocity
        v -= velocity_increment
    if keys[pygame.K_LEFT]:    # Increase articulation angle rate
        gamma_rate = gamma_rate_increment
    elif keys[pygame.K_RIGHT]: # Decrease articulation angle rate
        gamma_rate = -gamma_rate_increment
    else:
        gamma_rate = 0  # Reset gamma_rate to zero if no turn key is pressed

    # Update vehicle state with enhanced realism
    x1, y1, theta1, gamma, v = update_state(x1, y1, theta1, gamma, v, gamma_rate)
    
    # Draw the vehicle
    draw_vehicle(x1, y1, theta1, gamma)
    
    # Display the screen
    pygame.display.flip()
    
    # Control frame rate
    clock.tick(30)

pygame.quit()
