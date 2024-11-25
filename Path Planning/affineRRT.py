import pygame
import math
import numpy as np
import random

# Screen dimensions and settings
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BG_COLOR = (255, 255, 255)
NODE_COLOR = (0, 0, 255)
EDGE_COLOR = (0, 255, 0)
TARGET_COLOR = (255, 0, 0)
STEP_SIZE = 20

# Vehicle parameters
L_f = 30   # Front wheelbase
L_r = 30   # Rear wheelbase
phi_min = -math.radians(38)  # Min articulation angle (-38°)
phi_max = math.radians(38)   # Max articulation angle (+38°)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("RRT for Center-Articulated Vehicle")
clock = pygame.time.Clock()

# Node definition
class Node:
    def __init__(self, x, y, theta=0, phi=0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta  # Heading angle
        self.phi = phi      # Articulation angle
        self.parent = parent

# Function to create a node based on the kinematics
def simulate_motion(node, v, phi_rate, step_size, dt=0.1):
    """Simulate the motion of an articulated vehicle."""
    x, y, theta, phi = node.x, node.y, node.theta, node.phi
    L = L_f + L_r

    # Simulate motion over the step size
    steps = int(step_size / (v * dt))
    for _ in range(steps):
        # Update articulation angle (bounded by limits)
        phi += phi_rate * dt
        phi = max(phi_min, min(phi_max, phi))  # Enforce articulation limits

        # Update position and heading
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += (v * math.tan(phi) / L) * dt

    return Node(x, y, theta, phi, parent=node)

# Extend the tree
def extend_tree_kinematic(current_node, step_size, target, v=5.0):
    """Extend the RRT tree considering vehicle kinematics."""
    dx = target[0] - current_node.x
    dy = target[1] - current_node.y
    theta_target = np.arctan2(dy, dx)

    # Compute articulation angle rate to steer towards the target
    phi_target = theta_target - current_node.theta
    phi_target = max(phi_min, min(phi_max, phi_target))
    phi_rate = (phi_target - current_node.phi) / step_size  # Simple proportional control

    # Simulate motion
    new_node = simulate_motion(current_node, v, phi_rate, step_size)
    return new_node

# Draw the tree
def draw_tree(screen, nodes):
    for node in nodes:
        if node.parent:
            pygame.draw.line(
                screen, EDGE_COLOR,
                (int(node.x), int(node.y)),
                (int(node.parent.x), int(node.parent.y)), 2
            )
        draw_vehicle(screen, node)

# Draw the articulated vehicle
def draw_vehicle(screen, node):
    """Draw the articulated vehicle."""
    # Front section
    front_x = node.x - L_f * math.cos(node.theta)
    front_y = node.y - L_f * math.sin(node.theta)
    # Rear section
    rear_x = node.x + L_r * math.cos(node.theta + node.phi)
    rear_y = node.y + L_r * math.sin(node.theta + node.phi)

    # Draw front and rear sections
    pygame.draw.line(screen, (0, 0, 255), (node.x, node.y), (front_x, front_y), 2)
    pygame.draw.line(screen, (255, 0, 0), (node.x, node.y), (rear_x, rear_y), 2)
    pygame.draw.circle(screen, (0, 255, 0), (int(node.x), int(node.y)), 3)

# Main function
def main():
    running = True

    # Define start and target points
    start = Node(100, 100)
    target = (700, 500)

    nodes = [start]

    while running:
        screen.fill(BG_COLOR)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Random sampling for target or random point
        if random.random() < 0.1:  # Bias towards target
            sample = target
        else:  # Random sample
            sample = (random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT))

        # Find nearest node
        nearest_node = min(nodes, key=lambda n: np.hypot(n.x - sample[0], n.y - sample[1]))

        # Extend the tree
        new_node = extend_tree_kinematic(nearest_node, STEP_SIZE, sample)

        # Add to tree if new node is valid (within bounds)
        if 0 <= new_node.x <= SCREEN_WIDTH and 0 <= new_node.y <= SCREEN_HEIGHT:
            nodes.append(new_node)

        # Draw target
        pygame.draw.circle(screen, TARGET_COLOR, target, 5)

        # Draw the tree
        draw_tree(screen, nodes)

        # Check if we reached the target
        if np.hypot(new_node.x - target[0], new_node.y - target[1]) < STEP_SIZE:
            print("Target reached!")
            break

        # Update the display
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()