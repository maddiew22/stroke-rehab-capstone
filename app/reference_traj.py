
import math
import pygame

# Configuration
pygame.init()

WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Elbow Rehabilitation Prototype")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 28)

# Illustrative development parameters
MIN_ANGLE = 0.0
MAX_ANGLE = 130.0
CYCLE_TIME = 8.0       # Seconds per flexion-extension cycle
ARM_LENGTH = 150
FPS = 60

# Simulation only: replace with encoder measurements later
theta_actual = 30.0
running = True
start_time = pygame.time.get_ticks() / 1000.0


def reference_trajectory(t):
    """Smooth repeating elbow flexion-extension trajectory."""
    midpoint = (MIN_ANGLE + MAX_ANGLE) / 2
    amplitude = (MAX_ANGLE - MIN_ANGLE) / 2

    theta_ref = midpoint - amplitude * math.cos(
        2 * math.pi * t / CYCLE_TIME
    )

    # Analytical derivative, in degrees/second
    omega_ref = (
        amplitude * (2 * math.pi / CYCLE_TIME)
        * math.sin(2 * math.pi * t / CYCLE_TIME)
    )

    return theta_ref, omega_ref



def draw_arm(surface, elbow, angle_deg, color, length):
    """
    Draw a stylized human arm with a vertical upper arm
    and a forearm that rotates around the elbow.
    """

    ex, ey = elbow
    angle = math.radians(angle_deg)

    # Forearm direction and perpendicular direction
    dx = math.sin(angle)
    dy = math.cos(angle)
    px = math.cos(angle)
    py = -math.sin(angle)

    def point(distance, offset=0):
        """Convert forearm-relative coordinates to screen coordinates."""
        return (
            int(ex + distance * dx + offset * px),
            int(ey + distance * dy + offset * py)
        )

    # Colors
    outline = (100, 65, 50)
    skin = (244, 183, 143)
    highlight = (255, 215, 184)
    shading = (205, 137, 105)

    # Upper arm
    shoulder = (ex, ey - 135)

    pygame.draw.line(
        surface, outline, shoulder, (ex, ey), 54
    )
    pygame.draw.line(
        surface, skin, shoulder, (ex, ey), 46
    )

    # Rounded shoulder
    pygame.draw.circle(surface, outline, shoulder, 27)
    pygame.draw.circle(surface, skin, shoulder, 23)

    # Forearm (rotates at elbow)
    forearm_shape = [
        point(0, -19),
        point(length * 0.25, -17),
        point(length * 0.70, -12),
        point(length, -8),
        point(length, 8),
        point(length * 0.70, 13),
        point(length * 0.25, 17),
        point(0, 19),
    ]

    pygame.draw.polygon(surface, outline, forearm_shape)

    forearm_inner = [
        point(2, -15),
        point(length * 0.25, -13),
        point(length * 0.70, -9),
        point(length - 2, -5),
        point(length - 2, 5),
        point(length * 0.70, 9),
        point(length * 0.25, 13),
        point(2, 15),
    ]

    pygame.draw.polygon(surface, skin, forearm_inner)

    # Subtle forearm highlight
    pygame.draw.line(
        surface,
        highlight,
        point(25, 3),
        point(length * 0.78, 3),
        3,
    )

    # Hand and wrist
    hand_shape = [
        point(length - 2, -9),
        point(length + 12, -13),
        point(length + 29, -10),
        point(length + 38, -4),
        point(length + 39, 5),
        point(length + 30, 12),
        point(length + 12, 13),
        point(length - 2, 8),
    ]

    pygame.draw.polygon(surface, outline, hand_shape)

    hand_inner = [
        point(length, -7),
        point(length + 12, -10),
        point(length + 27, -7),
        point(length + 34, -3),
        point(length + 34, 4),
        point(length + 27, 9),
        point(length + 12, 10),
        point(length, 6),
    ]

    pygame.draw.polygon(surface, skin, hand_inner)

    # Knuckle details
    for offset in [-5, 1, 7]:
        pygame.draw.line(
            surface,
            shading,
            point(length + 17, offset),
            point(length + 24, offset),
            1,
        )

    # Rounded elbow connection
    pygame.draw.circle(surface, outline, (ex, ey), 23)
    pygame.draw.circle(surface, skin, (ex, ey), 19)

    # Elbow shading
    pygame.draw.arc(
        surface,
        shading,
        (ex - 17, ey - 17, 34, 34),
        math.radians(10),
        math.radians(170),
        3,
    )

    # Soft highlight
    pygame.draw.ellipse(
        surface,
        highlight,
        (ex - 9, ey - 11, 12, 8),
    )

    # Small elbow crease
    pygame.draw.arc(
        surface,
        shading,
        (ex - 8, ey + 3, 16, 10),
        math.radians(190),
        math.radians(340),
        2,
    )



while running:
    dt = clock.tick(FPS) / 1000.0
    elapsed = pygame.time.get_ticks() / 1000.0 - start_time

    # Generate the target reference
    theta_ref, omega_ref = reference_trajectory(elapsed)

    # Process input (simulated patient movement)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        theta_actual += 50 * dt
    if keys[pygame.K_DOWN]:
        theta_actual -= 50 * dt

    theta_actual = max(
        MIN_ANGLE, min(MAX_ANGLE, theta_actual)
    )

    # 3. Compare reference and simulated actual angle
    error = theta_ref - theta_actual
    tracking = abs(error) <= 8.0

    # Draw interface
    screen.fill((245, 247, 250))

    screen.blit(
        font.render("Reference arm", True, (30, 90, 200)),
        (50, 30)
    )
    screen.blit(
        font.render("Simulated patient arm", True, (200, 60, 60)),
        (480, 30)
    )

    draw_arm(screen, (200, 230), theta_ref, (30, 90, 200), ARM_LENGTH)
    draw_arm(screen, (620, 230), theta_actual, (200, 60, 60), ARM_LENGTH)

    lines = [
        f"Tracking error: {error:.1f} deg",
        "Use UP/DOWN arrows to simulate elbow movement"
    ]

    for i, line in enumerate(lines):
        screen.blit(
            font.render(line, True, (35, 35, 35)),
            (50, 430 + i * 27)
        )

    pygame.display.flip()

pygame.quit()
