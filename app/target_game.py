
import math
import random
import pygame

# GAME SETTINGS
WIDTH, HEIGHT = 900, 600
FPS = 60

BG = (225, 242, 255)
DARK = (35, 50, 70)
BLUE = (45, 115, 220)
WHITE = (255, 255, 255)
RED = (225, 55, 65)
GREY = (135, 140, 150)
SKIN = (235, 180, 140)
SKIN_OUTLINE = (190, 130, 100)

TARGET_X = 770
TARGET_RADIUS = 35
TARGET_MOVE_INTERVAL = 5
DART_INTERVAL = 0.45
DART_SPEED = 600
GAME_DURATION = 60

SHOULDER = (205, 280)
ELBOW = (205, 440)
FOREARM_LENGTH = 130

# Forearm angle measured from the positive horizontal axis.
# Negative = upward, positive = downward.
MIN_ANGLE = math.radians(-65)
MAX_ANGLE = math.radians(35)
INITIAL_ANGLE = math.radians(-25)

TRAJECTORY_DURATION = 1.5

# INITIALIZE PYGAME
pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Rehabilitation Dart Target Game")

clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 23, bold=True)
small_font = pygame.font.SysFont("Arial", 17)
big_font = pygame.font.SysFont("Arial", 42, bold=True)


# GAME STATE
def reset_game():
    now = pygame.time.get_ticks() / 1000

    return {
        "score": 0,
        "user_angle": INITIAL_ANGLE,
        "theta_ref": INITIAL_ANGLE,
        "theta_dot_ref": 0.0,
        "trajectory": None,
        "trajectory_start_time": now,
        "trajectory_duration": TRAJECTORY_DURATION,
        "target_y": HEIGHT // 2,
        "last_target_change": now,
        "last_dart": now,
        "darts": [],
        "start_time": now,
        "paused": False,
        "game_over": False,
        "hits": 0,
        "shots": 0,
    }


game = reset_game()


# HELPER FUNCTIONS
def draw_text(text, x, y, font_obj, color=DARK, centered=False):
    image = font_obj.render(text, True, color)
    rect = image.get_rect()

    if centered:
        rect.center = (x, y)
    else:
        rect.topleft = (x, y)

    screen.blit(image, rect)


def get_hand_position(angle):
    """Calculate the hand position from the elbow angle."""
    x = ELBOW[0] + FOREARM_LENGTH * math.cos(angle)
    y = ELBOW[1] + FOREARM_LENGTH * math.sin(angle)
    return int(x), int(y)


def calculate_target_angle(target_y):
    """
    Calculate the angle from the elbow to the target.

    Since the dart travels along the forearm's direction,
    this angle makes the dart's path pass through the target.
    """
    dx = TARGET_X - ELBOW[0]
    dy = target_y - ELBOW[1]

    return math.atan2(dy, dx)


def generate_reference_trajectory(theta_start, theta_end, duration):
    """Generate a cubic smoothstep trajectory and its velocity."""

    def trajectory(t):
        s = max(0.0, min(1.0, t / duration))

        blend = 3 * s**2 - 2 * s**3

        theta_ref = (
            theta_start + (theta_end - theta_start) * blend
        )

        if 0.0 < s < 1.0:
            theta_dot_ref = (
                (theta_end - theta_start)
                * (6 * s - 6 * s**2)
                / duration
            )
        else:
            theta_dot_ref = 0.0

        return theta_ref, theta_dot_ref

    return trajectory


def draw_target(x, y):
    pygame.draw.circle(screen, RED, (x, y), TARGET_RADIUS)
    pygame.draw.circle(screen, WHITE, (x, y), 26)
    pygame.draw.circle(screen, RED, (x, y), 17)
    pygame.draw.circle(screen, WHITE, (x, y), 9)
    pygame.draw.circle(screen, RED, (x, y), 4)


def draw_arm(angle, color, outline, alpha=None):
    """
    Draw an arm at the supplied angle.
    The upper arm is vertical and the forearm rotates.
    """
    hand = get_hand_position(angle)

    # Draw upper arm
    pygame.draw.line(
        screen, color, SHOULDER, ELBOW, 25
    )
    pygame.draw.line(
        screen, outline, SHOULDER, ELBOW, 2
    )

    # Shoulder and elbow joints
    pygame.draw.circle(screen, outline, SHOULDER, 15)
    pygame.draw.circle(screen, color, SHOULDER, 11)

    pygame.draw.circle(screen, outline, ELBOW, 15)
    pygame.draw.circle(screen, color, ELBOW, 11)

    # Forearm
    pygame.draw.line(
        screen, color, ELBOW, hand, 20
    )
    pygame.draw.line(
        screen, outline, ELBOW, hand, 2
    )

    # Hand
    pygame.draw.circle(screen, color, hand, 12)
    pygame.draw.circle(screen, outline, hand, 12, 2)

    # Aiming line
    aim_end = (
        int(hand[0] + 42 * math.cos(angle)),
        int(hand[1] + 42 * math.sin(angle)),
    )

    pygame.draw.line(screen, outline, hand, aim_end, 3)

    return hand


def draw_dart(dart):
    x, y = dart["x"], dart["y"]
    angle = dart["angle"]

    tail = (
        int(x - 15 * math.cos(angle)),
        int(y - 15 * math.sin(angle)),
    )
    tip = (
        int(x + 4 * math.cos(angle)),
        int(y + 4 * math.sin(angle)),
    )

    pygame.draw.line(
        screen, DARK, tail, (int(x), int(y)), 4
    )
    pygame.draw.circle(screen, BLUE, tip, 4)


# MAIN GAME LOOP
running = True

while running:
    dt = min(clock.tick(FPS) / 1000, 0.05)
    now = pygame.time.get_ticks() / 1000

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

            elif event.key == pygame.K_SPACE:
                if not game["game_over"]:
                    game["paused"] = not game["paused"]

            elif event.key == pygame.K_r:
                game = reset_game()

    keys = pygame.key.get_pressed()

    elapsed = now - game["start_time"]
    time_left = max(0, GAME_DURATION - elapsed)

    # UPDATE GAME
    if not game["paused"] and not game["game_over"]:

        if time_left <= 0:
            game["game_over"] = True

        else:
            # USER-CONTROLLED ARM
            # Arrow keys control the patient's virtual arm (replace with exoskeleton position data eventually)
            if keys[pygame.K_UP]:
                game["user_angle"] -= math.radians(65) * dt

            if keys[pygame.K_DOWN]:
                game["user_angle"] += math.radians(65) * dt

            game["user_angle"] = max(
                MIN_ANGLE,
                min(MAX_ANGLE, game["user_angle"])
            )

            # MOVE TARGET AND GENERATE NEW REFERENCE
            if now - game["last_target_change"] >= TARGET_MOVE_INTERVAL:

                theta_start = game["theta_ref"]

                game["target_y"] = random.randint(100, HEIGHT - 100)

                theta_end = calculate_target_angle(
                    game["target_y"]
                )

                theta_end = max(
                    MIN_ANGLE,
                    min(MAX_ANGLE, theta_end)
                )

                game["trajectory"] = generate_reference_trajectory(
                    theta_start,
                    theta_end,
                    game["trajectory_duration"]
                )

                game["trajectory_start_time"] = now
                game["last_target_change"] = now

            # UPDATE REFERENCE ANGLE AND VELOCITY
            if game["trajectory"] is not None:
                trajectory_time = (
                    now - game["trajectory_start_time"]
                )

                (
                    game["theta_ref"],
                    game["theta_dot_ref"]
                ) = game["trajectory"](trajectory_time)

            # AUTOMATICALLY FIRE FROM THE USER-CONTROLLED HAND
            if now - game["last_dart"] >= DART_INTERVAL:

                hand_x, hand_y = get_hand_position(
                    game["user_angle"]
                )

                game["darts"].append({
                    "x": float(hand_x),
                    "y": float(hand_y),
                    "angle": game["user_angle"],
                })

                game["last_dart"] = now
                game["shots"] += 1

            # UPDATE DART POSITIONS AND DETECT HITS
            remaining_darts = []

            for dart in game["darts"]:
                dart["x"] += (
                    DART_SPEED * math.cos(dart["angle"]) * dt
                )
                dart["y"] += (
                    DART_SPEED * math.sin(dart["angle"]) * dt
                )

                dx = dart["x"] - TARGET_X
                dy = dart["y"] - game["target_y"]

                if math.hypot(dx, dy) <= TARGET_RADIUS:
                    game["score"] += 10
                    game["hits"] += 1
                    continue

                if (
                    -20 <= dart["x"] <= WIDTH + 20
                    and -20 <= dart["y"] <= HEIGHT + 20
                ):
                    remaining_darts.append(dart)

            game["darts"] = remaining_darts


    # DRAW GAME
    screen.fill(BG)

    # Ground
    pygame.draw.rect(
        screen, (190, 225, 185),
        (0, 520, WIDTH, HEIGHT - 520)
    )

    # Target travel range
    pygame.draw.line(
        screen, GREY, (TARGET_X, 80), (TARGET_X, 520), 2
    )

    # Target
    draw_target(TARGET_X, game["target_y"])

    # REFERENCE ARM FIRST (GREY)
    # Draw underneath the user arm so both remain visible.
    draw_arm(
        game["theta_ref"],
        GREY,
        (90, 95, 105)
    )

    # USER ARM SECOND (SKIN COLOUR)
    draw_arm(
        game["user_angle"],
        SKIN,
        SKIN_OUTLINE
    )

    # Darts
    for dart in game["darts"]:
        draw_dart(dart)

    # DISPLAY FEEDBACK
    pygame.draw.rect(
        screen, WHITE, (15, 15, 240, 100),
        border_radius=12
    )

    draw_text(f"Score: {game['score']}", 28, 22, font)
    draw_text(
        f"Hits: {game['hits']} / {game['shots']}",
        28, 53, small_font
    )

    pygame.draw.rect(
        screen, WHITE, (WIDTH - 165, 15, 150, 48),
        border_radius=12
    )
    draw_text(f"Time: {int(time_left)}s", WIDTH - 152, 23, font)

    # Controller signals
    error = game["theta_ref"] - game["user_angle"]

    draw_text(
        f"Angle error: {math.degrees(error):.1f} deg",
        20, HEIGHT - 64, small_font, DARK
    )

    draw_text(
        "UP/DOWN: move arm   SPACE: pause   R: restart",
        20, HEIGHT - 38, small_font
    )

    # Pause overlay
    if game["paused"]:
        overlay = pygame.Surface(
            (WIDTH, HEIGHT), pygame.SRCALPHA
        )
        overlay.fill((20, 30, 45, 160))
        screen.blit(overlay, (0, 0))

        draw_text(
            "PAUSED", WIDTH // 2, HEIGHT // 2 - 20,
            big_font, WHITE, centered=True
        )
        draw_text(
            "Press SPACE to resume",
            WIDTH // 2, HEIGHT // 2 + 30,
            font, WHITE, centered=True
        )

    # Game over
    if game["game_over"]:
        overlay = pygame.Surface(
            (WIDTH, HEIGHT), pygame.SRCALPHA
        )
        overlay.fill((20, 30, 45, 190))
        screen.blit(overlay, (0, 0))

        accuracy = (
            100 * game["hits"] / game["shots"]
            if game["shots"] else 0
        )

        draw_text(
            "TIME'S UP!", WIDTH // 2, HEIGHT // 2 - 80,
            big_font, WHITE, centered=True
        )
        draw_text(
            f"Final score: {game['score']}",
            WIDTH // 2, HEIGHT // 2 - 20,
            font, WHITE, centered=True
        )
        draw_text(
            f"Accuracy: {accuracy:.1f}%",
            WIDTH // 2, HEIGHT // 2 + 20,
            font, WHITE, centered=True
        )
        draw_text(
            "Press R to play again",
            WIDTH // 2, HEIGHT // 2 + 70,
            font, WHITE, centered=True
        )

    pygame.display.flip()

pygame.quit()
