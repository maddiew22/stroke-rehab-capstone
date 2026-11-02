import asyncio
import struct
import math
import time
import pygame

from bleak import BleakClient, BleakScanner

# BLUETOOTH SETTINGS

BT_DEVICE_NAME = "Exoskeleton_ESP32"

SERVICE_UUID = "12345678-1234-1234-1234-1234567890AB"

# Python -> ESP32: desired elbow angle
REFERENCE_UUID = "12345678-1234-1234-1234-1234567890AC"

# ESP32 -> Python: measured encoder angle
FEEDBACK_UUID = "12345678-1234-1234-1234-1234567890AD"

BT_SEND_INTERVAL = 0.02  # Send reference at 50 Hz

# How long encoder feedback can go missing before it is considered stale
FEEDBACK_TIMEOUT = 0.5


# CONFIGURATION

pygame.init()

WIDTH, HEIGHT = 900, 600

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Elbow Rehabilitation Prototype")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 28)

# Elbow flexion convention:
# 0 degrees = fully extended
# Positive angles = elbow flexion

MIN_ANGLE = 0.0
MAX_ANGLE = 140.0

CYCLE_TIME = 12
ARM_LENGTH = 150
FPS = 60

# Shared storage for the latest encoder feedback.
# The BLE callback updates this dictionary asynchronously.
feedback = {
    "angle": None,
    "last_update": None
}


# SEND REFERENCE ANGLE TO ESP32

async def send_reference_angle(client, theta_ref):

    if client is None or not client.is_connected:
        return

    # Convert reference from degrees to radians
    theta_ref_rad = math.radians(theta_ref)

    # Enforce reference angle limits
    theta_ref_rad = max(
        math.radians(MIN_ANGLE),
        min(math.radians(MAX_ANGLE), theta_ref_rad)
    )

    # ESP32 expects a string containing the angle in radians
    message = f"{theta_ref_rad:.4f}"

    try:
        await client.write_gatt_char(
            REFERENCE_UUID,
            message.encode("utf-8"),
            response=True
        )

    except Exception as e:
        print(f"Bluetooth transmission error: {e}")


# REFERENCE TRAJECTORY

def reference_trajectory(t):
    """
    Smooth repeating elbow flexion-extension trajectory.

    Returns:
        theta_ref: desired angle (degrees)
        omega_ref: desired angular velocity (degrees/second)
    """

    midpoint = (MIN_ANGLE + MAX_ANGLE) / 2
    amplitude = (MAX_ANGLE - MIN_ANGLE) / 2

    theta_ref = midpoint - amplitude * math.cos(
        2 * math.pi * t / CYCLE_TIME
    )

    omega_ref = (
        amplitude * (2 * math.pi / CYCLE_TIME)
        * math.sin(2 * math.pi * t / CYCLE_TIME)
    )

    return theta_ref, omega_ref


# RECEIVE ENCODER FEEDBACK FROM ESP32

def feedback_handler(sender, data):
    """Decode ESP32's 4-byte little-endian float (elbow angle in radians)."""

    # Print the raw notification occasionally so the terminal remains readable.
    now = time.monotonic()

    if len(data) != 4:
        print(f"Unexpected BLE feedback size: {len(data)} bytes; raw={data.hex()}")
        return

    try:
        angle_rad = struct.unpack("<f", data)[0]

        if not math.isfinite(angle_rad):
            print(f"Invalid encoder float received: {angle_rad}")
            return

        angle_deg = math.degrees(angle_rad)

        # Keep the raw value visible when it is outside the expected display range.
        if not MIN_ANGLE <= angle_deg <= MAX_ANGLE:
            print(
                f"Encoder value outside expected range: "
                f"{angle_rad:.4f} rad = {angle_deg:.2f} deg"
            )
            return

        feedback["angle"] = angle_deg
        feedback["last_update"] = now

        if now - feedback.get("last_print", 0.0) >= 1.0:
            print(
                f"Encoder feedback: {angle_rad:.4f} rad "
                f"({angle_deg:.2f} deg)"
            )
            feedback["last_print"] = now

    except (struct.error, ValueError) as e:
        print(f"Feedback decoding error: {e}")


# ARM DRAWING FUNCTION

def draw_arm(surface, elbow, angle_deg, color, length):

    ex, ey = elbow
    angle = math.radians(angle_deg)

    dx = math.sin(angle)
    dy = math.cos(angle)
    px = math.cos(angle)
    py = -math.sin(angle)

    def point(distance, offset=0):
        return (
            int(ex + distance * dx + offset * px),
            int(ey + distance * dy + offset * py)
        )

    outline = (100, 65, 50)
    skin = (244, 183, 143)
    highlight = (255, 215, 184)
    shading = (205, 137, 105)

    # Upper arm
    shoulder = (ex, ey - 135)

    pygame.draw.line(surface, outline, shoulder, (ex, ey), 54)
    pygame.draw.line(surface, skin, shoulder, (ex, ey), 46)

    pygame.draw.circle(surface, outline, shoulder, 27)
    pygame.draw.circle(surface, skin, shoulder, 23)

    # Forearm
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

    # Forearm highlight
    pygame.draw.line(
        surface,
        highlight,
        point(25, 3),
        point(length * 0.78, 3),
        3
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
            1
        )

    # Elbow connection
    pygame.draw.circle(surface, outline, (ex, ey), 23)
    pygame.draw.circle(surface, skin, (ex, ey), 19)

    pygame.draw.arc(
        surface,
        shading,
        (ex - 17, ey - 17, 34, 34),
        math.radians(10),
        math.radians(170),
        3
    )

    pygame.draw.ellipse(
        surface,
        highlight,
        (ex - 9, ey - 11, 12, 8)
    )

    pygame.draw.arc(
        surface,
        shading,
        (ex - 8, ey + 3, 16, 10),
        math.radians(190),
        math.radians(340),
        2
    )


# MAIN GAME LOOP

async def main():

    client = None

    print("Searching for Exoskeleton_ESP32...")

    try:
        device = await BleakScanner.find_device_by_name(
            BT_DEVICE_NAME,
            timeout=10.0
        )

        if device is not None:

            client = BleakClient(device)

            await client.connect()

            if client.is_connected:

                print("Connected to ESP32 via BLE!")

                # Subscribe to encoder feedback notifications
                await client.start_notify(
                    FEEDBACK_UUID,
                    feedback_handler
                )

                print("Subscribed to encoder feedback.")

        else:
            print("ESP32 not found. Running without Bluetooth.")

    except Exception as e:
        print(f"Bluetooth connection failed: {e}")
        client = None

    # INITIALIZATION

    running = True

    start_time = time.monotonic()

    last_bt_send = 0.0

    try:

        while running:

            dt = clock.tick(FPS) / 1000.0

            current_time = time.monotonic()

            elapsed = current_time - start_time

            # GENERATE REFERENCE TRAJECTORY

            theta_ref, omega_ref = reference_trajectory(elapsed)

            # CHECK ENCODER FEEDBACK

            theta_actual = feedback["angle"]
            last_feedback_time = feedback["last_update"]

            feedback_age = (
                current_time - last_feedback_time
                if last_feedback_time is not None
                else None
            )

            feedback_valid = (
                theta_actual is not None
                and feedback_age is not None
                and feedback_age < FEEDBACK_TIMEOUT
            )

            # STREAM REFERENCE TO ESP32 WHILE CONNECTED
            # Do not make reference transmission depend on receiving
            # feedback: that creates a startup deadlock.
            if client is not None and client.is_connected:

                if current_time - last_bt_send >= BT_SEND_INTERVAL:

                    await send_reference_angle(client, theta_ref)
                    last_bt_send = current_time

            # PROCESS PYGAME INPUT

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:

                    if event.key == pygame.K_ESCAPE:
                        running = False

            # CALCULATE TRACKING ERROR

            if feedback_valid:

                error = theta_ref - theta_actual

                tracking = abs(error) <= 8.0

            else:

                error = None
                tracking = False

            # DRAW INTERFACE

            screen.fill((245, 247, 250))

            # Titles
            screen.blit(
                font.render(
                    "Reference arm",
                    True,
                    (30, 90, 200)
                ),
                (50, 30)
            )

            screen.blit(
                font.render(
                    "Patient arm",
                    True,
                    (200, 60, 60)
                ),
                (480, 30)
            )

            # Draw reference arm
            draw_arm(
                screen,
                (200, 230),
                theta_ref,
                (30, 90, 200),
                ARM_LENGTH
            )

            # Draw actual arm only when feedback is valid
            if feedback_valid:

                draw_arm(
                    screen,
                    (620, 230),
                    theta_actual,
                    (200, 60, 60),
                    ARM_LENGTH
                )

            # DISPLAY ANGLE INFORMATION

            lines = [
                f"Reference angle: {theta_ref:.1f} deg",
                (
                    f"Actual encoder angle: {theta_actual:.1f} deg"
                    if feedback_valid
                    else "Actual encoder angle: Waiting for feedback..."
                ),
                (
                    f"Tracking error: {error:.1f} deg"
                    if error is not None
                    else "Tracking error: --"
                )
            ]

            for i, line in enumerate(lines):

                screen.blit(
                    font.render(line, True, (35, 35, 35)),
                    (50, 410 + i * 30)
                )

            # BLUETOOTH STATUS

            bt_connected = (
                client is not None
                and client.is_connected
            )

            bt_status = (
                "Bluetooth: CONNECTED"
                if bt_connected
                else "Bluetooth: DISCONNECTED"
            )

            bt_color = (
                (30, 150, 70)
                if bt_connected
                else (200, 60, 60)
            )

            screen.blit(
                font.render(bt_status, True, bt_color),
                (50, 530)
            )

            # ENCODER FEEDBACK STATUS

            if feedback_valid:

                feedback_text = "Encoder feedback: RECEIVING"
                feedback_color = (30, 150, 70)

            else:

                feedback_text = "Encoder feedback: NOT RECEIVING"
                feedback_color = (200, 60, 60)

            screen.blit(
                font.render(
                    feedback_text,
                    True,
                    feedback_color
                ),
                (480, 490)
            )

            # TRACKING STATUS

            if not feedback_valid:

                tracking_text = "Tracking: WAITING FOR FEEDBACK"

            elif tracking:

                tracking_text = "Tracking: GOOD"

            else:

                tracking_text = "Tracking: NEEDS IMPROVEMENT"

            screen.blit(
                font.render(
                    tracking_text,
                    True,
                    (35, 35, 35)
                ),
                (480, 530)
            )

            pygame.display.flip()

            # Allow BLE callbacks to be processed
            await asyncio.sleep(0)

    finally:

        if client is not None and client.is_connected:

            try:
                await client.stop_notify(FEEDBACK_UUID)
            except Exception:
                pass

            await client.disconnect()

            print("Bluetooth connection closed.")

        pygame.quit()

# RUN APPLICATION

asyncio.run(main())
