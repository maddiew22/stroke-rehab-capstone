
import asyncio
import math
import struct
import time

import flet as ft
import flet.canvas as cv
from bleak import BleakClient, BleakScanner


BT_DEVICE_NAME = "Exoskeleton_ESP32"
SERVICE_UUID = "12345678-1234-1234-1234-1234567890AB"
REFERENCE_UUID = "12345678-1234-1234-1234-1234567890AC"
FEEDBACK_UUID = "12345678-1234-1234-1234-1234567890AD"

BT_SEND_INTERVAL = 0.02
FEEDBACK_TIMEOUT = 0.5

MIN_ANGLE = 0.0
MAX_ANGLE = 140.0
CYCLE_TIME = 15.0

ARM_LENGTH = 125
UI_FPS = 30
IMAGE_UPDATE_THRESHOLD = 0.1


feedback = {
    "angle": None,
    "last_update": None,
}


def reference_trajectory(t):
    midpoint = (MIN_ANGLE + MAX_ANGLE) / 2
    amplitude = (MAX_ANGLE - MIN_ANGLE) / 2

    theta_ref = midpoint - amplitude * math.cos(
        2 * math.pi * t / CYCLE_TIME
    )

    omega_ref = (
        amplitude
        * (2 * math.pi / CYCLE_TIME)
        * math.sin(2 * math.pi * t / CYCLE_TIME)
    )

    return theta_ref, omega_ref


def make_arm_shapes(angle_deg):
    shapes = []

    ex, ey = 190, 155

    angle = math.radians(angle_deg)

    dx = math.sin(angle)
    dy = math.cos(angle)

    px = math.cos(angle)
    py = -math.sin(angle)

    def point(distance, offset=0):
        return (
            ex + distance * dx + offset * px,
            ey + distance * dy + offset * py,
        )

    outline = "#644132"
    skin = "#F4B78F"
    highlight = "#FFD7B8"
    shading = "#CD8969"

    outline_paint = ft.Paint(
        color=outline,
        stroke_width=54,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    upper_arm_paint = ft.Paint(
        color=skin,
        stroke_width=46,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    forearm_outline_paint = ft.Paint(
        color=outline,
        stroke_width=38,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    forearm_paint = ft.Paint(
        color=skin,
        stroke_width=30,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    highlight_paint = ft.Paint(
        color=highlight,
        stroke_width=3,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    shoulder = (ex, ey - 130)

    shapes.append(
        cv.Line(
            shoulder[0], shoulder[1],
            ex, ey,
            paint=outline_paint,
        )
    )

    shapes.append(
        cv.Line(
            shoulder[0], shoulder[1],
            ex, ey,
            paint=upper_arm_paint,
        )
    )

    shapes.append(
        cv.Circle(
            shoulder[0], shoulder[1], 27,
            paint=ft.Paint(color=outline),
        )
    )

    shapes.append(
        cv.Circle(
            shoulder[0], shoulder[1], 23,
            paint=ft.Paint(color=skin),
        )
    )

    shapes.append(
        cv.Line(
            *point(0),
            *point(ARM_LENGTH),
            paint=forearm_outline_paint,
        )
    )

    shapes.append(
        cv.Line(
            *point(0),
            *point(ARM_LENGTH),
            paint=forearm_paint,
        )
    )

    shapes.append(
        cv.Line(
            *point(25, 3),
            *point(ARM_LENGTH * 0.78, 3),
            paint=highlight_paint,
        )
    )

    hand_outline_paint = ft.Paint(
        color=outline,
        stroke_width=25,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    hand_paint = ft.Paint(
        color=skin,
        stroke_width=17,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    shapes.append(
        cv.Line(
            *point(ARM_LENGTH - 2),
            *point(ARM_LENGTH + 35),
            paint=hand_outline_paint,
        )
    )

    shapes.append(
        cv.Line(
            *point(ARM_LENGTH - 2),
            *point(ARM_LENGTH + 35),
            paint=hand_paint,
        )
    )

    finger_paint = ft.Paint(
        color=shading,
        stroke_width=2,
        style=ft.PaintingStyle.STROKE,
        stroke_cap=ft.StrokeCap.ROUND,
    )

    for offset in [-5, 1, 7]:
        shapes.append(
            cv.Line(
                *point(ARM_LENGTH + 20, offset),
                *point(ARM_LENGTH + 27, offset),
                paint=finger_paint,
            )
        )

    shapes.append(
        cv.Circle(
            ex, ey, 23,
            paint=ft.Paint(color=outline),
        )
    )

    shapes.append(
        cv.Circle(
            ex, ey, 19,
            paint=ft.Paint(color=skin),
        )
    )

    shapes.append(
        cv.Circle(
            ex - 5, ey - 7, 4,
            paint=ft.Paint(color=highlight),
        )
    )

    return shapes


def feedback_handler(sender, data):
    if len(data) != 4:
        print(f"Unexpected BLE feedback size: {len(data)} bytes")
        return

    try:
        angle_rad = struct.unpack("<f", data)[0]

        if not math.isfinite(angle_rad):
            return

        angle_deg = math.degrees(angle_rad)

        if MIN_ANGLE <= angle_deg <= MAX_ANGLE:
            feedback["angle"] = angle_deg
            feedback["last_update"] = time.monotonic()

    except (struct.error, ValueError) as exc:
        print(f"Feedback decoding error: {exc}")


async def send_reference_angle(client, theta_ref):
    if client is None or not client.is_connected:
        return

    theta_ref_rad = math.radians(
        max(MIN_ANGLE, min(MAX_ANGLE, theta_ref))
    )

    message = f"{theta_ref_rad:.4f}"

    try:
        await client.write_gatt_char(
            REFERENCE_UUID,
            message.encode("utf-8"),
            response=True,
        )

    except Exception as exc:
        print(f"Bluetooth transmission error: {exc}")


async def main(page: ft.Page):
    page.title = "Elbow Rehabilitation System"
    page.window.width = 1100
    page.window.height = 800
    page.theme_mode = ft.ThemeMode.LIGHT
    page.bgcolor = "#F3F5F8"
    page.padding = 24

    client = None

    running = False
    has_started = False

    start_time = None
    last_bt_send = 0.0

    theta_ref = MIN_ANGLE
    latest_error = None

    last_reference_drawn = 0.0
    last_patient_drawn = 0.0

    countdown_active = False
    countdown_end = None

    elapsed_before_pause = 0.0

    start_pause_button = None
    start_pause_label = None
    rehabilitation_page_active = False

    title = ft.Text(
        "BendAid: Stroke Rehabilitation Exoskeleton",
        size=26,
        weight=ft.FontWeight.BOLD,
        color="#243447",
    )

    bluetooth_status = ft.Text(
        "Bluetooth: DISCONNECTED",
        color="#C0392B",
        weight=ft.FontWeight.BOLD,
    )

    feedback_status = ft.Text(
        "Encoder feedback: WAITING",
        color="#C0392B",
    )

    tracking_status = ft.Text(
        "Tracking: WAITING FOR FEEDBACK",
        color="#667788",
        weight=ft.FontWeight.BOLD,
    )

    reference_value = ft.Text(
        "0.0°",
        size=25,
        weight=ft.FontWeight.BOLD,
    )

    actual_value = ft.Text(
        "--",
        size=25,
        weight=ft.FontWeight.BOLD,
    )

    error_value = ft.Text(
        "--",
        size=25,
        weight=ft.FontWeight.BOLD,
    )

    reference_canvas = cv.Canvas(
        width=380,
        height=300,
        shapes=make_arm_shapes(0),
    )

    patient_canvas = cv.Canvas(
        width=380,
        height=300,
        shapes=make_arm_shapes(0),
    )

    run_status = ft.Text(
        "Rehabilitation paused",
        color="#667788",
    )

    countdown_number = ft.Text(
        "5",
        size=64,
        weight=ft.FontWeight.BOLD,
        color="#1E5AA8",
    )

    countdown_circle = ft.Container(
        content=countdown_number,
        width=150,
        height=150,
        border=ft.Border(
            left=ft.BorderSide(6, "#1E5AA8"),
            top=ft.BorderSide(6, "#1E5AA8"),
            right=ft.BorderSide(6, "#1E5AA8"),
            bottom=ft.BorderSide(6, "#1E5AA8"),
        ),
        border_radius=75,
        alignment=ft.Alignment(0, 0),
        bgcolor="white",
        visible=False,
    )

    async def connect_ble(e=None):
        nonlocal client

        bluetooth_status.value = "Bluetooth: SEARCHING..."
        bluetooth_status.color = "#B9770E"

        page.update()

        try:
            device = await BleakScanner.find_device_by_name(
                BT_DEVICE_NAME,
                timeout=10.0,
            )

            if device is None:
                bluetooth_status.value = "Bluetooth: DEVICE NOT FOUND"
                bluetooth_status.color = "#C0392B"
                page.update()
                return

            new_client = BleakClient(device)
            await new_client.connect()

            if new_client.is_connected:
                await new_client.start_notify(
                    FEEDBACK_UUID,
                    feedback_handler,
                )

                client = new_client

                bluetooth_status.value = "Bluetooth: CONNECTED"
                bluetooth_status.color = "#218C55"

            else:
                bluetooth_status.value = "Bluetooth: CONNECTION FAILED"
                bluetooth_status.color = "#C0392B"

        except Exception as exc:
            print(f"Bluetooth connection failed: {exc}")

            bluetooth_status.value = "Bluetooth: CONNECTION FAILED"
            bluetooth_status.color = "#C0392B"

        page.update()

    async def toggle_rehabilitation(e):
        nonlocal running, has_started
        nonlocal start_time, countdown_active, countdown_end
        nonlocal elapsed_before_pause

        if running:
            elapsed_before_pause += time.monotonic() - start_time
            running = False
            start_time = None

            start_pause_label.value = "START"
            start_pause_button.icon = ft.Icons.PLAY_ARROW

            countdown_circle.visible = False

            run_status.value = "Rehabilitation paused"
            run_status.color = "#667788"

            page.update()

        elif not countdown_active:
            countdown_active = True
            countdown_end = time.monotonic() + 5

            countdown_number.value = "5"
            countdown_circle.visible = True

            start_pause_button.disabled = True
            start_pause_label.value = "GET READY..."

            run_status.value = "Get ready to begin"
            run_status.color = "#1E5AA8"

            page.update()

    async def return_home(e=None):
        nonlocal client
        nonlocal running, has_started
        nonlocal start_time, theta_ref
        nonlocal last_bt_send, latest_error
        nonlocal last_reference_drawn, last_patient_drawn
        nonlocal countdown_active, countdown_end
        nonlocal elapsed_before_pause, rehabilitation_page_active
        nonlocal start_pause_button, start_pause_label

        running = False
        has_started = False
        rehabilitation_page_active = False

        start_time = None

        countdown_active = False
        countdown_end = None

        elapsed_before_pause = 0.0

        theta_ref = MIN_ANGLE
        latest_error = None

        last_bt_send = 0.0

        last_reference_drawn = 0.0
        last_patient_drawn = 0.0

        feedback["angle"] = None
        feedback["last_update"] = None

        start_pause_button = None
        start_pause_label = None

        if client is not None:
            try:
                if client.is_connected:
                    await client.stop_notify(FEEDBACK_UUID)
                    await client.disconnect()

            except Exception as exc:
                print(f"Bluetooth disconnection error: {exc}")

            client = None

        page.controls.clear()

        page.add(
            ft.Column(
                [
                    title,

                    ft.Text(
                        "Welcome! Select an option to get started.",
                        size=16,
                        color="#667788",
                    ),

                    ft.Container(height=20),

                    ft.FilledButton(
                        "Start Rehabilitation",
                        icon=ft.Icons.PLAY_ARROW,
                        width=280,
                        on_click=show_rehabilitation,
                    ),

                    ft.OutlinedButton(
                        "View Progress",
                        icon=ft.Icons.SHOW_CHART,
                        width=280,
                        on_click=lambda e: show_placeholder(
                            "Rehabilitation Progress"
                        ),
                    ),

                    ft.OutlinedButton(
                        "Physio Settings",
                        icon=ft.Icons.SETTINGS,
                        width=280,
                        on_click=lambda e: show_placeholder("Settings"),
                    ),
                ],
                horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                alignment=ft.MainAxisAlignment.CENTER,
                expand=True,
                spacing=16,
            )
        )

        page.update()

    def show_placeholder(name):
        page.controls.clear()

        page.add(
            ft.Column(
                [
                    ft.Row(
                        [
                            ft.IconButton(
                                ft.Icons.ARROW_BACK,
                                on_click=return_home,
                            ),

                            ft.Text(
                                name,
                                size=26,
                                weight=ft.FontWeight.BOLD,
                            ),
                        ]
                    ),

                    ft.Text(f"{name} content will go here."),
                ],
                expand=True,
            )
        )

        page.update()

    def show_rehabilitation(e=None):
        nonlocal running, has_started
        nonlocal start_time, theta_ref
        nonlocal last_bt_send, latest_error
        nonlocal last_reference_drawn, last_patient_drawn
        nonlocal countdown_active, countdown_end
        nonlocal elapsed_before_pause, start_pause_button
        nonlocal start_pause_label, rehabilitation_page_active

        running = False
        has_started = False
        rehabilitation_page_active = True

        start_time = None

        theta_ref = MIN_ANGLE

        countdown_active = False
        countdown_end = None

        elapsed_before_pause = 0.0

        last_bt_send = 0.0
        latest_error = None

        last_reference_drawn = 0.0
        last_patient_drawn = 0.0

        feedback["angle"] = None
        feedback["last_update"] = None

        reference_canvas.shapes = make_arm_shapes(0)
        patient_canvas.shapes = make_arm_shapes(0)

        reference_value.value = "0.0°"
        actual_value.value = "--"
        error_value.value = "--"

        start_pause_label = ft.Text("START")

        start_pause_button = ft.FilledButton(
            content=start_pause_label,
            icon=ft.Icons.PLAY_ARROW,
            on_click=toggle_rehabilitation,
        )

        page.controls.clear()

        reference_card = ft.Container(
            content=ft.Column(
                [
                    ft.Text(
                        "REFERENCE ARM",
                        color="#1E5AA8",
                        weight=ft.FontWeight.BOLD,
                    ),

                    reference_canvas,

                    ft.Text(
                        "Target elbow angle",
                        color="#667788",
                    ),

                    reference_value,
                ],
                horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                spacing=8,
            ),
            bgcolor="white",
            border_radius=14,
            padding=16,
            expand=True,
        )

        patient_card = ft.Container(
            content=ft.Column(
                [
                    ft.Text(
                        "PATIENT ARM",
                        color="#B83232",
                        weight=ft.FontWeight.BOLD,
                    ),

                    patient_canvas,

                    ft.Text(
                        "Measured elbow angle",
                        color="#667788",
                    ),

                    actual_value,
                ],
                horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                spacing=8,
            ),
            bgcolor="white",
            border_radius=14,
            padding=16,
            expand=True,
        )

        page.add(
            ft.Column(
                [
                    ft.Row(
                        [
                            ft.IconButton(
                                ft.Icons.ARROW_BACK,
                                on_click=return_home,
                            ),

                            title,
                        ]
                    ),

                    ft.Stack(
                        [
                            ft.Row(
                                [
                                    reference_card,
                                    patient_card,
                                ],
                                spacing=16,
                                expand=True,
                            ),
                            ft.Container(
                                content=countdown_circle,
                                alignment=ft.Alignment(0, 0),
                                expand=True,
                            ),
                        ],
                        expand=True,
                    ),

                    ft.Container(
                        content=ft.Row(
                            [
                                ft.Column(
                                    [
                                        ft.Text(
                                            "TRACKING ERROR",
                                            color="#667788",
                                        ),

                                        error_value,
                                    ],
                                    spacing=4,
                                ),

                                ft.Column(
                                    [
                                        bluetooth_status,
                                        feedback_status,
                                        tracking_status,
                                    ],
                                    spacing=5,
                                ),
                            ],
                            alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
                        ),
                        bgcolor="white",
                        border_radius=14,
                        padding=16,
                    ),

                    ft.Row(
                        [
                            start_pause_button,

                            ft.OutlinedButton(
                                "Connect Bluetooth",
                                icon=ft.Icons.BLUETOOTH,
                                on_click=connect_ble,
                            ),

                            run_status,
                        ],
                        wrap=True,
                    ),
                ],
                expand=True,
                spacing=16,
            )
        )

        page.update()

    page.add(
        ft.Column(
            [
                title,

                ft.Text(
                    "Welcome! Select an option to get started.",
                    size=16,
                    color="#667788",
                ),

                ft.Container(height=20),

                ft.FilledButton(
                    "Start Rehabilitation",
                    icon=ft.Icons.PLAY_ARROW,
                    width=280,
                    on_click=show_rehabilitation,
                ),

                ft.OutlinedButton(
                    "View Progress",
                    icon=ft.Icons.SHOW_CHART,
                    width=280,
                    on_click=lambda e: show_placeholder(
                        "Rehabilitation Progress"
                    ),
                ),

                ft.OutlinedButton(
                    "Physio Settings",
                    icon=ft.Icons.SETTINGS,
                    width=280,
                    on_click=lambda e: show_placeholder("Settings"),
                ),
            ],
            horizontal_alignment=ft.CrossAxisAlignment.CENTER,
            alignment=ft.MainAxisAlignment.CENTER,
            expand=True,
            spacing=16,
        )
    )

    try:
        while True:
            now = time.monotonic()

            if rehabilitation_page_active:

                if countdown_active:
                    remaining = max(0, math.ceil(countdown_end - now))

                    if remaining > 0:
                        countdown_number.value = str(remaining)

                    else:
                        countdown_active = False
                        running = True
                        has_started = True

                        start_time = now

                        countdown_circle.visible = False

                        start_pause_button.disabled = False
                        start_pause_label.value = "PAUSE"
                        start_pause_button.icon = ft.Icons.PAUSE

                        run_status.value = "Rehabilitation running"
                        run_status.color = "#218C55"

                if running and start_time is not None:
                    elapsed = elapsed_before_pause + (now - start_time)
                    theta_ref, _ = reference_trajectory(elapsed)

                if (
                    client is not None
                    and client.is_connected
                    and has_started
                    and now - last_bt_send >= BT_SEND_INTERVAL
                ):
                    await send_reference_angle(client, theta_ref)
                    last_bt_send = now

                actual = feedback["angle"]
                last_update = feedback["last_update"]

                feedback_valid = (
                    actual is not None
                    and last_update is not None
                    and now - last_update < FEEDBACK_TIMEOUT
                )

                if feedback_valid:
                    latest_error = theta_ref - actual
                    tracking = abs(latest_error) <= 8.0
                else:
                    latest_error = None
                    tracking = False

                reference_value.value = f"{theta_ref:.1f}°"

                if abs(theta_ref - last_reference_drawn) >= IMAGE_UPDATE_THRESHOLD:
                    reference_canvas.shapes = make_arm_shapes(theta_ref)
                    last_reference_drawn = theta_ref

                if feedback_valid:
                    actual_value.value = f"{actual:.1f}°"

                    if abs(actual - last_patient_drawn) >= IMAGE_UPDATE_THRESHOLD:
                        patient_canvas.shapes = make_arm_shapes(actual)
                        last_patient_drawn = actual

                    error_value.value = f"{latest_error:+.1f}°"

                    feedback_status.value = "Encoder feedback: RECEIVING"
                    feedback_status.color = "#218C55"

                    tracking_status.value = (
                        "Tracking: GOOD"
                        if tracking
                        else "Tracking: NEEDS IMPROVEMENT"
                    )

                    tracking_status.color = (
                        "#218C55" if tracking else "#B9770E"
                    )

                else:
                    actual_value.value = "Waiting..."
                    error_value.value = "--"

                    feedback_status.value = "Encoder feedback: NOT RECEIVING"
                    feedback_status.color = "#C0392B"

                    tracking_status.value = "Tracking: WAITING FOR FEEDBACK"
                    tracking_status.color = "#667788"

                page.update()

            await asyncio.sleep(1 / UI_FPS)

    finally:
        if client is not None and client.is_connected:
            try:
                await client.stop_notify(FEEDBACK_UUID)
            except Exception:
                pass

            await client.disconnect()


if __name__ == "__main__":
    ft.run(main)