
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


class RehabilitationPage:

    def __init__(self, page, navigate):
        self.page = page
        self.navigate = navigate

        self.client = None

        self.running = False
        self.has_started = False
        self.active = True

        self.start_time = None
        self.last_bt_send = 0.0

        self.theta_ref = MIN_ANGLE
        self.latest_error = None

        self.last_reference_drawn = 0.0
        self.last_patient_drawn = 0.0

        self.countdown_active = False
        self.countdown_end = None

        self.elapsed_before_pause = 0.0

        self.loop_task = None

        self.feedback = {
            "angle": None,
            "last_update": None,
        }

        self.create_controls()

    def create_controls(self):

        self.title = ft.Text(
            "BendAid: Stroke Rehabilitation Exoskeleton",
            size=26,
            weight=ft.FontWeight.BOLD,
            color="#243447",
        )

        self.bluetooth_status = ft.Text(
            "Bluetooth: DISCONNECTED",
            color="#C0392B",
            weight=ft.FontWeight.BOLD,
        )

        self.feedback_status = ft.Text(
            "Encoder feedback: WAITING",
            color="#C0392B",
        )

        self.tracking_status = ft.Text(
            "Tracking: WAITING FOR FEEDBACK",
            color="#667788",
            weight=ft.FontWeight.BOLD,
        )

        self.reference_value = ft.Text(
            "0.0°",
            size=25,
            weight=ft.FontWeight.BOLD,
        )

        self.actual_value = ft.Text(
            "--",
            size=25,
            weight=ft.FontWeight.BOLD,
        )

        self.error_value = ft.Text(
            "--",
            size=25,
            weight=ft.FontWeight.BOLD,
        )

        self.reference_canvas = cv.Canvas(
            width=380,
            height=300,
            shapes=make_arm_shapes(0),
        )

        self.patient_canvas = cv.Canvas(
            width=380,
            height=300,
            shapes=make_arm_shapes(0),
        )

        self.run_status = ft.Text(
            "Rehabilitation paused",
            color="#667788",
        )

        self.countdown_number = ft.Text(
            "5",
            size=64,
            weight=ft.FontWeight.BOLD,
            color="#1E5AA8",
        )

        self.countdown_circle = ft.Container(
            content=self.countdown_number,
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

        self.start_pause_label = ft.Text("START")

        self.start_pause_button = ft.FilledButton(
            content=self.start_pause_label,
            icon=ft.Icons.PLAY_ARROW,
            on_click=self.toggle_rehabilitation,
        )

    def build(self):

        reference_card = ft.Container(
            content=ft.Column(
                [
                    ft.Text(
                        "REFERENCE ARM",
                        color="#1E5AA8",
                        weight=ft.FontWeight.BOLD,
                    ),

                    self.reference_canvas,

                    ft.Text(
                        "Target elbow angle",
                        color="#667788",
                    ),

                    self.reference_value,
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

                    self.patient_canvas,

                    ft.Text(
                        "Measured elbow angle",
                        color="#667788",
                    ),

                    self.actual_value,
                ],
                horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                spacing=8,
            ),
            bgcolor="white",
            border_radius=14,
            padding=16,
            expand=True,
        )

        return ft.Column(
            [
                ft.Row(
                    [
                        ft.IconButton(
                            ft.Icons.ARROW_BACK,
                            on_click=self.return_home,
                        ),

                        self.title,
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
                            content=self.countdown_circle,
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

                                    self.error_value,
                                ],
                                spacing=4,
                            ),

                            ft.Column(
                                [
                                    self.bluetooth_status,
                                    self.feedback_status,
                                    self.tracking_status,
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
                        self.start_pause_button,

                        ft.OutlinedButton(
                            "Connect Bluetooth",
                            icon=ft.Icons.BLUETOOTH,
                            on_click=self.connect_ble,
                        ),

                        self.run_status,
                    ],
                    wrap=True,
                ),
            ],
            expand=True,
            spacing=16,
        )

    def start(self):
        self.loop_task = asyncio.create_task(
            self.rehabilitation_loop()
        )

    def feedback_handler(self, sender, data):

        if len(data) != 4:
            print(
                f"Unexpected BLE feedback size: {len(data)} bytes"
            )
            return

        try:
            angle_rad = struct.unpack("<f", data)[0]

            if not math.isfinite(angle_rad):
                return

            angle_deg = math.degrees(angle_rad)

            if MIN_ANGLE <= angle_deg <= MAX_ANGLE:
                self.feedback["angle"] = angle_deg
                self.feedback["last_update"] = time.monotonic()

        except (struct.error, ValueError) as exc:
            print(f"Feedback decoding error: {exc}")

    async def send_reference_angle(self, theta_ref):

        if self.client is None or not self.client.is_connected:
            return

        theta_ref_rad = math.radians(
            max(MIN_ANGLE, min(MAX_ANGLE, theta_ref))
        )

        message = f"{theta_ref_rad:.4f}"

        try:
            await self.client.write_gatt_char(
                REFERENCE_UUID,
                message.encode("utf-8"),
                response=True,
            )

        except Exception as exc:
            print(f"Bluetooth transmission error: {exc}")

    async def connect_ble(self, e=None):

        self.bluetooth_status.value = "Bluetooth: SEARCHING..."
        self.bluetooth_status.color = "#B9770E"

        self.page.update()

        try:
            device = await BleakScanner.find_device_by_name(
                BT_DEVICE_NAME,
                timeout=10.0,
            )

            if device is None:
                self.bluetooth_status.value = (
                    "Bluetooth: DEVICE NOT FOUND"
                )
                self.bluetooth_status.color = "#C0392B"
                self.page.update()
                return

            new_client = BleakClient(device)
            await new_client.connect()

            if new_client.is_connected:

                await new_client.start_notify(
                    FEEDBACK_UUID,
                    self.feedback_handler,
                )

                self.client = new_client

                self.bluetooth_status.value = (
                    "Bluetooth: CONNECTED"
                )
                self.bluetooth_status.color = "#218C55"

            else:
                self.bluetooth_status.value = (
                    "Bluetooth: CONNECTION FAILED"
                )
                self.bluetooth_status.color = "#C0392B"

        except Exception as exc:
            print(f"Bluetooth connection failed: {exc}")

            self.bluetooth_status.value = (
                "Bluetooth: CONNECTION FAILED"
            )
            self.bluetooth_status.color = "#C0392B"

        self.page.update()

    async def toggle_rehabilitation(self, e=None):

        if self.running:

            self.elapsed_before_pause += (
                time.monotonic() - self.start_time
            )

            self.running = False
            self.start_time = None

            self.start_pause_label.value = "START"
            self.start_pause_button.icon = ft.Icons.PLAY_ARROW

            self.countdown_circle.visible = False

            self.run_status.value = "Rehabilitation paused"
            self.run_status.color = "#667788"

            self.page.update()

        elif not self.countdown_active:

            self.countdown_active = True
            self.countdown_end = time.monotonic() + 5

            self.countdown_number.value = "5"
            self.countdown_circle.visible = True

            self.start_pause_button.disabled = True
            self.start_pause_label.value = "GET READY..."

            self.run_status.value = "Get ready to begin"
            self.run_status.color = "#1E5AA8"

            self.page.update()

    async def return_home(self, e=None):

        await self.dispose()

        self.navigate("home")

    async def dispose(self):

        self.active = False
        self.running = False
        self.countdown_active = False

        if self.loop_task is not None:
            self.loop_task.cancel()

            try:
                await self.loop_task
            except asyncio.CancelledError:
                pass

            self.loop_task = None

        if self.client is not None:

            try:
                if self.client.is_connected:
                    await self.client.stop_notify(FEEDBACK_UUID)
                    await self.client.disconnect()

            except Exception as exc:
                print(f"Bluetooth disconnection error: {exc}")

            self.client = None

    async def rehabilitation_loop(self):

        while self.active:

            now = time.monotonic()

            if self.countdown_active:

                remaining = max(
                    0,
                    math.ceil(self.countdown_end - now)
                )

                if remaining > 0:
                    self.countdown_number.value = str(remaining)

                else:
                    self.countdown_active = False
                    self.running = True
                    self.has_started = True

                    self.start_time = now

                    self.countdown_circle.visible = False

                    self.start_pause_button.disabled = False
                    self.start_pause_label.value = "PAUSE"
                    self.start_pause_button.icon = ft.Icons.PAUSE

                    self.run_status.value = (
                        "Rehabilitation running"
                    )
                    self.run_status.color = "#218C55"

            if self.running and self.start_time is not None:

                elapsed = (
                    self.elapsed_before_pause
                    + (now - self.start_time)
                )

                self.theta_ref, _ = reference_trajectory(elapsed)

            if (
                self.client is not None
                and self.client.is_connected
                and self.has_started
                and now - self.last_bt_send >= BT_SEND_INTERVAL
            ):

                await self.send_reference_angle(self.theta_ref)
                self.last_bt_send = now

            actual = self.feedback["angle"]
            last_update = self.feedback["last_update"]

            feedback_valid = (
                actual is not None
                and last_update is not None
                and now - last_update < FEEDBACK_TIMEOUT
            )

            if feedback_valid:

                self.latest_error = self.theta_ref - actual
                tracking = abs(self.latest_error) <= 8.0

            else:

                self.latest_error = None
                tracking = False

            self.reference_value.value = (
                f"{self.theta_ref:.1f}°"
            )

            if (
                abs(
                    self.theta_ref - self.last_reference_drawn
                ) >= IMAGE_UPDATE_THRESHOLD
            ):

                self.reference_canvas.shapes = make_arm_shapes(
                    self.theta_ref
                )

                self.last_reference_drawn = self.theta_ref

            if feedback_valid:

                self.actual_value.value = f"{actual:.1f}°"

                if (
                    abs(actual - self.last_patient_drawn)
                    >= IMAGE_UPDATE_THRESHOLD
                ):

                    self.patient_canvas.shapes = make_arm_shapes(
                        actual
                    )

                    self.last_patient_drawn = actual

                self.error_value.value = (
                    f"{self.latest_error:+.1f}°"
                )

                self.feedback_status.value = (
                    "Encoder feedback: RECEIVING"
                )
                self.feedback_status.color = "#218C55"

                self.tracking_status.value = (
                    "Tracking: GOOD"
                    if tracking
                    else "Tracking: NEEDS IMPROVEMENT"
                )

                self.tracking_status.color = (
                    "#218C55" if tracking else "#B9770E"
                )

            else:

                self.actual_value.value = "Waiting..."
                self.error_value.value = "--"

                self.feedback_status.value = (
                    "Encoder feedback: NOT RECEIVING"
                )
                self.feedback_status.color = "#C0392B"

                self.tracking_status.value = (
                    "Tracking: WAITING FOR FEEDBACK"
                )
                self.tracking_status.color = "#667788"

            self.page.update()

            await asyncio.sleep(1 / UI_FPS)
