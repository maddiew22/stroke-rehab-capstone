
import flet as ft
import config


def build(page, navigate):

    cycle_time_input = ft.TextField(
        label="Cycle Time (seconds)",
        value=str(config.CYCLE_TIME),
        width=280,
        keyboard_type=ft.KeyboardType.NUMBER,
        text_size=18,
    )

    session_duration_input = ft.TextField(
        label="Session Duration (minutes)",
        value=str(config.SESSION_DURATION / 60),
        width=280,
        keyboard_type=ft.KeyboardType.NUMBER,
        text_size=18,
    )

    status_text = ft.Text(
        "",
        color="#218C55",
        size=14,
    )

    def save_settings(e):
        try:
            cycle_time = float(cycle_time_input.value)
            session_duration = float(session_duration_input.value)

            if cycle_time <= 0 or session_duration <= 0:
                raise ValueError

            config.CYCLE_TIME = cycle_time
            config.SESSION_DURATION = session_duration * 60

            status_text.value = "Settings saved successfully!"
            status_text.color = "#218C55"

        except (ValueError, TypeError):
            status_text.value = "Please enter valid positive numbers."
            status_text.color = "#C0392B"

        page.update()

    return ft.Column(
        [
            ft.Row(
                [
                    ft.IconButton(
                        ft.Icons.ARROW_BACK,
                        on_click=lambda e: navigate("home"),
                    ),
                    ft.Text(
                        "Physio Settings",
                        size=26,
                        weight=ft.FontWeight.BOLD,
                    ),
                ]
            ),

            ft.Text(
                "Rehabilitation Parameters",
                size=20,
                weight=ft.FontWeight.BOLD,
            ),

            cycle_time_input,
            session_duration_input,

            ft.Text(
                "The cycle time determines how long one complete flexion-extension cycle takes.",
                size=14,
            ),

            ft.FilledButton(
                "Save Settings",
                icon=ft.Icons.SAVE,
                width=280,
                height=55,
                on_click=save_settings,
            ),

            status_text,
        ],
        expand=True,
        spacing=20,
    )
