import flet as ft


def build(page, navigate):
    return ft.Column(
        [
            ft.Text(
                "BendAid: Stroke Rehabilitation Exoskeleton",
                size=26,
                weight=ft.FontWeight.BOLD,
                color="#243447",
            ),

            ft.Text(
                "Welcome! Select an option to get started.",
                size=20,
                color="#667788",
            ),

            ft.Container(height=20),

            ft.FilledButton(
                content = ft.Text(
                    "Start Rehabilitation",
                    size=20,
                    weight=ft.FontWeight.BOLD,
                ),
                icon=ft.Icons.PLAY_ARROW,
                width=320,
                height=65,
                style=ft.ButtonStyle(
                    padding=20,
                ),
                on_click=lambda e: navigate("rehabilitation"),
            ),

            ft.OutlinedButton(
                content = ft.Text(
                    "View Progress",
                    size=20,
                    weight=ft.FontWeight.BOLD,
                ),
                icon=ft.Icons.SHOW_CHART,
                width=320,
                height=65,
                style=ft.ButtonStyle(
                    padding=20,
                ),
                on_click=lambda e: navigate("progress"),
            ),

            ft.OutlinedButton(
                content=ft.Text(
                    "Physio Settings",
                    size=20,
                    weight=ft.FontWeight.BOLD,
                ),
                icon=ft.Icons.SETTINGS,
                width=320,
                height=65,
                style=ft.ButtonStyle(
                    padding=20,
                ),
                on_click=lambda e: navigate("settings"),
            ),
        ],
        horizontal_alignment=ft.CrossAxisAlignment.CENTER,
        alignment=ft.MainAxisAlignment.CENTER,
        expand=True,
        spacing=16,
    )