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
                size=16,
                color="#667788",
            ),

            ft.Container(height=20),

            ft.FilledButton(
                "Start Rehabilitation",
                icon=ft.Icons.PLAY_ARROW,
                width=280,
                on_click=lambda e: navigate("rehabilitation"),
            ),

            ft.OutlinedButton(
                "View Progress",
                icon=ft.Icons.SHOW_CHART,
                width=280,
                on_click=lambda e: navigate("progress"),
            ),

            ft.OutlinedButton(
                "Physio Settings",
                icon=ft.Icons.SETTINGS,
                width=280,
                on_click=lambda e: navigate("settings"),
            ),
        ],
        horizontal_alignment=ft.CrossAxisAlignment.CENTER,
        alignment=ft.MainAxisAlignment.CENTER,
        expand=True,
        spacing=16,
    )