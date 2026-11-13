import flet as ft


def build(page, navigate):
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

            ft.Text("Physiotherapist settings will appear here."),
        ],
        expand=True,
    )