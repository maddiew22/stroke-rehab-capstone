
import flet as ft

from pages import home, progress, settings
from pages.rehabilitation import RehabilitationPage


def main(page: ft.Page):
    page.title = "Elbow Rehabilitation System"
    page.window.width = 1100
    page.window.height = 800
    page.theme_mode = ft.ThemeMode.LIGHT
    page.bgcolor = "#F3F5F8"
    page.padding = 24

    rehabilitation = None

    def navigate(page_name):

        async def change_page():

            nonlocal rehabilitation

            if rehabilitation is not None:
                await rehabilitation.dispose()
                rehabilitation = None

            page.clean()

            if page_name == "home":
                page.add(home.build(page, navigate))

            elif page_name == "progress":
                page.add(progress.build(page, navigate))

            elif page_name == "settings":
                page.add(settings.build(page, navigate))

            elif page_name == "rehabilitation":

                rehabilitation = RehabilitationPage(
                    page,
                    navigate,
                )

                page.add(rehabilitation.build())

                rehabilitation.start()

            page.update()

        page.run_task(change_page)

    navigate("home")


if __name__ == "__main__":
    ft.run(main)
