import argparse
from textwrap import dedent

from flask import Flask, Response


class LcmImageArrayMainPage(Flask):
    def __init__(self):
        super().__init__("meldis_lcm_image_viewer")
        self.add_url_rule("/", view_func=self._serve_main_page)

        # Fake channels.
        self.channels = ["foo", "bar", "baz"]
        self._port = 9000

    def _serve_main_page(self):
        html_prefix = dedent(
            """\
            <!doctype html>
            <html>
            <body>
                <h1>Meldis Image Viewer</h1>
            """
        )

        html_suffix = dedent(
            """\
                <script>
                function openNewTab(port) {{
                    window.open("https://127.0.0.1:" + port, "_blank");
                    }}
                </script>
            </body>
            </html>
            """
        )

        html_buttons = []
        for channel in self.channels:
            html_buttons.append(
                f"<button type='button' onclick='openNewTab({self._port})'>"
                f"{channel}</button>\n"
            )
            self._port += 1
        return f"{html_prefix}{''.join(html_buttons)}{html_suffix}"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="URL to host on, default: 127.0.0.1.",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=False,
        default=8000,
        help="Port to host on, default: 8000.",
    )
    args = parser.parse_args()

    main_page = LcmImageArrayMainPage()
    main_page.run(
        host=args.host, port=args.port, debug=False, threaded=False
    )


if __name__ == "__main__":
    main()
