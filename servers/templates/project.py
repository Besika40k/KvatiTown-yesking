import os

from .base import render_template

_EXTRA_CSS = open(
    os.path.join(os.path.dirname(__file__), "project", "project.css")
).read()
_EXTRA_JS = open(
    os.path.join(os.path.dirname(__file__), "project", "project.js")
).read()
_CONTENT = open(
    os.path.join(os.path.dirname(__file__), "project", "project.html")
).read()


def get_template(title="Project", subtitle="Real Duckiebot"):
    return render_template(
        title=title,
        subtitle=subtitle,
        content_html=_CONTENT,
        extra_css=_EXTRA_CSS,
        extra_js=_EXTRA_JS,
    )


PROJECT_TEMPLATE = get_template()
