"""Render the model reference Markdown and LaTeX into inert HTML templates."""

from pathlib import Path
from xml.etree.ElementTree import fromstring


REFERENCE_MARKER = "<!-- MODEL_REFERENCES -->"


def render_references(source: str, directory: Path, models: dict) -> str:
    if source.count(REFERENCE_MARKER) != 1:
        raise ValueError("Expected one model reference marker in the web index.")
    try:
        from latex2mathml.converter import convert
        from markdown_it import MarkdownIt
        from mdit_py_plugins.dollarmath import dollarmath_plugin
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "Install web build dependencies with python -m pip install -r LapSimLab/web/requirements.txt."
        ) from error

    def render_math(latex, options):
        mathml = convert(latex.strip(), display="block" if options["display_mode"] else "inline")
        fromstring(mathml)
        return mathml

    markdown = MarkdownIt("commonmark", {"html": False}).use(
        dollarmath_plugin, renderer=render_math, allow_labels=False,
    )
    templates = []
    for key, choices in models.items():
        variants = (*choices, "disabled") if key == "aero_model" else choices
        for value in variants:
            name = f"{key}-{value}"
            path = directory / f"{name}.md"
            try:
                body = markdown.render(path.read_text(encoding="utf-8"))
            except Exception as error:
                raise ValueError(f"Could not render model reference {path.name}: {error}") from error
            templates.append(f'<template id="reference-{name}">\n{body}</template>')

    return source.replace(REFERENCE_MARKER, "\n".join(templates))
