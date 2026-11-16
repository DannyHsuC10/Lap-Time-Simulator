"""Translate shared solver messages using the caller's active catalog."""

from contextlib import contextmanager
from contextvars import ContextVar
from collections.abc import Mapping


_catalog: ContextVar[Mapping[str, str] | None] = ContextVar("ui_catalog", default=None)


@contextmanager
def use_translations(catalog: Mapping[str, str]):
    token = _catalog.set(catalog)
    try:
        yield
    finally:
        _catalog.reset(token)


def localize(message: str, **values) -> str:
    catalog = _catalog.get()
    if catalog is not None:
        message = catalog.get(message, message)
    return message.format(**values) if values else message
