"""
Ableware Command Hub — entry point.

Run from the project root:
    python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8000
"""

from server.app import app  # noqa: F401 — re-exported for uvicorn
