#!/usr/bin/env python3
"""Utility helpers for building status messages and JSON payloads."""

import json


def create_status_payload(status: str, message: str = None, **extra) -> dict:
    """Build a dictionary status payload."""
    payload = {'status': status}
    if message is not None:
        payload['message'] = message
    payload.update(extra)
    return payload


def format_status_json(status: str, message: str = None, **extra) -> str:
    """Build a JSON string status payload."""
    return json.dumps(create_status_payload(status, message, **extra))
