#!/usr/bin/env python3
"""Utility helpers for parsing commands and payloads."""

import ast
import json
import re
import unicodedata


def parse_payload_dict(payload) -> dict:
    """Parse JSON string, Python dict repr, or dict into a clean dictionary."""
    if payload is None:
        return {}
    if isinstance(payload, dict):
        if "data" in payload and len(payload) == 1:
            inner = payload["data"]
            if isinstance(inner, dict):
                return parse_payload_dict(inner)
            if isinstance(inner, str):
                text = inner.strip()
                if not text:
                    return {}
                try:
                    parsed = json.loads(text)
                    if isinstance(parsed, dict):
                        return parse_payload_dict(parsed)
                    return {"data": parsed}
                except Exception:
                    try:
                        parsed = ast.literal_eval(text)
                        if isinstance(parsed, dict):
                            return parse_payload_dict(parsed)
                        return {"data": parsed}
                    except Exception:
                        return {"data": text}
            return payload
        return payload
    if isinstance(payload, str):
        text = payload.strip()
        if not text:
            return {}
        try:
            parsed = json.loads(text)
            if isinstance(parsed, dict):
                return parse_payload_dict(parsed)
            return {"data": parsed}
        except Exception:
            try:
                parsed = ast.literal_eval(text)
                if isinstance(parsed, dict):
                    return parse_payload_dict(parsed)
                return {"data": parsed}
            except Exception:
                return {"data": text}
    return {}


def parse_command(payload):
    """Parse a command string from plain string, JSON string, or dict."""
    if payload is None:
        raise ValueError("payload is empty")

    if isinstance(payload, str):
        text = payload.strip()
        if not text:
            raise ValueError("payload is empty")
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            try:
                parsed = ast.literal_eval(text)
            except Exception:
                return text.lower()
        return parse_command(parsed)

    if isinstance(payload, dict):
        if "command" in payload and isinstance(payload["command"], str):
            return payload["command"].strip().lower()
        if "data" in payload:
            return parse_command(payload["data"])

    raise ValueError(f"unsupported payload: {payload!r}")


def extract_map_name(msg_data):
    """Robustly extract map_name from msg_data string, JSON string, or dict."""
    if msg_data is None:
        return ''

    if isinstance(msg_data, dict):
        payload = msg_data.get('data', msg_data)
        if isinstance(payload, dict):
            return str(payload.get('map_name', '')).strip()
        if isinstance(payload, str):
            return payload.strip()

    if isinstance(msg_data, str):
        text = msg_data.strip()
        if not text:
            return ''
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return text

        if isinstance(parsed, dict):
            inner = parsed.get('data', parsed)
            if isinstance(inner, dict):
                return str(inner.get('map_name', '') or parsed.get('map_name', '')).strip()
            if isinstance(inner, str):
                return inner.strip()
        if isinstance(parsed, str):
            return parsed.strip()

    return ''


def sanitize_map_name(map_name: str) -> str:
    """Normalize and sanitize map name to avoid unsafe filesystem characters."""
    if not map_name:
        return ''
    try:
        normalized = unicodedata.normalize('NFKD', map_name)
        normalized = normalized.encode('ascii', 'ignore').decode('ascii')
    except Exception:
        normalized = map_name
    return re.sub(r'[^a-zA-Z0-9_\-\.]', '', normalized)
