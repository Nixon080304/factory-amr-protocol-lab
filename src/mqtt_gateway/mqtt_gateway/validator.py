# SPDX-License-Identifier: Apache-2.0
"""Validate bounded MQTT bytes before they enter the mission system."""

import json
from importlib.resources import files

from jsonschema import Draft202012Validator

from .models import MissionPayload


class MissionValidationError(ValueError):
    error_code = "INVALID_MISSION"

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


class MissionValidator:
    def __init__(self):
        schema = json.loads(
            files("mqtt_gateway").joinpath("schemas/mission.schema.json").read_text(encoding="utf-8")
        )
        self._validator = Draft202012Validator(schema)

    def validate(self, raw_payload: bytes) -> MissionPayload:
        if len(raw_payload) > 4096:
            raise MissionValidationError("Payload exceeds 4096 bytes")
        try:
            payload = json.loads(raw_payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError, RecursionError) as error:
            raise MissionValidationError("Payload must be valid UTF-8 JSON") from error
        error = next(self._validator.iter_errors(payload), None)
        if error is not None:
            field = ".".join(str(part) for part in error.absolute_path) or "mission"
            raise MissionValidationError(f"{field} violates {error.validator}")
        return MissionPayload(**payload)
