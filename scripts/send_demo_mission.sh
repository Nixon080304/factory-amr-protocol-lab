#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
export PYTHONNOUSERSITE=1
project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
"$project_root/.venv/bin/python" - <<'PY'
import json
import os
import uuid
import paho.mqtt.client as mqtt

port = os.environ.get('FACTORY_MQTT_PORT', '1883')
if not port.isascii() or not port.isdigit() or not 1 <= int(port) <= 65535:
    raise SystemExit('FACTORY_MQTT_PORT must be an integer from 1 to 65535')
mission = {'mission_id': 'M-001', 'robot_id': 'amr_01', 'pickup': 'assembly',
           'dropoff': 'inspection', 'part': 'motor'}
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='demo-publisher-' + uuid.uuid4().hex)
client.connect('127.0.0.1', int(port))
client.loop_start()
try:
    result = client.publish('factory/missions/request', json.dumps(mission), qos=1, retain=False)
    result.wait_for_publish(timeout=10)
    if not result.is_published():
        raise SystemExit('Mission publication was not acknowledged within 10 seconds')
    print(json.dumps(mission))
finally:
    client.disconnect()
    client.loop_stop()
PY
