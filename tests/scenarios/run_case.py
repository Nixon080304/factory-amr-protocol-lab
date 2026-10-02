#!/usr/bin/env python3
"""Reuse reviewed real system assertions; export evidence only after cleanup."""
import json
import os
from pathlib import Path
import signal
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/system'))
sys.path.insert(0, str(ROOT / 'src/protocol_observer'))


def interrupted(signum, frame):
    raise InterruptedError(f'owned scenario interrupted by signal {signum}')


def reset_protocol(rig):
    from std_srvs.srv import Trigger
    from test_protocol_faults import wait
    future = rig['reset'].call_async(Trigger.Request())
    wait(future.done)
    assert future.result().success, future.result().message
    assert rig['gateway'].faults.query() == rig['modbus'].faults.query() == ()


def run(name, output):
    from test_protocol_faults import rig, test_modbus_fault_outcome_cleanup_and_trace, \
        test_mqtt_duplicate_and_conflict_execute_once_and_trace, test_disconnect_local_completion_and_ordered_replay
    if name == 'success':
        from test_successful_mission import test_successful_factory_mission
        actual = test_successful_factory_mission()
        actual['source'] = 'real_gazebo'
    elif name in ('wrong_marker', 'nav_retry', 'nav_failure'):
        from test_autonomy_faults import test_autonomy_fault_outcome_recovery_and_trace
        internal, error = {'wrong_marker': ('wrong_marker', 'STATION_NOT_CONFIRMED'),
            'nav_retry': ('nav_reject_once', None), 'nav_failure': ('nav_reject_twice', 'NAVIGATION_FAILED')}[name]
        actual = test_autonomy_fault_outcome_recovery_and_trace(internal, error)
        actual['source'] = 'real_gazebo'
    elif name == 'qos_mismatch':
        from test_qos_behavior import test_real_dds_mismatch_records_policies_and_recovers
        actual = test_real_dds_mismatch_records_policies_and_recovers()
        actual['source'] = 'isolated_dds'
    else:
        # Direct fixture entry retains production peers, bounded action driver,
        # and the fixture's owned executor/container/port cleanup assertions.
        fixture = rig.__wrapped__()
        peer = next(fixture)
        try:
            reset_protocol(peer)
            if name.startswith('modbus_') or name == 'plc_fault':
                error = {'modbus_delay': None, 'modbus_timeout': 'PLC_TIMEOUT',
                    'modbus_stale_completion': 'STALE_PLC_STATE', 'plc_fault': 'PLC_FAULT'}[name]
                test_modbus_fault_outcome_cleanup_and_trace(peer, name, error)
            elif name in ('mqtt_duplicate', 'mqtt_conflict'):
                test_mqtt_duplicate_and_conflict_execute_once_and_trace(peer, name)
            elif name == 'mqtt_disconnect':
                test_disconnect_local_completion_and_ordered_replay(peer)
            else:
                raise ValueError(f'unknown scenario: {name}')
            reset_protocol(peer)
            # Conflict rejection is not the authoritative action's result.
            final = [status for status in peer['statuses'] if status['state'] in ('COMPLETED', 'FAILED')
                     and status['error_code'] != 'MISSION_ID_CONFLICT'][-1]
            actual = dict(final_state=final['state'], error_code=final['error_code'], mission_id='M-fault',
                          action_executions=len(peer['executed']), source='real_protocol_navigation_driver',
                          events=peer['events'], statuses=peer['statuses'],
                          payload_state=peer['payload'].state,
                          plc_counters=[peer['plc'].stations[unit].registers[2] for unit in (1, 2)])
        finally:
            try:
                next(fixture)
            except StopIteration:
                pass
    from datetime import datetime, timezone
    from protocol_observer.models import ProtocolEventRecord
    from protocol_observer.trace_writer import TraceWriter
    # Preserve the actual observer trace for physical missions; use the same
    # serializer for probes that have no observer process.
    events = actual.pop('events')
    trace = output / 'protocol_events.jsonl'
    if not trace.exists():
        writer = TraceWriter(trace)
        for event in events:
            writer.append(ProtocolEventRecord(datetime.fromtimestamp(event.stamp.sec + event.stamp.nanosec / 1e9,
                tz=timezone.utc), event.mission_id, event.protocol, event.direction, event.event, event.outcome,
                event.latency_ms, event.detail))
    actual['domain_id'] = int(os.environ['FACTORY_SCENARIO_DOMAIN'])
    actual['cleanup_verified'] = True
    (output / 'actual.json').write_text(json.dumps(actual, indent=2) + '\n')


if __name__ == '__main__':
    name, directory = sys.argv[1:]
    output = Path(directory)
    os.environ['FACTORY_SCENARIO_OUTPUT'] = str(output)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('DISPLAY', ':0')
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    # The public supervisor retains the assigned domain lease through cleanup.
    os.environ['ROS_DOMAIN_ID'] = os.environ['FACTORY_SCENARIO_DOMAIN']
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    run(name, output)
