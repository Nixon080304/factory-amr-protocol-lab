"""Connection readiness requires the matching successful request SUBACK."""

import paho.mqtt.client as mqtt
from types import SimpleNamespace
from mqtt_gateway.client import MqttClient, FAULT_REQUEST_TOPIC


def test_connection_readiness_waits_for_matching_subscription_ack(monkeypatch):
    transport = MqttClient()
    readiness = []
    transport.set_handlers(lambda raw: None, readiness.append)
    # Replace only the socket write boundary; callback parsing and readiness
    # behavior remain the real adapter's implementation.
    monkeypatch.setattr(
        transport.client,
        "subscribe",
        lambda *args, **kwargs: (mqtt.MQTT_ERR_SUCCESS, 7),
    )
    transport.client.on_connect(transport.client, None, None, 0, None)
    assert readiness == []
    granted = mqtt.ReasonCode(mqtt.PacketTypes.SUBACK, identifier=1)
    transport.client.on_subscribe(transport.client, None, 99, [granted], None)
    assert readiness == []
    transport.client.on_subscribe(transport.client, None, 7, [granted], None)
    assert readiness == [True]
    transport.client.on_disconnect(transport.client, None, None, 0, None)
    transport.client.on_subscribe(transport.client, None, 7, [granted], None)
    assert readiness == [True, False]


def test_rejected_subscription_never_reports_ready(monkeypatch):
    transport = MqttClient()
    readiness = []
    transport.set_handlers(lambda raw: None, readiness.append)
    monkeypatch.setattr(
        transport.client,
        "subscribe",
        lambda *args, **kwargs: (mqtt.MQTT_ERR_SUCCESS, 7),
    )
    transport.client.on_connect(transport.client, None, None, 0, None)
    denied = mqtt.ReasonCode(mqtt.PacketTypes.SUBACK, identifier=128)
    transport.client.on_subscribe(transport.client, None, 7, [denied], None)
    assert readiness == [False]


def test_fault_topic_is_explicit_and_suback_is_generation_scoped(monkeypatch):
    transport = MqttClient()
    ordinary, generated, readiness, subscriptions, unsubscribed = [], [], [], [], []
    transport.set_handlers(ordinary.append, lambda ready: None)
    transport.set_fault_handlers(generated.append, readiness.append)

    def subscribe(topic, **kwargs):
        subscriptions.append(topic)
        return mqtt.MQTT_ERR_SUCCESS, len(subscriptions)

    monkeypatch.setattr(transport.client, "subscribe", subscribe)
    monkeypatch.setattr(transport.client, "unsubscribe", unsubscribed.append)
    monkeypatch.setattr(transport.client, "is_connected", lambda: True)
    transport.client.on_connect(transport.client, None, None, 0, None)
    assert subscriptions == ["factory/missions/request"]
    transport.enable_fault_requests(1)
    assert subscriptions == ["factory/missions/request", FAULT_REQUEST_TOPIC]
    assert readiness == [(1, False)]
    granted = mqtt.ReasonCode(mqtt.PacketTypes.SUBACK, identifier=1)
    transport.disable_fault_requests()
    transport.enable_fault_requests(2)
    transport.client.on_subscribe(transport.client, None, 2, [granted], None)
    assert readiness == [(1, False), (1, False), (2, False)]
    transport.client.on_subscribe(transport.client, None, 3, [granted], None)
    assert readiness[-1] == (2, True)
    for topic in (
        "factory/missions/request",
        FAULT_REQUEST_TOPIC,
        FAULT_REQUEST_TOPIC + "/other",
    ):
        transport.client.on_message(
            transport.client, None, SimpleNamespace(topic=topic, payload=b"{}")
        )
    assert ordinary == generated == [b"{}"]
    transport.disable_fault_requests()
    assert unsubscribed == [FAULT_REQUEST_TOPIC, FAULT_REQUEST_TOPIC]


def test_connect_will_and_shutdown_packets_share_retained_fleet_offline_topic(
    monkeypatch,
):
    transport = MqttClient()
    packets = []

    def wire(command, packet, mid, qos, *args, **kwargs):
        packets.append(bytes(packet))
        return mqtt.MQTT_ERR_SUCCESS

    monkeypatch.setattr(transport.client, "_packet_queue", wire)
    transport.client._send_connect(30)

    def start(packet):
        cursor = 1
        while packet[cursor] & 0x80:
            cursor += 1
        return cursor + 1

    def text_field(packet, cursor):
        size = int.from_bytes(packet[cursor : cursor + 2], "big")
        return packet[cursor + 2 : cursor + 2 + size].decode(), cursor + 2 + size

    connect = packets[0]
    _, cursor = text_field(connect, start(connect))
    flags = connect[cursor + 1]
    identity, cursor = text_field(connect, cursor + 4)
    topic, cursor = text_field(connect, cursor)
    payload, _ = text_field(connect, cursor)
    assert topic == "factory/fleet/availability" and payload == "offline"
    assert "amr_01" not in identity
    assert flags & 0x20 and (flags >> 3) & 3 == 1
    # Replace only the unavailable native socket. Paho still encodes PUBLISH.
    transport.client._sock = SimpleNamespace(close=lambda: None)
    transport.close()
    publish = next(packet for packet in packets if packet[0] >> 4 == 3)
    topic, cursor = text_field(publish, start(publish))
    assert topic == "factory/fleet/availability"
    assert (
        publish[cursor + 2 :] == b"offline"
    )  # QoS 1 packet identifier precedes payload.
    assert publish[0] & 1 and (publish[0] >> 1) & 3 == 1
