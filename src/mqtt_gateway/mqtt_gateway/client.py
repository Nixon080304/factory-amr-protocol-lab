"""Paho boundary. Its network thread only forwards immutable input to ROS."""

import paho.mqtt.client as mqtt
import threading

FAULT_REQUEST_TOPIC = "factory/faults/injected_request"
FLEET_AVAILABILITY_TOPIC = "factory/fleet/availability"


class MqttClient:
    def __init__(self, host="127.0.0.1", port=1883):
        self.host, self.port = host, port
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id="factory_fleet_gateway"
        )
        self.client.will_set(FLEET_AVAILABILITY_TOPIC, "offline", qos=1, retain=True)
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self._request_subscription = None
        self._fault_subscription = None
        self._fault_requests_enabled = False
        self._fault_generation = None
        self._fault_message = lambda raw: None
        self._fault_connection = lambda ready: None
        self._subscription_lock = threading.RLock()

    def set_handlers(self, message, connection):
        def received(client, userdata, msg):
            if msg.topic == FAULT_REQUEST_TOPIC:
                self._fault_message(bytes(msg.payload))
            elif msg.topic == "factory/missions/request":
                message(bytes(msg.payload))

        self.client.on_message = received

        def connected(client, userdata, flags, reason, properties):
            with self._subscription_lock:
                self._request_subscription = None
                if reason == 0:
                    result, identifier = client.subscribe(
                        "factory/missions/request", qos=1
                    )
                    if result == mqtt.MQTT_ERR_SUCCESS:
                        self._request_subscription = identifier
                    else:
                        connection(False)
                    if self._fault_requests_enabled:
                        self._subscribe_fault_requests()
                else:
                    connection(False)

        def subscribed(client, userdata, identifier, reasons, properties):
            with self._subscription_lock:
                ready = len(reasons) == 1 and not reasons[0].is_failure
                if identifier == self._request_subscription:
                    self._request_subscription = None
                    connection(ready)
                elif identifier == self._fault_subscription:
                    self._fault_subscription = None
                    self._fault_connection((self._fault_generation, ready))

        def disconnected(client, userdata, flags, reason, properties):
            with self._subscription_lock:
                self._request_subscription = self._fault_subscription = None
                connection(False)
                self._fault_connection((self._fault_generation, False))

        self.client.on_connect = connected
        self.client.on_subscribe = subscribed
        self.client.on_disconnect = disconnected

    def set_fault_handlers(self, message, connection):
        self._fault_message, self._fault_connection = message, connection

    def _subscribe_fault_requests(self):
        self._fault_connection((self._fault_generation, False))
        result, identifier = self.client.subscribe(FAULT_REQUEST_TOPIC, qos=1)
        self._fault_subscription = (
            identifier if result == mqtt.MQTT_ERR_SUCCESS else None
        )

    def enable_fault_requests(self, generation):
        with self._subscription_lock:
            if not self._fault_requests_enabled:
                self._fault_requests_enabled = True
                self._fault_generation = generation
                if self.client.is_connected():
                    self._subscribe_fault_requests()

    def disable_fault_requests(self):
        with self._subscription_lock:
            if self._fault_requests_enabled:
                self._fault_requests_enabled = False
                self._fault_subscription = None
                self._fault_connection((self._fault_generation, False))
                self._fault_generation = None
                self.client.unsubscribe(FAULT_REQUEST_TOPIC)

    def start(self):
        self.client.connect_async(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def publish(self, topic, payload, qos=0, retain=False):
        return (
            self.client.publish(topic, payload, qos=qos, retain=retain).rc
            == mqtt.MQTT_ERR_SUCCESS
        )

    def pause(self):
        """Explicit fault control; normal transport never calls this hook."""
        self.client.disconnect()
        self.client.loop_stop()

    def resume(self):
        self.start()

    def close(self):
        self.client.publish(FLEET_AVAILABILITY_TOPIC, "offline", qos=1, retain=True)
        self.client.disconnect()
        self.client.loop_stop()
