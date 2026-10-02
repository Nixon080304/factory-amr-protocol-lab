"""Paho boundary. Its network thread only forwards immutable input to ROS."""
import paho.mqtt.client as mqtt


class MqttClient:
    def __init__(self, host='127.0.0.1', port=1883):
        self.host, self.port = host, port
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='factory_amr_01')
        self.client.will_set('factory/robots/amr_01/availability', 'offline', qos=1, retain=True)
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)

    def set_handlers(self, message, connection):
        self.client.on_message = lambda client, userdata, msg: message(bytes(msg.payload))
        def connected(client, userdata, flags, reason, properties):
            if reason == 0:
                client.subscribe('factory/missions/request', qos=1)
                connection(True)
            else:
                connection(False)
        self.client.on_connect = connected
        self.client.on_disconnect = lambda client, userdata, flags, reason, properties: connection(False)

    def start(self):
        self.client.connect_async(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def publish(self, topic, payload, qos=0, retain=False):
        return self.client.publish(topic, payload, qos=qos, retain=retain).rc == mqtt.MQTT_ERR_SUCCESS

    def pause(self):
        """Explicit fault control; normal transport never calls this hook."""
        self.client.disconnect()
        self.client.loop_stop()

    def resume(self):
        self.start()

    def close(self):
        self.client.publish('factory/robots/amr_01/availability', 'offline', qos=1, retain=True)
        self.client.disconnect()
        self.client.loop_stop()
