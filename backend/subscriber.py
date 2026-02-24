"""IoT Core MQTT5 subscriber for digital twin telemetry.

Uses MQTT over WebSocket with IAM/SigV4 authentication.
Designed to run on an EC2 instance with an appropriate instance profile.
"""

import json
import logging
import threading
import time
from typing import Callable, Optional

import boto3
from awscrt import auth, mqtt5
from awsiot import mqtt5_client_builder

from config_loader import load_config
from joint_converter import convert_payload

logger = logging.getLogger(__name__)


class TelemetryBuffer:
    """Thread-safe single-slot buffer holding the latest telemetry."""

    def __init__(self):
        self._lock = threading.Lock()
        self._raw: Optional[dict] = None
        self._converted: Optional[dict] = None
        self._timestamp: int = 0
        self._message_count: int = 0

    def update(self, raw_payload: dict) -> None:
        """Update buffer with new telemetry payload.

        Args:
            raw_payload: Raw IoT telemetry message.
        """
        converted = convert_payload(raw_payload)
        with self._lock:
            self._raw = raw_payload
            self._converted = converted
            self._timestamp = raw_payload.get('timestamp', 0)
            self._message_count += 1

    def get_latest(self) -> Optional[dict]:
        """Returns the latest converted payload, or None."""
        with self._lock:
            return self._converted

    def get_raw(self) -> Optional[dict]:
        """Returns the latest raw payload, or None."""
        with self._lock:
            return self._raw

    @property
    def last_timestamp(self) -> int:
        """Returns the timestamp of the last received message."""
        with self._lock:
            return self._timestamp

    @property
    def message_count(self) -> int:
        """Returns the total number of messages received."""
        with self._lock:
            return self._message_count


class IoTSubscriber:
    """Subscribes to IoT Core MQTT topic via WebSocket/SigV4."""

    def __init__(self, buffer: TelemetryBuffer, on_message: Optional[Callable] = None):
        """Initialize the IoT subscriber.

        Args:
            buffer: TelemetryBuffer instance to store incoming messages.
            on_message: Optional callback function to invoke on each message.
        """
        self.config = load_config()
        self.buffer = buffer
        self.on_message = on_message
        self._client: Optional[mqtt5.Client] = None

        iot_cfg = self.config['iot']
        self.topic = f"{iot_cfg['topicPrefix']}/{iot_cfg['deviceId']}/telemetry"
        self.region = self.config['aws']['region']
        self.client_id = f"isaac-sim-dt-{int(time.time())}"

    def _get_endpoint(self) -> str:
        """Discover the IoT Core data endpoint for this region.

        Returns:
            str: IoT Core endpoint address.
        """
        client = boto3.client('iot', region_name=self.region)
        resp = client.describe_endpoint(endpointType='iot:Data-ATS')
        return resp['endpointAddress']

    def _on_message(self, data):
        """Internal MQTT message callback.

        Args:
            data: PublishReceivedData containing the publish packet.
        """
        try:
            payload = json.loads(data.publish_packet.payload.decode('utf-8'))
            self.buffer.update(payload)
            if self.on_message:
                self.on_message(payload)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            logger.warning(f"Failed to parse message: {e}")

    def _on_connection_success(self, data: mqtt5.LifecycleConnectSuccessData):
        """Callback invoked on successful connection to IoT Core.

        Args:
            data: Connection success event data.
        """
        logger.info("Connected to IoT Core")
        self._client.subscribe(
            subscribe_packet=mqtt5.SubscribePacket(
                subscriptions=[
                    mqtt5.Subscription(
                        topic_filter=self.topic,
                        qos=mqtt5.QoS.AT_MOST_ONCE,
                    )
                ]
            )
        )
        logger.info(f"Subscribed to {self.topic}")

    def _on_disconnection(self, data: mqtt5.LifecycleDisconnectData):
        """Callback invoked on disconnection from IoT Core.

        Args:
            data: Disconnection event data.
        """
        logger.warning(f"Disconnected: {data.disconnect_packet}")

    def _on_connection_failure(self, data: mqtt5.LifecycleConnectFailureData):
        """Callback invoked on connection failure.

        Args:
            data: Connection failure event data.
        """
        logger.warning(f"Connection failed, will retry: {data.exception}")

    def start(self):
        """Start the MQTT client and connect to IoT Core."""
        endpoint = self._get_endpoint()
        logger.info(f"IoT endpoint: {endpoint}")

        credentials_provider = auth.AwsCredentialsProvider.new_default_chain()

        self._client = mqtt5_client_builder.websockets_with_default_aws_signing(
            endpoint=endpoint,
            region=self.region,
            credentials_provider=credentials_provider,
            client_id=self.client_id,
            on_publish_received=self._on_message,
            on_lifecycle_connection_success=self._on_connection_success,
            on_lifecycle_disconnection=self._on_disconnection,
            on_lifecycle_connection_failure=self._on_connection_failure,
        )
        self._client.start()
        logger.info(f"MQTT client started (client_id={self.client_id})")

    def stop(self):
        """Stop the MQTT client and disconnect from IoT Core."""
        if self._client:
            self._client.stop()
            logger.info("MQTT client stopped")


def main():
    """Standalone mode: subscribe and print messages."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    buffer = TelemetryBuffer()
    subscriber = IoTSubscriber(buffer)
    subscriber.start()

    try:
        last_count = 0
        last_report = time.time()
        while True:
            time.sleep(1.0)
            count = buffer.message_count
            now = time.time()
            if now - last_report >= 30:
                rate = (count - last_count) / (now - last_report)
                logger.info(f"Message rate: {rate:.1f} msg/s (total: {count})")
                last_count = count
                last_report = now

            latest = buffer.get_latest()
            if latest:
                logger.debug(f"Latest: {latest}")
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        subscriber.stop()


if __name__ == '__main__':
    main()
