import time
import json
import oci
from base64 import b64decode

stream_ocid = "your_stream_ocid"
config = oci.config.from_file()
client = oci.streaming.StreamClient(config)

cursor_details = oci.streaming.models.CreateGroupCursorDetails(
    group_name="latency-group",
    instance_name="latency-instance",
    type="TRIM_HORIZON",
    commit_on_get=True
)
cursor = client.create_group_cursor(stream_ocid, cursor_details).data.value

def receive():
    for _ in range(10):
        messages = client.get_messages(stream_ocid, cursor, limit=10)
        for msg in messages.data:
            decoded = json.loads(b64decode(msg.value.encode()))
            latency = (time.time() - decoded['timestamp']) * 1000
            print(f"OCI Latency: {latency:.2f} ms")
        cursor = messages.headers["opc-next-cursor"]
        time.sleep(0.2)

receive()
