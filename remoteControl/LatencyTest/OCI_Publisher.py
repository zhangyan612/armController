import time
import json
import oci
from base64 import b64encode

stream_ocid = "your_stream_ocid"
config = oci.config.from_file()
stream_client = oci.streaming.StreamClient(config)

def send():
    for _ in range(10):
        payload = json.dumps({'timestamp': time.time()})
        message = oci.streaming.models.PutMessagesDetailsEntry(
            key=b64encode(b"latency").decode(),
            value=b64encode(payload.encode()).decode()
        )
        messages = oci.streaming.models.PutMessagesDetails(messages=[message])
        stream_client.put_messages(stream_ocid, messages)
        time.sleep(0.2)

send()
