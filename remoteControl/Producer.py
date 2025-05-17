import oci  
  
from base64 import b64encode  
  
ociMessageEndpoint = "https://cell-1.streaming.us-chicago-1.oci.oraclecloud.com"  
ociStreamOcid = "ocid1.stream.oc1.us-chicago-1.amaaaaaaoa2s45aam2sr7zfgu34z365rgfkeuch5qule2b2dbbpztvoudq6q"  
ociConfigFilePath = "C:\\Users\\yanzh\\.oci\\config"
ociProfileName = "DEFAULT"  
  
def produce_messages(client, stream_id):
  # Build up a PutMessagesDetails and publish some messages to the stream
  message_list = []
  for i in range(100):
      key = "messageKey" + str(i)
      value = "messageValue " + str(i)
      encoded_key = b64encode(key.encode()).decode()
      encoded_value = b64encode(value.encode()).decode()
      message_list.append(oci.streaming.models.PutMessagesDetailsEntry(key=encoded_key, value=encoded_value))  
  
  print("Publishing {} messages to the stream {} ".format(len(message_list), stream_id))
  messages = oci.streaming.models.PutMessagesDetails(messages=message_list)
  put_message_result = client.put_messages(stream_id, messages)
  
  # The put_message_result can contain some useful metadata for handling failures
  for entry in put_message_result.data.entries:
      if entry.error:
          print("Error ({}) : {}".format(entry.error, entry.error_message))
      else:
          print("Published message to partition {} , offset {}".format(entry.partition, entry.offset))

config = oci.config.from_file(ociConfigFilePath, ociProfileName)
stream_client = oci.streaming.StreamClient(config, service_endpoint=ociMessageEndpoint)

# Publish some messages to the stream
produce_messages(stream_client, ociStreamOcid)