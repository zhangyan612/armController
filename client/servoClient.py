#!/usr/bin/env python
import pika
import sys
import json

remoteHost = 'localhost'
credential = pika.credentials.PlainCredentials('yan', 'yan', erase_on_connect=False)

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host=remoteHost, credentials=credential))
channel = connection.channel()

channel.exchange_declare(exchange='servo', exchange_type='fanout')

command = {'action': 'status', 'id': 1}
command = {'action': 'open', 'id': 1}

message = json.dumps(command)
channel.basic_publish(exchange='servo', routing_key='', body=message)
print(f" [x] Sent {message}")
connection.close()
