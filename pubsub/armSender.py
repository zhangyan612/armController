#!/usr/bin/env python
import pika
import sys
import json

remoteHost = 'localhost'
credential = pika.credentials.PlainCredentials('yan', 'yan', erase_on_connect=False)

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host=remoteHost, credentials=credential))
channel = connection.channel()

channel.exchange_declare(exchange='arm', exchange_type='fanout')

command = {'action': 'move', 'joint1': 0, 'joint1': 0, 'joint3': 0, 'joint4': 0, 'joint5': 0, 'joint6': 0}
message = json.dumps(command)
channel.basic_publish(exchange='arm', routing_key='', body=message)
print(f" [x] Sent {message}")
connection.close()
