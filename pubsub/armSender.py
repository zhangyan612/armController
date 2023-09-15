#!/usr/bin/env python
import pika
import sys
import json

remoteHost = '192.168.0.247'  #'localhost'
credential = pika.credentials.PlainCredentials('yan', 'yan', erase_on_connect=False)

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host=remoteHost, credentials=credential))
channel = connection.channel()

channel.exchange_declare(exchange='arm', exchange_type='fanout')


def arm_actions():
    wscript = {
        'action': 'SetTask',
        'payload': {
            'type': 'WScriptTask',
            'args': """
            // WScript goes here
            MOVEL -0.000,0.157,0.204,180,0,-90,3
            """
        }
    }
    start = {
    "action": "Enable",
    }
    disable = {
    "action": "Disable",
    }

# command = {'command': 'arm', 'action': 'Enable'}
# command = {'command': 'arm', 'action': 'Disable'}

command = {
    "command": 'arm',
    "action": "SetTask",
    "payload": {
    "type": "BackToZeroTask"
    }
}

message = json.dumps(command)
channel.basic_publish(exchange='arm', routing_key='', body=message)
print(f" [x] Sent {message}")
connection.close()
