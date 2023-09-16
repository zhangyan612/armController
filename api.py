#!/usr/bin/env python
from flask import Flask, request
from flask_cors import CORS
import pika
import sys
import json
import config

remoteHost = config.MQHost
credential = pika.credentials.PlainCredentials(config.MQUsername, config.MQPassword, erase_on_connect=False)

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host=remoteHost, credentials=credential))
channel = connection.channel()

channel.exchange_declare(exchange='arm', exchange_type='fanout')

app = Flask(__name__)
CORS(app)


@app.route('/')
def index():
    return 'This is the flask api for robot control'

@app.route('/action', methods=['POST'])
def action():
    data = request.json  # Assuming the request data is in JSON format
    print(data)

    if data['action'] == 'apply':
        command = {
            "command": 'arm',
            "action": "PositionControlWithAccel",
            "payload": data['values']
        }
        message = json.dumps(command)
        # channel.basic_publish(exchange='arm', routing_key='', body=message)

    if data['action'] == 'enable':
        command = {'command': 'arm', 'action': 'Enable'}
        message = json.dumps(command)
        # channel.basic_publish(exchange='arm', routing_key='', body=message)

    if data['action'] == 'disable':
        command = {'command': 'arm', 'action': 'Disable'}
        message = json.dumps(command)
        # channel.basic_publish(exchange='arm', routing_key='', body=message)

    if data['action'] == 'open':
        command = {'action': 'open', 'id': 1}
        message = json.dumps(command)
        # channel.basic_publish(exchange='servo', routing_key='', body=message)

    if data['action'] == 'close':
        command = {'action': 'close', 'id': 1}
        message = json.dumps(command)
        channel.basic_publish(exchange='servo', routing_key='', body=message)

    return data

        
app.run()
