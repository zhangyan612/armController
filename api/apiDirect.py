#!/usr/bin/env python
from flask import Flask, render_template, request
from flask_cors import CORS
import sys
import json
import config


app = Flask(__name__)
CORS(app)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/testui')
def testui():
    return render_template('test.html')

@app.route('/action', methods=['POST'])
def action():
    data = request.json  # Assuming the request data is in JSON format
    print(data)
    exchange = ''
    message = ''

    if data['action'] == 'apply':
        exchange='arm'
        command = {
            "command": 'arm',
            "action": "PositionControlWithAccel",
            "payload": data['values']
        }
        message = json.dumps(command)

    if data['action'] == 'reset':
        exchange='arm'
        command = {
            "command": 'arm',
            "action": "SetTask",
            "payload": {
            "type": "BackToZeroTask"
            }
        }
        message = json.dumps(command)

    if data['action'] == 'enable':
        exchange='arm'
        command = {'command': 'arm', 'action': 'Enable'}
        message = json.dumps(command)

    if data['action'] == 'disable':
        exchange='arm'
        command = {'command': 'arm', 'action': 'Disable'}
        message = json.dumps(command)

    if data['action'] == 'open':
        exchange='servo'
        command = {'action': 'open', 'id': 1}
        message = json.dumps(command)

    if data['action'] == 'close':
        exchange='servo'
        command = {'action': 'close', 'id': 1}
        message = json.dumps(command)

    try:
        if exchange and message:
            print('send msg')
            # channel.basic_publish(exchange=exchange, routing_key='', body=message)
        else:
            print("Exchange and/or message is empty, cannot publish.")
    except Exception as e:
        # Handle the exception here
        print(f"An error occurred: {str(e)}")

    return data

@app.route('/test', methods=['POST'])
def test():
    data = request.json  # You can access POST data as JSON if the request has a JSON content-type
    print(data)
    # Process the data as needed
    return f"Received data: {data}"


app.run(host="0.0.0.0")
