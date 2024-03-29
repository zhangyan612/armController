#!/usr/bin/env python
from flask import Flask, render_template, request
from flask_cors import CORS
import sys
import json
import config
import serial


app = Flask(__name__)
CORS(app)

# Check if the serial port is open
if ser.isOpen():
    print("Head Serial port is open")
else:
    print("Failed to open head serial port")


@app.route('/')
def index():
    return {'api works'}


@app.route('/action', methods=['POST'])
def action():
    data = request.json
    print(data)
    exchange = ''
    message = ''

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


app.run(host="0.0.0.0")
