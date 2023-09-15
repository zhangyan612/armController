#!/usr/bin/env python
import pika
import json
import time
import websockets
import asyncio

remoteHost = '192.168.0.247'  #'localhost'
credential = pika.credentials.PlainCredentials('yan', 'yan', erase_on_connect=False)

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host=remoteHost, credentials=credential))
channel = connection.channel()

channel.exchange_declare(exchange='arm', exchange_type='fanout')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue
channel.queue_bind(exchange='arm', queue=queue_name)

print(' [*] Waiting for arm command. To exit press CTRL+C')

async def moveArm(command):
    url = "ws://localhost:8080/api/ws"
    # Connect to the server
    moveCommand = {'action': command['action']}
    if command.get('payload'):
        moveCommand['payload'] = command['payload']
    async with websockets.connect(url) as ws:
        # Send the arm command message
        await ws.send(json.dumps(moveCommand))

# receive command from MQ and send to arm server socket
def command_receiver(ch, method, properties, body):
    msg = json.loads(body)
    print(msg)
    if msg.get('command'):
        command = msg['command']
        if command == 'arm':
            asyncio.get_event_loop().run_until_complete(moveArm(msg))

channel.basic_consume(
    queue=queue_name, on_message_callback=command_receiver, auto_ack=True)

channel.start_consuming()

