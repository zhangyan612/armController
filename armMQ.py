#!/usr/bin/env python
import pika
import json
import time

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()

channel.exchange_declare(exchange='arm', exchange_type='fanout')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue
channel.queue_bind(exchange='arm', queue=queue_name)

print(' [*] Waiting for arm command. To exit press CTRL+C')


def command_receiver(ch, method, properties, body):
    command = json.loads(body)
    print(command)
    action = command['action']
    
    if action == 'move':
        url = "ws://127.0.0.1/echo"
        # # Connect to the server
        # async with websockets.connect(url) as ws:
        #     # Send a greeting message
        #     await ws.send("Hello Server!")

        # servoControl.move_command(action, id)
    # if action == 'status':
        # publish to servo state channel
        # servoData = servoState.read_servo_state(id)
        # stateUpdate = {'action': 'feedback', 'id': id, 'state': servoData}
        # message = json.dumps(stateUpdate)
        # channel.basic_publish(exchange='servo', routing_key='', body=message)


channel.basic_consume(
    queue=queue_name, on_message_callback=command_receiver, auto_ack=True)

channel.start_consuming()

