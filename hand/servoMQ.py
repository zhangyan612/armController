#!/usr/bin/env python
import pika
import json
import servoControl
import servoState
import time

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()

channel.exchange_declare(exchange='servo', exchange_type='fanout')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue
channel.queue_bind(exchange='servo', queue=queue_name)

print(' [*] Waiting for servo Command. To exit press CTRL+C')

# channel.exchange_declare(exchange='servoState', exchange_type='fanout')
# print(' Created Servo State exchange')

def status_sender():
    servoId = servoState.serial_servo_read_id()
    previousState = {
        'id': 1,
        'position': 0,
        'temp': 0,
        'voltage': 0
    }
    if servoId is not None:
        while True:
            servoData = servoState.read_servo_state(servoId)
            if servoData['position'] != previousState['position']:
                print(servoData)
                previousState = servoData
                # message = json.dumps(command)
                # channel.basic_publish(exchange='servo', routing_key='', body=message)

            time.sleep(1) # only check every seconds, otherwise will receive too much data



def command_receiver(ch, method, properties, body):
    command = json.loads(body)
    print(command)
    action = command['action']
    id = command['id']
    if action == 'open' or action == 'close':
        servoControl.move_command(action, id)
    if action == 'status':
        # publish to servo state channel
        servoData = servoState.read_servo_state(id)
        stateUpdate = {'action': 'feedback', 'id': id, 'state': servoData}
        message = json.dumps(stateUpdate)
        channel.basic_publish(exchange='servo', routing_key='', body=message)


channel.basic_consume(
    queue=queue_name, on_message_callback=command_receiver, auto_ack=True)

channel.start_consuming()

