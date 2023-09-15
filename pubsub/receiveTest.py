#!/usr/bin/env python
import pika
import json

credential = pika.credentials.PlainCredentials('yan', 'yan', erase_on_connect=False)
connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost', credentials=credential))
channel = connection.channel()

channel.exchange_declare(exchange='servo', exchange_type='fanout')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue

channel.queue_bind(exchange='servo', queue=queue_name)

print(' [*] Waiting for messages. To exit press CTRL+C')

def callback(ch, method, properties, body):
    response = json.loads(body)
    print(response)
    print(response['action'])

channel.basic_consume(
    queue=queue_name, on_message_callback=callback, auto_ack=True)

channel.start_consuming()
