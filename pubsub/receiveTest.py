#!/usr/bin/env python
import pika

credentials = pika.credentials.PlainCredentials('guest', 'guest', erase_on_connect=False)
connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='192.168.0.247', credentials=credentials))
channel = connection.channel()

channel.exchange_declare(exchange='logs', exchange_type='fanout')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue

channel.queue_bind(exchange='logs', queue=queue_name)

print(' [*] Waiting for logs. To exit press CTRL+C')

def callback(ch, method, properties, body):
    print(f" [x] {body}")

channel.basic_consume(
    queue=queue_name, on_message_callback=callback, auto_ack=True)

channel.start_consuming()
