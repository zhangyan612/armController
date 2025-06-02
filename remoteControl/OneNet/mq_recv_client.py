# coding: utf-8
import threading

import paho.mqtt.client as mqtt
import time
import mq.onenet_mq_pb2 as proto
from sastoken import sas_token
from _ssl import CERT_NONE, CERT_OPTIONAL, CERT_REQUIRED


log = []

def ts_print(*args):
    t = time.strftime("[%Y-%m-%d %H:%M:%S.")
    ms = str(time.time()).split('.')[1][:3]
    t += ms + ']:'
    print(t, *args)

    temp = t
    for x in args:
        temp += ' ' + str(x)
    temp += '\n'
    if len(log) < 100:
        log.append(temp)


class MQClient:
    def __init__(self, host, port, namespace, access_key, topic_name, sub_name):

        self.status = False
        self.namespace = namespace
        self.access_key = access_key
        self.topic_name = topic_name
        self.sub_name = sub_name
        self.host = host
        self.port = port
        self.topic = '$sys/pb/consume/%s/%s/%s' % (self.namespace, self.topic_name, self.sub_name)
        self.count = 0
        self.msg_id = 0

        client_id = self.topic_name  # clientid：用户自定义合法的UTF-8字符串，可为空
        username = self.namespace  # 填写涉及的ns，目前不支持除数字、字母、-、下划线以外的特殊字符
        try:
            password = sas_token('sha256', self.namespace, access_key)
        except Exception as e:
            ts_print('参数错误，生产token失败：' + str(e))
            return

        ts_print('username: ' + username)
        ts_print('password: ' + password)

        self.client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect

        self.client.tls_set(ca_certs='certificate.pem', cert_reqs=CERT_REQUIRED)
        self.client.tls_insecure_set(True)

        self.client.username_pw_set(username=username, password=password)
        self.status = True

    def on_connect(self, client, userdata, flags, rc):
        ts_print("<<<<CONNACK")
        ts_print("connected with result code " + mqtt.connack_string(rc), rc)

        if rc == 0:
            ts_print('订阅topic: %s' % self.topic)
            client.subscribe(self.topic, qos=1)

    def on_message(self, client, userdata, msg):
        self.count += 1
        proto_msg = proto.Msg()
        proto_msg.ParseFromString(msg.payload)
        ts_print(self.count, proto_msg.msgid, proto_msg.data.decode(), proto_msg.timestamp, time.time())

    def on_subscribe(self, client, obj, mid, granted_qos):
        ts_print("Subscribed: mid: " + str(mid) + "  qos:", granted_qos[0])

    def on_disconnect(self, client, userdata, rc):
        ts_print('DISCONNECTED with result code: %d' % rc, mqtt.connack_string(rc))
        #重新设置一下password,防止重连时token过期
        password = sas_token('sha256', self.namespace, access_key)
        self.client.username_pw_set(username=self.namespace,password=password)

    def run(self):
        if self.status:
            try:
                self.client.connect(host=self.host, port=self.port, keepalive=120)
            except Exception as e:
                ts_print('服务器连接失败 %s' % str(e))
                return
            self.client.loop_start()


if __name__ == '__main__':

    host = '183.230.40.96'
    port = 8883

    access_key = 'QGq1rrJm8+ySSkhaR3OgmvZZRkcLRXLmRnmhq8AYNC4=' #acckey
    namespace = 'yan_robot'   #Mqid
    topic_name1 = 'robot' #topic
    sub_name1 = 'consumer' #sub

    mq = MQClient(host, port, namespace, access_key, topic_name1, sub_name1)
    mq.run()
    while True:
        time.sleep(1)




