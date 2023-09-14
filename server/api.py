# from flask import Flask

# app = Flask(__name__)

# @app.route('/')
# def hello():
#     return 'Hello, World from flask!'

# app.run()

from flask import Flask, render_template
from flask_sock import Sock

app = Flask(__name__)
sock = Sock(app)


@app.route('/')
def index():
    return 'This is the flask api for robot control'

@app.route('/moveArm')
def moveArm(data):
    data= {}
    return 'move arm '

@sock.route('/echo')
def echo(sock):
    while True:
        data = sock.receive()
        sock.send(data)

        
app.run()