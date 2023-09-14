# Robot Arm Control

1. run the startServer.bat file to start the server
2. run py control file

rabbitmq service install on windows server for pub sub communication

https://www.rabbitmq.com/install-windows.html#configure



Install the "URL Rewrite" module if it's not already installed.
https://www.iis.net/downloads/microsoft/url-rewrite
https://iis-umbraco.azurewebsites.net/downloads/microsoft/application-request-routing

Create a new "Inbound Rule" to rewrite WebSocket requests. Here's a sample rule:

Match URL: Requested URL: Matches the Pattern
Using: Wildcards
Pattern: ws://localhost:7890/*
Action: Rewrite
Rewrite URL: ws://127.0.0.1:7890/{R:0}
Append query string: Checked
This rule will redirect WebSocket requests from yourdomain.com to your Python WebSocket server running on 127.0.0.1:8765.

