#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import websockets
import asyncio
import threading
from flask import Flask, render_template

from diagnostic_msgs.msg import DiagnosticArray

app = Flask(__name__)
class DiagnosticsSubscriber(Node):

    def __init__(self, websocket_url, server_port):
        super().__init__('diagnostics_subscriber')
        self.websocket_url = websocket_url
        self.server_port = server_port
        self.subscription = self.create_subscription(
            DiagnosticArray, 'diagnostics_agg', self.diagnostics_callback, 10)
        self.subscription  # prevent unused variable warning

        # Initialize asyncio event loop in a separate thread
        self.async_loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.start_async_loop, args=(self.async_loop,))
        self.thread.start()

        # Start the WebSocket server
        self.websocket_server = None
        self.start_websocket_server()

    def start_async_loop(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def start_websocket_server(self):
        # Ensure the WebSocket server is started by scheduling it in the event loop
        asyncio.run_coroutine_threadsafe(self._start_websocket_server(), self.async_loop)

    async def _start_websocket_server(self):
        # websockets.serve is a coroutine, we need to await it here
        self.websocket_server = await websockets.serve(self.ws_conn_handler, "0.0.0.0", self.server_port)
        self.get_logger().info(f"WebSocket server started at ws://0.0.0.0:{self.server_port}")

    async def ws_conn_handler(self, websocket, path):
        # Handle the WebSocket connection handshake
        await websocket.send(json.dumps({"status": "WebSocket server running"}))
        async for message in websocket:
            print(f"Received message from client: {message}")

    async def send_json(self, diagnostic_data):
        if hasattr(self, 'websocket') and self.websocket:  # Ensure WebSocket connection is established
            diagnostic_json = json.dumps(diagnostic_data, indent=4)
            await self.websocket.send(diagnostic_json)  # Send diagnostic data to the WebSocket client

    def diagnostics_callback(self, msg):
        # Build a dictionary of diagnostic data
        diagnostic_list = []
        for status in msg.status:
            status_dict = {
                'name': status.name,
                'message': status.message,
                'hardware_id': status.hardware_id,
                'values': {}
            }
            for value in status.values:
                status_dict['values'][value.key] = value.value
            diagnostic_list.append(status_dict)
        
        diagnostic_data = {status['name']: status for status in diagnostic_list}

        # Send JSON data over WebSocket
        asyncio.run_coroutine_threadsafe(self.send_json(diagnostic_data), self.async_loop)

        # Flask route
        @app.routine('/')
        def index():
            return
        render_template('diagnostics_view.html')

def main(args=None):
    rclpy.init(args=args)
    
    # Update the WebSocket URL and port for the server to use
    websocket_url = 'ws://localhost:8080'
    server_port = 8080  # Ensure this port matches the client
    
    # Flask in seperate thread
    flask_thread = threading.Thread(target=lambda:app.run(host="0.0.0.0", port=5000, use_reloader=False, debug=False))
    flask_thread.start()

    diagnostics_subscriber = DiagnosticsSubscriber(websocket_url, server_port)
    rclpy.spin(diagnostics_subscriber)

    # Clean up
    diagnostics_subscriber.async_loop.stop()
    diagnostics_subscriber.thread.join()
    diagnostics_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
