"""
  On/Off switch from webpage
"""

import queue
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs

from osgar.node import Node


# Create a queue to share data from handler to main thread
data_queue = queue.Queue()


def web_content(checked):
    content = b"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>Simple On/Off Switch</title>
<style>
  /* Container for the switch */
  .switch {
    position: relative;
    display: inline-block;
    width: 60px;
    height: 34px;
  }

  /* Hide the default checkbox */
  .switch input {
    opacity: 0;
    width: 0;
    height: 0;
  }

  /* The slider */
  .slider {
    position: absolute;
    cursor: pointer;
    top: 0; left: 0; right: 0; bottom: 0;
    background-color: #ccc;
    transition: 0.4s;
    border-radius: 34px;
  }

  /* The circle inside the slider */
  .slider:before {
    position: absolute;
    content: "";
    height: 26px;
    width: 26px;
    left: 4px;
    bottom: 4px;
    background-color: white;
    transition: 0.4s;
    border-radius: 50%;
  }

  /* When checked, change background */
  input:checked + .slider {
    background-color: #2196F3;
  }

  /* Move the circle to the right when checked */
  input:checked + .slider:before {
    transform: translateX(26px);
  }
</style>
</head>
<body>

<h2>Simple On/Off Toggle Switch</h2>

<form method="POST" action="/">
  <label class="switch">
    <input type="checkbox" name="toggle" onchange="this.form.submit()" checked="checked" />
    <span class="slider"></span>
  </label>
</form>

</body>
</html>"""
    if not checked:
        content = content.replace(b'checked="checked"', b'')
    return content


class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print('MD:', self.path)
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(web_content(checked=False))
        return

    def do_POST(self):
        print('MD-POST:', self.path)
        length = int(self.headers['content-length'])
        post_data = self.rfile.read(length).decode('utf-8')
        params = parse_qs(post_data)
        toggle_status = 'toggle' in params

        data_queue.put(toggle_status)

        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(web_content(toggle_status))


def run_server():
    server = ThreadingHTTPServer(('', 8888), MyHandler)
    server.controller = None
    server.serve_forever()


class WebPageSwitch(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('status')

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.controller = self
        self.server_thread.start()

    def run(self):
        while self.is_bus_alive():
            switch_status = data_queue.get()  # blocks until a value is available
            print(f"Received value from HTTP handler: {switch_status}")
            self.publish('status', switch_status)
