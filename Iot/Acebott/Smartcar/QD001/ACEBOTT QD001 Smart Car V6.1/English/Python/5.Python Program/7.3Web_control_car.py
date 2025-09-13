import network
import socket
from machine import Pin
import time
import libs.vehicle


# WiFi configuration
ssid = 'ESP32-Car'
password = '12345678'

# Vehicle object
vehicle = libs.vehicle.ACB_Vehicle()
speeds = 255

# Set up the WiFi access point
ap = network.WLAN(network.AP_IF)
ap.config(essid=ssid, password=password,authmode=network.AUTH_WPA2_PSK)
ap.active(True)

# Wait for the AP to be active
while not ap.active():
    time.sleep(1)

print('Car Ready! Use "http://{}" to connect'.format(ap.ifconfig()[0]))

# Create a simple web server
def web_page():
    html = """<html>
<style> body {-webkit-user-select: none; -khtml-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none;}</style>
<body>
<center><h1>moveCar control</h1></center>
<center>
<button ontouchstart="moveCar('tl')" ontouchend="moveCar('s')" style="width:200px;height:250px">Turn Left</button>
<button ontouchstart="moveCar('f')" ontouchend="moveCar('s')" style="width:200px;height:250px">Forward</button>
<button ontouchstart="moveCar('tr')" ontouchend="moveCar('s')" style="width:200px;height:250px">Turn Right</button>
</center><center>
<button ontouchstart="moveCar('l')" ontouchend="moveCar('s')" style="width:200px;height:250px">Left</button>
<button ontouchstart="moveCar('b')" ontouchend="moveCar('s')" style="width:200px;height:250px">Backward</button>
<button ontouchstart="moveCar('r')" ontouchend="moveCar('s')" style="width:200px;height:250px">Right</button>
</center>
<script>
function moveCar(move) {
    fetch('/Car?move=' + move);
}
</script>
</body>
</html>"""
    return html

# Set up the socket
addr = socket.getaddrinfo('0.0.0.0', 80)[0]
s = socket.socket()
s.bind(('0.0.0.0', 80))
s.listen(1)

vehicle.move(vehicle.Stop, 0)

while True:
    cl, addr = s.accept()
    request = cl.recv(1024)
    request = str(request)
    if '/Car?move=' in request:
        move = request.split('/Car?move=')[1].split(' ')[0]
        if move == 'f':
            vehicle.move(vehicle.Forward, speeds)
        elif move == 'b':
            vehicle.move(vehicle.Backward, speeds)
        elif move == 'l':
            vehicle.move(vehicle.Contrarotate, speeds)
        elif move == 'r':
            vehicle.move(vehicle.Clockwise, speeds)
        elif move == 's':
            vehicle.move(vehicle.Stop, 0)
        elif move == 'tl':
            vehicle.move(vehicle.Move_Left, speeds)
        elif move == 'tr':
            vehicle.move(vehicle.Move_Right, speeds)
    
    # Send the web page response
    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(web_page())
    cl.close()
