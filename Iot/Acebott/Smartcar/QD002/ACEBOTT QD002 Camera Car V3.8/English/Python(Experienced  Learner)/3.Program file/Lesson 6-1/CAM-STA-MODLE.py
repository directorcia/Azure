from microdot import Microdot
import time
import camera
from machine import reset,UART,Pin
import network
import _thread
import time
import socket
import select

dataLen = 0
index_a = 0
buffer = bytearray(52)
receiveBuffer = bytearray()
prevc = 0
isStart = False
ED_client = True
WA_en = False
sendBuff = ""
val = 0
tcp_server = None

gpLED = Pin(4, Pin.OUT)

ssid = "ACEBOTT"      #Set WiFi name
password = "12345678" #Set WiFi passwor

try:
    camera.init(0, format=camera.JPEG)
except OSError:
    camera.deinit()
    camera.init(0, format=camera.JPEG)

# Picture up and down direction Settings
camera.flip(1)
# Picture left and right direction Settings
camera.mirror(1)

#camera.framesize(camera.FRAME_SXGA) #High picture quality 1280x1024
camera.framesize(camera.FRAME_SVGA) #Medium quality 800x600
#camera.framesize(camera.FRAME_QVGA) #Low picture quality 320x240

def readBuffer(index):
    return buffer[index]

def writeBuffer(index, c):
    buffer[index] = c

print("Camera ready")

def connect():
    global tcp_server
    
    for _ in range(1):
        gpLED.on()  # turn on LED
        time.sleep(0.2)
        gpLED.off()  # turn off LED
        time.sleep(0.2)
        
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    print("Connecting to WiFi...")
    # Wait for connection
    for _ in range(5):
        if wlan.isconnected():
            ip_adr = wlan.ifconfig()[0]
            print("Connected, IP:", ip_adr)
            return
        time.sleep(3)
        
    

connect()
app = Microdot()

    
@app.route('/')
def index(request):
    return '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-CAM</title>
    <style>
      body {
        font-family: '微软雅黑', '宋体', 'Helvetica Neue', Arial, sans-serif;
        background-color: #f4f4f4;
        color: #333;
        text-align: center;
        margin: 0;
        padding: 0;
      }
      h1 {
        color: #4CAF50;
      }
      .video-container {
        width: 100%;
        margin: 5px auto;
        box-shadow: 0 4px 8px 0 rgba(0,0,0,0.2);
        border-radius: 10px;
        overflow: hidden;
      }
      .video-container img {
        width: 100%;
        height: auto;
        border-radius: 5px;
      }
    </style>
  </head>
  <body>
    <h1>ESP32-CAM</h1>
    <div class="video-container">
      <img src="/stream" alt="Video Feed">
    </div>
  </body>
</html>
''', 200, {'Content-Type': 'text/html'}

  
def start_servers():
    global isStart, index_a, dataLen, prevc, st, ED_client, client, server,tcp_server
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.bind(('0.0.0.0', 100))
    tcp_server.listen(1)
    client, _ = tcp_server.accept()
    if client:
        WA_en = True
        ED_client = True
        print("[Client connected]")
        uart = UART(2, 115200, tx=13, rx=14)
        while client:
            
            ready = select.select([client], [], [], 0.0001)
            if ready[0]:
                clientBuff =  client.read(1)[0] & 0xFF
                print(clientBuff)
                uart.write(bytes([clientBuff]))

        client.close()
        print("[Client disconnected]")
        move(Stop, 0)#Stop
    else:
        if ED_client:
          ED_client = False
    
@app.route('/stream')
def stream(request):
    def stream():
        yield b'--frame\r\n'
        while True:
            frame = camera.capture()
            yield b'Content-Type: image/jpeg\r\n\r\n' + frame + \
                b'\r\n--frame\r\n'

    return stream(), 200, {'Content-Type':
                           'multipart/x-mixed-replace; boundary=frame'}

    
if __name__ == '__main__':
    
    _thread.start_new_thread(start_servers, ())  # Start the start_servers function in a new thread
    app.run(debug=True, port=81)  # Run the Microdot server


