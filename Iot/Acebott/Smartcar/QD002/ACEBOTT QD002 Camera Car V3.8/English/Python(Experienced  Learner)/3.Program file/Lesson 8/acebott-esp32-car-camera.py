from microdot import Microdot
import time
import camera
from machine import reset,UART,Pin
import network
import _thread
import time
import socket
import select

dataLen = 0  # Initialize data length to 0
index_a = 0  # Initialize index variable to 0
buffer = bytearray(52)  # Create a buffer of size 52 bytes
receiveBuffer = bytearray()  # Create a buffer for received data
prevc = 0  # Initialize previous command variable to 0
isStart = False  # Flag to check if the process has started
ED_client = True  # Flag to indicate client connection status
WA_en = False  # Flag for WiFi access enabled
sendBuff = ""  # Initialize the send buffer as an empty string
val = 0  # Initialize a generic variable to 0

gpLED = Pin(4, Pin.OUT)# Configure GPIO pin 4 as an output for the LED

ssid = "ESP32-Car"
password = "12345678"

def readBuffer(index):  # Function to read a value from the buffer
    return buffer[index]  # Return the value at the specified index

def writeBuffer(index, c):  # Function to write a value to the buffer
    buffer[index] = c  # Assign the value to the specified index

try:
    camera.init(0, format=camera.JPEG)# Initialize the camera with JPEG format
except OSError:
    camera.deinit()# Deinitialize the camera in case of an error
    camera.init(0, format=camera.JPEG)# Reinitialize the camera with JPEG format

#camera.framesize(camera.FRAME_SXGA) #High picture quality 1280x1024
camera.framesize(camera.FRAME_SVGA) #Medium quality 800x600
# camera.framesize(camera.FRAME_QVGA) #Low picture quality 320x240

# Picture up and down direction Settings
camera.flip(1)
# Picture left and right direction Settings
camera.mirror(1)

print("Camera ready")

def connect():  # Function to set up a WiFi access point and TCP server
    global tcp_server  # Declare the global variable for the TCP server
    
    for _ in range(1):  # Flash the LED to indicate startup
        gpLED.on()  # Turn on the LED
        time.sleep(0.2)
        gpLED.off()  # Turn off the LED
        time.sleep(0.2)
        
    ap = network.WLAN(network.AP_IF) # Create a WiFi access point
    ap.active(True) # Activate the access point
    ap.config(essid=ssid, password=password,authmode=network.AUTH_WPA2_PSK)
    print('Access Point created. Network config:', ap.ifconfig())
    
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.bind(('0.0.0.0', 100))
    tcp_server.listen(1)

connect()  # Call the connect function to set up the network
app = Microdot()  #Create an instance of the Microdot web server

# web video html    
@app.route('/')
def index(request):
    return '''<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8" />

    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>ESP32 CAM</title>
    <style>
      * {
        padding: 0px;
        margin: 0px;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
      }
      .grid-container {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        grid-template-rows: repeat(4, 1fr);
        gap: 10px;
      }
      .grid-item {
        display: flex;
        justify-content: center;
      }
      button {
        background-color: #0d6efd;
        border: none;
        color: white;
        padding: 8px 16px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 12px;
        margin: 6px 3px;
        cursor: pointer;
        -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
      }

      button:hover {
        background-color: #3778e7;
      }
    </style>
  </head>
  <body>
    <img
      id="camera"
      style="background-color: black"
      width="100%"
      height="200"
      
    />
    <div style="display: flex; justify-content: space-around">
      <button onclick="camera(this)" id="StartScreen">Start Screen</button>
      <button onclick="camera(this)" id="CloseScreen">Close Screen</button>
      <button type="button" id="ESP32LED">Open LED</button>
    </div>
    <hr />
    <div style="display: flex; justify-content: center">
      <div class="grid-container">
        <div id="LeftUp" class="grid-item"><button>Left Up</button></div>
        <div id="Forward" class="grid-item"><button>Forward</button></div>
        <div id="RightUp" class="grid-item"><button>Right Up</button></div>
        <div id="Left" class="grid-item"><button>Left</button></div>
        <div class="grid-item"></div>
        <div id="Right" class="grid-item"><button>Right</button></div>
        <div id="LeftDown" class="grid-item"><button>Left Down</button></div>
        <div id="Backward" class="grid-item"><button>Backward</button></div>
        <div id="RightDown" class="grid-item"><button>Right Down</button></div>

        <div class="grid-item">
          <button id="Anticlockwise">Anticlockwise</button>
        </div>
        <div class="grid-item"></div>
        <div class="grid-item"><button id="Clockwise">Clockwise</button></div>
      </div>
    </div>
    <div>
      <span>Speed</span>
      <input
        id="speed"
        type="range"
        max="5"
        min="1"
        step="1"
        value="3"
        style="width: 80%; margin-bottom: 20px; margin-top: 10px"
      />
    </div>
    <div>
      <span>Servo</span>
      <input
        id="servo"
        type="range"
        max="180"
        step="30"
        style="width: 80%; margin-bottom: 20px; margin-top: 10px"
      />
    </div>
    <hr />
    <!-- <div style="display: flex; justify-content: space-around">
      <button onclick="general('LED&value=1')">Open LED</button>
      <button onclick="general('LED&value=0')">Close LED</button>
      <button onclick="mode(this)" id="Shooting">Shooting</button>
    </div>
    <hr /> -->
    <div style="display: flex; justify-content: space-around">
      <button onclick="general('Buzzer&value=1')">Buzzer1</button>
      <button onclick="general('Buzzer&value=2')">Buzzer2</button>
      <button onclick="general('Buzzer&value=3')">Buzzer3</button>
      <button onclick="general('Buzzer&value=4')">Buzzer4</button>
    </div>
    <hr />
    <div style="display: flex; justify-content: space-around">
      <button onclick="general('Track&value=1')">Track1</button>
      <button onclick="general('Track&value=2')">Track2</button>
      <button onclick="mode(this)" id="Avoidance">Avoidance</button>
      <button onclick="mode(this)" id="Follow">Follow</button>
      <button onclick="general('stopA')">Stop</button>
    </div>
    <hr />

    <script>
    window.onload = function() {
        var img = document.getElementById("camera");
        img.src = "http://192.168.4.1:82/stream";  // 
      };
      window.mobileCheck = function () {
        let check = false;
        (function (a) {
          if (
            /(android|bb\d+|meego).+mobile|avantgo|bada\/|blackberry|blazer|compal|elaine|fennec|hiptop|iemobile|ip(hone|od)|iris|kindle|lge |maemo|midp|mmp|mobile.+firefox|netfront|opera m(ob|in)i|palm( os)?|phone|p(ixi|re)\/|plucker|pocket|psp|series(4|6)0|symbian|treo|up\.(browser|link)|vodafone|wap|windows ce|xda|xiino/i.test(
              a
            ) ||
            /1207|6310|6590|3gso|4thp|50[1-6]i|770s|802s|a wa|abac|ac(er|oo|s\-)|ai(ko|rn)|al(av|ca|co)|amoi|an(ex|ny|yw)|aptu|ar(ch|go)|as(te|us)|attw|au(di|\-m|r |s )|avan|be(ck|ll|nq)|bi(lb|rd)|bl(ac|az)|br(e|v)w|bumb|bw\-(n|u)|c55\/|capi|ccwa|cdm\-|cell|chtm|cldc|cmd\-|co(mp|nd)|craw|da(it|ll|ng)|dbte|dc\-s|devi|dica|dmob|do(c|p)o|ds(12|\-d)|el(49|ai)|em(l2|ul)|er(ic|k0)|esl8|ez([4-7]0|os|wa|ze)|fetc|fly(\-|_)|g1 u|g560|gene|gf\-5|g\-mo|go(\.w|od)|gr(ad|un)|haie|hcit|hd\-(m|p|t)|hei\-|hi(pt|ta)|hp( i|ip)|hs\-c|ht(c(\-| |_|a|g|p|s|t)|tp)|hu(aw|tc)|i\-(20|go|ma)|i230|iac( |\-|\/)|ibro|idea|ig01|ikom|im1k|inno|ipaq|iris|ja(t|v)a|jbro|jemu|jigs|kddi|keji|kgt( |\/)|klon|kpt |kwc\-|kyo(c|k)|le(no|xi)|lg( g|\/(k|l|u)|50|54|\-[a-w])|libw|lynx|m1\-w|m3ga|m50\/|ma(te|ui|xo)|mc(01|21|ca)|m\-cr|me(rc|ri)|mi(o8|oa|ts)|mmef|mo(01|02|bi|de|do|t(\-| |o|v)|zz)|mt(50|p1|v )|mwbp|mywa|n10[0-2]|n20[2-3]|n30(0|2)|n50(0|2|5)|n7(0(0|1)|10)|ne((c|m)\-|on|tf|wf|wg|wt)|nok(6|i)|nzph|o2im|op(ti|wv)|oran|owg1|p800|pan(a|d|t)|pdxg|pg(13|\-([1-8]|c))|phil|pire|pl(ay|uc)|pn\-2|po(ck|rt|se)|prox|psio|pt\-g|qa\-a|qc(07|12|21|32|60|\-[2-7]|i\-)|qtek|r380|r600|raks|rim9|ro(ve|zo)|s55\/|sa(ge|ma|mm|ms|ny|va)|sc(01|h\-|oo|p\-)|sdk\/|se(c(\-|0|1)|47|mc|nd|ri)|sgh\-|shar|sie(\-|m)|sk\-0|sl(45|id)|sm(al|ar|b3|it|t5)|so(ft|ny)|sp(01|h\-|v\-|v )|sy(01|mb)|t2(18|50)|t6(00|10|18)|ta(gt|lk)|tcl\-|tdg\-|tel(i|m)|tim\-|t\-mo|to(pl|sh)|ts(70|m\-|m3|m5)|tx\-9|up(\.b|g1|si)|utst|v400|v750|veri|vi(rg|te)|vk(40|5[0-3]|\-v)|vm40|voda|vulc|vx(52|53|60|61|70|80|81|83|85|98)|w3c(\-| )|webc|whit|wi(g |nc|nw)|wmlb|wonu|x700|yas\-|your|zeto|zte\-/i.test(
              a.substr(0, 4)
            )
          )
            check = true;
        })(navigator.userAgent || navigator.vendor || window.opera);
        return check;
      };

      document.querySelector("#servo").addEventListener("input", (event) => {
        fetch(
          `/control?cmd=${event.currentTarget.id}&angle=${event.target.value}`
        ).then((response) => console.log(response.statusText));
        // document.getElementById("info").innerHTML = event.target.value;
      });

      document.querySelector("#speed").addEventListener("input", (event) => {
        fetch(
          `/control?cmd=${event.currentTarget.id}&value=${event.target.value}`
        ).then((response) => console.log(response.statusText));

        // document.getElementById("info").innerHTML = event.target.value;
      });

      let direction = [
        "LeftUp",
        "Forward",
        "RightUp",
        "Left",
        "Right",
        "LeftDown",
        "Backward",
        "RightDown",
        "Anticlockwise",
        "Clockwise",
      ];
      direction.forEach((item) => {
        let element = document.getElementById(item);
        element.addEventListener(
          window.mobileCheck() ? "touchstart" : "mousedown",
          (event) => {
            fetch(`/control?cmd=car&direction=${event.currentTarget.id}`).then(
              (response) => console.log(response.statusText)
            );
          }
        );
        element.addEventListener(
          window.mobileCheck() ? "touchend" : "mouseup",
          (event) => {
            fetch(`/control?cmd=car&direction=stop`).then((response) =>
              console.log(response.statusText)
            );
          }
        );
      });
      
      var button = document.getElementById("ESP32LED");
            var buttonText = ["Close LED", "Open LED"];
            var currentTextIndex = 0;
            button.addEventListener("click", function() {
              button.textContent = buttonText[currentTextIndex];
              if (button.textContent === "Close LED") {
                general('CAM_LED&value=1')
              } else if (button.textContent === "Open LED") {
                general('CAM_LED&value=0')
              }
              currentTextIndex = (currentTextIndex + 1) % buttonText.length;
            });

      function mode(e) {
        console.log(event.currentTarget.id);
        fetch(`/control?cmd=${event.currentTarget.id}`).then((response) =>
          console.log(response.statusText)
        );
      }

      function camera(e) {
        var img = document.getElementById("camera");
        switch (event.currentTarget.id) {
          case "StartScreen":
            img.src = "http://192.168.4.1:82/stream";
            break;
          case "PauseScreen":
            img.src = "/pause";
            break;
          case "CloseScreen":
            img.src = `data:image/jpeg;base64,/9j/4AAQSkZJRgABAQEAYABgAAD/4QAiRXhpZgAATU0AKgAAAAgAAQESAAMAAAABAAEAAAAAAAD/2wBDAAIBAQIBAQICAgICAgICAwUDAwMDAwYEBAMFBwYHBwcGBwcICQsJCAgKCAcHCg0KCgsMDAwMBwkODw0MDgsMDAz/2wBDAQICAgMDAwYDAwYMCAcIDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAz/wAARCAAeAB4DASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD+f+iiigAooooAKKKKACiiigD/2Q==`;
            break;
        }
      }

      function general(data) {
        fetch(`/control?cmd=${data}`).then((response) =>
          console.log(response.statusText)
        );
      }
    </script>
  </body>
</html>
''', 200, {'Content-Type': 'text/html'}

# Function to handle the TCP server in a separate thread  
def start_servers():
    global isStart, index_a, dataLen, prevc, st, ED_client, client, server, previousMillis, clientBuff
    while True:
        client, _ = tcp_server.accept()
        if client:
            WA_en = True
            ED_client = True
            print("[Client connected]")
            uart = UART(2, 115200, tx=13, rx=14)
            previousMillis = time.ticks_ms()
            timeoutDuration = 3000
            while client:
                if (time.ticks_ms() - previousMillis) > timeoutDuration and clientBuff is None and st==True:
                    break
                ready = select.select([client], [], [], 0.0001)
                if ready[0]:
                    try:
                        previousMillis = time.ticks_ms()
                        clientBuff =  client.read(1)[0] & 0xFF
                        st = False
                        if clientBuff == 200:
                            st = True
                        uart.write(bytes([clientBuff]))
                        if clientBuff == 0x55 and not isStart:
                            if prevc == 0xff:
                                index_a = 1
                                isStart = True
                        else:
                            prevc = clientBuff
                            if isStart:
                                if index_a == 2:
                                    dataLen = clientBuff
                                elif index_a > 2:
                                    dataLen -= 1
                                writeBuffer(index_a, clientBuff)
                        index_a += 1
                        if index_a > 120:
                            index_a = 0
                            isStart = False
                        if isStart and dataLen == 0 and index_a > 3:
                            isStart = False
                            device = readBuffer(10)
                            vals = readBuffer(12)
                            if device == 0x06:
                                if vals == 1:  # Turn on LED
                                    gpLED.on()
                                    vals=-1
                                elif (val == 0):  # Turn off LED
                                    gpLED.off()
                                    vals=-1
                            index_a = 0
                        clientBuff = None
                    except OSError as e:
                        data = [0xFF, 0x55, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0C, 0x00, 0x00]    
                        uart.write(bytes(data))    
                        client.close()
                        print("[Client disconnected]")
            data = [0xFF, 0x55, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0C, 0x00, 0x00]    
            uart.write(bytes(data))
            client.close()
            print("[Client disconnected]")
    #         move(Stop, 0)#Stop
        else:
            if ED_client:
              ED_client = False

# video stream
@app.route('/control')
def control(request):
    data = []
    cmd = request.args.get('cmd')
    uart = UART(2, 115200, tx=13, rx=14)
    if cmd == 'car':
        data = [0xFF, 0x55, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0C, 0x00]
        direction = request.args.get('direction')
        direction_map = {
            "Forward": 0x01,
            "Backward": 0x02,
            "Left": 0x03,
            "Right": 0x04,
            "Anticlockwise": 0x09,
            "Clockwise": 0x0A,
            "stop": 0x00,
            "LeftUp": 0x05,
            "RightUp": 0x07,
            "LeftDown": 0x06,
            "RightDown": 0x08
        }
        if direction in direction_map:
            data.append(direction_map[direction])
            
    elif cmd == 'Buzzer':
        value = request.args.get('value', '')
        data = [0xFF, 0x55, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00]
        buzzer_map = {
            "1": 0x01,
            "2": 0x02,
            "3": 0x03,
            "4": 0x04
        }
        if value in buzzer_map:
            data.append(buzzer_map[value])
            
    elif cmd == 'servo':
        angle = int(request.args.get('angle', 0))
        data = [0xFF, 0x55, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, angle]
    elif cmd == 'CAM_LED':
        value = request.args.get('value', '')
        if value == "1": # Turn on LED
            gpLED.on() 
            time.sleep(0.2)
            value=-1;
        elif value == "0":  # Turn off LED
            gpLED.off()
            time.sleep(0.2)
            value=-1
    elif cmd == "speed":
        value = request.args.get('value', '')
        data = [0xFF, 0x55, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0D, 0x00]
        
        speed_map = {
            "1": 0x82,
            "2": 0xA0,
            "3": 0xBE,
            "4": 0xDC,
            "5": 0xFF
        }
        if value in speed_map:
            data.append(speed_map[value])
            
    elif cmd == 'Track':
        value = request.args.get('value', '')
        if value == "1":
            data = [0xFF, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]  # Mode 1
        elif value == "2":
            data = [0xFF, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05]  # Mode 2
        
    elif cmd == 'Avoidance':
        data = [0xFF, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06]
        
    elif cmd == 'Follow':
        data = [0xFF, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07]
        
    elif cmd == 'stopA':
        data = [0xFF, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03]
        
    uart.write(bytes(data))    
    return "OK", 200
start_servers
app2 = Microdot()
@app2.route('/stream')
def stream(request):
    def stream():
        yield b'--frame\r\n'
        while True:
            frame = camera.capture()
            yield b'Content-Type: image/jpeg\r\n\r\n' + frame + \
                b'\r\n--frame\r\n'

    return stream(), 200, {'Content-Type':
                           'multipart/x-mixed-replace; boundary=frame'}

    
def start_web_servers():
    app.run(debug=True, port=80)  # Run web on port 81

def start_stream_server():
    app2.run(debug=True, port=82)  # Run app2 (stream) on port 82

if __name__ == '__main__':
    _thread.start_new_thread(start_servers, ())
    _thread.start_new_thread(start_web_servers, ())  # Start web on port 81
    _thread.start_new_thread(start_stream_server, ())  # Start app on port 82

    

