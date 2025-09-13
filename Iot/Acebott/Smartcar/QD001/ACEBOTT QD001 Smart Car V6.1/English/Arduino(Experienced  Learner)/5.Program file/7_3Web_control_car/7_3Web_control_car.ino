#include <vehicle.h>
#include <WiFi.h>  //Import libraries related to WiFi functionality 
#include <WebServer.h>//Import the web server library

const char* ssid = "SmartCar";//WIFI name
const char* password = "12345678"; // WIFI password
vehicle Acebott;
int Speeds = 255;
WebServer server(80);//Create the web server object 
void handleRoot() {//Create a web page, and design the control interface of the smart car on the web page
  String message = "<html>";
  message += "<style> body {-webkit-user-select: none; -khtml-user-select: none;-moz-user-select: none;-ms-user-select: none;user-select: none;}</style><body>";
  message += "<center><h1>moveCar control</h1></center>";
  message +="<center>";
  message += "<button onmousedown=\"moveCar('tl')\" onmouseup=\"moveCar('s')\" ontouchstart=\"moveCar('tl')\" ontouchend=\"moveCar('s')\" style=\"width:200px;height:250px\">Trun left</button>";
  message += "<button onmousedown=\"moveCar('f')\" onmouseup=\"moveCar('s')\" ontouchstart=\"moveCar('f')\" ontouchend=\"moveCar('s')\" style=\"width:200px;height:250px\">Forward</button>";
  message += "<button onmousedown=\"moveCar('tr')\" onmouseup=\"moveCar('s')\" ontouchstart=\"moveCar('tr')\" ontouchend=\"moveCar('s')\" style=\"width:200px;height:250px\">Trun Right</button>";
  message +="</center><center>";
  message += "<button onmousedown=\"moveCar('l')\" onmouseup=\"moveCar('s')\" ontouchstart=\"moveCar('l')\" ontouchend=\"moveCar('s')\" style=\"width:200px;height:250px\">Left</button>";
  message += "<button onmousedown=\"moveCar('b')\" onmouseup=\"moveCar('s')\" ontouchstart=\"moveCar('b')\" ontouchend=\"moveCar('s')\" style=\"width:200px;height:250px\">Backward</button>";
  message += "<button onmousedown=\"moveCar('r')\" onmouseup=\"moveCar('s')\" ontouchstart=\"moveCar('r')\" ontouchend=\"moveCar('s')\" style=\"width:200px;height:250px\">Right</button>";
  message +="</center>";
  message += "<script>";
  message += "function moveCar(move) {";
  message += "  fetch('/Car?move=' + move);";
  message += "}";
  message += "</script>";
  message += "</body></html>";
  server.send(200, "text/html", message);
}

void handlemoveCar() {  //Smart car control functions
  if (server.hasArg("move")) {
    String move = server.arg("move");
    Serial.println(move);
    if (move == "f") {
      Acebott.Move(Forward, Speeds); 
    } else if (move == "b") {
      Acebott.Move(Backward, Speeds); 
    } else if (move == "l") {
      Acebott.Move(Contrarotate, Speeds);
    } else if (move == "r") {
      Acebott.Move(Clockwise, Speeds); 
    } else if (move == "s") {
      Acebott.Move(Stop, 0);
    } 
    else if (move == "tl") {
      Acebott.Move(Move_Left, Speeds);
    } 
    else if (move == "tr") {
      Acebott.Move(Move_Right, Speeds);
    } 
  } 
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);//Set the WiFi band
  WiFi.mode(WIFI_AP);//Set WiFi Mode
  WiFi.softAP(ssid, password);//Generating WiFi hotspot
  Serial.print("\r\n");
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());//Print ESP32 IP infomation
  Serial.println("' to connect");
  server.on("/", handleRoot);//Enable the web page generation service
  server.on("/Car", handlemoveCar);//Enable the car control service
  server.begin();
  Serial.println("Server started!");
  Acebott.Init();
  Acebott.Move(Stop, 0);
}
void loop() {
  server.handleClient();//Loop through user requests
}