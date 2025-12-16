#include <ACB_ARM.h>
#include <Ps3Controller.h>
#include <WiFi.h>

#define FIRMWARE_VERSION "ACB_ARM V2.2 20250620"

ACB_ARM ARM;

// WiFi user name and password
const char* ssid = "Robot_Arm";
const char* password = "12345678";
String macAddress = "20:00:00:00:38:40";  // PS3 Bluetooth controller MAC address

int Chassis_input, Shoulder_input, Elbow_input, Claws_input;
int Chassis_slide, Shoulder_slide, Elbow_slide, Claws_slide;
int PTPX,PTPY,PTPZ;
long timing;
long last_time;

bool record = false;

void setup() {    // initialize
  
  Serial.begin(115200);  // set the baud rate to 115200

  ARM.Chassis_angle_adjust(8);  // Default 0
  ARM.Slight_adjust(0,2);   // Default 0,0
  ARM.ARM_init(5,16,17,18); // The parameters are four Servo pins
  ARM.JoyStick_init(32,33,34,35,36,39); // Joystick initialization

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Set the transmit power of WiFi
  WiFi.mode(WIFI_AP); // Set WiFi mode to AP (Access Point) mode
  WiFi.softAP(ssid, password, 9); // Set the WiFi hotspot name and password, and enable AP mode
  IPAddress myIP = WiFi.softAPIP(); // Obtain the IP address of the AP mode
  Serial.print("AP IP address: ");
  Serial.println(myIP); // address: 192.168.4.1
  
  ARM.startAppServer();
  Ps3.begin(macAddress.c_str());

}

void loop() {
  //---------------------JoyStick---------------------
  handleJoystick();
  Ps3_Control(); 

  //---------------------  A P P  --------------------
  // NULL
  if (ARM.val == 3){

  }

  // speed
  // else if (ARM.val == 20) {         // speed
  //   ARM.Speed(50);  
  // } 
  
  // Slide input
  else if (ARM.val == 21) {         // chassis
    Chassis_input = ARM.Chassis_Silde_Angle;
    ARM.ChassisCmd(Chassis_input);  
    ARM.chassis_angle = Chassis_input;
    ARM.getPositon();
    ARM.val = 3;
  } else if (ARM.val == 22) {         // shoulder
    Shoulder_input = ARM.Shoulder_Silde_Angle;
    ARM.ShoulderCmd(Shoulder_input);  
    ARM.shoulder_angle = Shoulder_input;
    ARM.getPositon();
    ARM.val = 3;
  } else if (ARM.val == 23) {         // elbow
    Elbow_input = ARM.Elbow_Silde_Angle;
    ARM.ElbowCmd(Elbow_input);  
    ARM.elbow_angle = Elbow_input;
    ARM.getPositon();
    ARM.val = 3;
  } else if (ARM.val == 24) {         // claws
    Claws_input = ARM.Claws_Silde_Angle;
    ARM.ClawsCmd(Claws_input); 
    ARM.claws_angle = Claws_input; 
    ARM.getPositon();
    ARM.val = 3;
  } 

  // Silde control
  else if (ARM.val == 25) {         // chassis
    Chassis_slide = ARM.Chassis_Silde_Angle;
    ARM.JoyChassisCmd(Chassis_slide);  
    ARM.chassis_angle = Chassis_slide;
    ARM.getPositon();
    ARM.val = 3;
  } else if (ARM.val == 26) {         // shoulder
    Shoulder_slide = ARM.Shoulder_Silde_Angle;
    ARM.JoyShoulderCmd(Shoulder_slide);  
    ARM.shoulder_angle = Shoulder_slide;
    ARM.getPositon();
    ARM.val = 3;
  } else if (ARM.val == 27) {         // elbow
    Elbow_slide = ARM.Elbow_Silde_Angle;
    ARM.JoyElbowCmd(Elbow_slide);  
    ARM.elbow_angle = Elbow_slide;
    ARM.getPositon();
    ARM.val = 3;
  } else if (ARM.val == 28) {         // claws
    Claws_slide = ARM.Claws_Silde_Angle;
    ARM.JoyClawsCmd(Claws_slide);  
    ARM.claws_angle = Claws_slide; 
    ARM.getPositon();
    ARM.val = 3;
  } 

  // Mode control
  else if (ARM.val == 31 && record) {         // save
    ARM.saveState();
    delay(200);
    ARM.val = 3;
  } else if (ARM.val == 32) {         // end Record
    record = true;
  } else if (ARM.val == 33) {         // start Record
    record = false;
  } else if (ARM.val == 34 && !record) {         // Run
    ARM.executeStates();
    delay(200);
    ARM.val = 3;
  } else if (ARM.val == 55 && !record) {         
    ARM.executeStates();
    delay(100);
  } else if (ARM.val == 35 && !record) {         // Reset
    ARM.clearSavedStates();
    delay(200);
    ARM.val = 3;
  } 

  // Mode select 1-6
  else if (ARM.val == 40) {           // 0
    // ARM.mode = 0;
  } else if (ARM.val == 41) {         // 1
    ARM.mode = 1;
  } else if (ARM.val == 42) {         // 2
    ARM.mode = 2;
  } else if (ARM.val == 43) {         // 3
    ARM.mode = 3;
  } else if (ARM.val == 44) {         // 4
    ARM.mode = 4;
  } else if (ARM.val == 45) {         // 5
    ARM.mode = 5;
  } else if (ARM.val == 46) {         // 6
    ARM.mode = 6;
  }
  
  else if (ARM.val == 54) {         // ptp
    ARM.PtpCmd(ARM.PTP_X,ARM.PTP_Y,ARM.PTP_Z);
    
  }

  //zero
  else if (ARM.val == 60) {         // x
    ARM.Zero();
  }

}

void handleJoystick() {
  ARM.get_JoyStick();

  if (ARM.JoyY1 < 50) {  // chassis left
    ARM.chassis_angle = ARM.chassis_angle + 1;
    ARM.JoyChassisCmd(ARM.chassis_angle);
  }

  if (ARM.JoyY1 > 3500) { // chassis right
    ARM.chassis_angle = ARM.chassis_angle - 1;
    ARM.JoyChassisCmd(ARM.chassis_angle);
  }

  if (ARM.JoyX1 < 50) {  // Shoulder down
    if (ARM.limit_z > 0 ){
      ARM.shoulder_angle = ARM.shoulder_angle + 1;
    }
    ARM.JoyShoulderCmd(ARM.shoulder_angle);
  }

  if (ARM.JoyX1 > 4000) {  // Shoulder up
    ARM.shoulder_angle = ARM.shoulder_angle - 1;
    ARM.JoyShoulderCmd(ARM.shoulder_angle);
  }

  if (ARM.JoyX2 < 50) { // Elbow up 
    
    ARM.elbow_angle = ARM.elbow_angle + 1;
    ARM.JoyElbowCmd(ARM.elbow_angle);
  }
  
  if (ARM.JoyX2 > 4000) { // Elbow dwon 
    if (ARM.limit_z > 0 ){
      ARM.elbow_angle = ARM.elbow_angle - 1;
    }
    ARM.JoyElbowCmd(ARM.elbow_angle);
  }

  if (ARM.JoyY2 > 4000) { // Claws open
    ARM.claws_angle = ARM.claws_angle + 1;
    ARM.JoyClawsCmd(ARM.claws_angle);
  }

  if (ARM.JoyY2 < 50) {  // Claws close
    ARM.claws_angle = ARM.claws_angle - 1;
    ARM.JoyClawsCmd(ARM.claws_angle);
  }
}
char Ps3_Key_Steta[20];
void Ps3_Control(){

  if (Ps3.event.button_down.l1) strcpy(Ps3_Key_Steta, "l1_down");      // select button pressed 
  if (Ps3.event.button_up.l1) strcpy(Ps3_Key_Steta, "l1_up");      // select button pressed 
  if (Ps3.event.button_down.r1) strcpy(Ps3_Key_Steta, "r1");     // PS button pressed


  if ((Ps3.data.analog.stick.ly < 90 && Ps3.data.analog.stick.ly > -90) && Ps3.data.analog.stick.lx <= -90) {  // chassis left
    ARM.chassis_angle = ARM.chassis_angle + 1;
    ARM.JoyChassisCmd(ARM.chassis_angle);
  }

  if ((Ps3.data.analog.stick.ly < 90 && Ps3.data.analog.stick.ly > -90) && Ps3.data.analog.stick.lx >= 90) { // chassis right
    ARM.chassis_angle = ARM.chassis_angle - 1;
    ARM.JoyChassisCmd(ARM.chassis_angle);
  }

  if ((Ps3.data.analog.stick.lx <= 90 && Ps3.data.analog.stick.lx >= -90) && Ps3.data.analog.stick.ly <= -90) {  // Shoulder down
    if (ARM.limit_z > 0 ){
     ARM.shoulder_angle = ARM.shoulder_angle + 1;
    }
    ARM.JoyShoulderCmd(ARM.shoulder_angle);
  }

  if ((Ps3.data.analog.stick.lx <= 90 && Ps3.data.analog.stick.lx >= -90) && Ps3.data.analog.stick.ly >= 90) {  // Shoulder up
    ARM.shoulder_angle = ARM.shoulder_angle - 1;
    ARM.JoyShoulderCmd(ARM.shoulder_angle);
  }

  if ((Ps3.data.analog.stick.rx <= 90 && Ps3.data.analog.stick.rx >= -90) && Ps3.data.analog.stick.ry >= 90 ) { // Elbow up 
    ARM.elbow_angle = ARM.elbow_angle + 1;
    ARM.JoyElbowCmd(ARM.elbow_angle);
  }
  
  if ((Ps3.data.analog.stick.rx <= 90 && Ps3.data.analog.stick.rx >= -90) && Ps3.data.analog.stick.ry <= -90 ) { // Elbow dwon 
    if (ARM.limit_z > 0 ){
      ARM.elbow_angle = ARM.elbow_angle - 1;
    }
    ARM.JoyElbowCmd(ARM.elbow_angle);
  }

  if ((Ps3.data.analog.stick.ry < 90 && Ps3.data.analog.stick.ry > -90) && Ps3.data.analog.stick.rx <= -90) { // Claws open
    ARM.claws_angle = ARM.claws_angle + 1;
    ARM.JoyClawsCmd(ARM.claws_angle);
  }

  if ((Ps3.data.analog.stick.ry < 90 && Ps3.data.analog.stick.ry > -90) && Ps3.data.analog.stick.rx >= 90) {  // Claws close
    ARM.claws_angle = ARM.claws_angle - 1;
    ARM.JoyClawsCmd(ARM.claws_angle);
  }

  if (Ps3.event.button_down.ps) {

    Ps3.end();                                       // Exit the PS3 connection
  }

  if (strcmp(Ps3_Key_Steta, "l1_down") == 0) {       // Triangle button

    timing = (millis());                             // Switch to obstacle avoidance mode

    memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta)); 
  }
  
 if (strcmp(Ps3_Key_Steta, "l1_up") == 0) {          // Triangle button
  
  last_time = (millis()) - timing;    
  
 if (last_time > 300 && last_time < 3000) {         // Press for more than 300 milliseconds but less than 3 seconds
        
    ARM.Button_saveState();                         // record mode
  
  }   
 if (last_time > 3000) {                            // Press for more than 3 seconds.
  
    ARM.Button_clearSavedStates();                  // Clear record mode
  
    } 
     memset(Ps3_Key_Steta, 0, sizeof(Ps3_Key_Steta)); 
  }

  if (Ps3.event.button_down.r1) {
    ARM.Button_executeStates();                     // Operation record mode           
  }

}
