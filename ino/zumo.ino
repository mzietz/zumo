#include <Wire.h>
#include <LSM303.h>
#include <ZumoMotors.h>

long timer=0;
int vright = 0;
int vleft = 0;

LSM303 imu;
ZumoMotors motors;

int cmd_speed,cmd_angle;
float speed;

const byte numChars = 32;
char tempChars[numChars];
char receivedChars[numChars];
char messageFromPC[numChars] = {0};
boolean newData = false;


void setup() {
  Serial.begin(9600);
  Serial.println("Serial");
  Wire.begin();
  imu.init();
  imu.enableDefault();
}


void loop()
{
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          // this temporary copy is necessary to protect the original data
          //   because strtok() used in parseData() replaces the commas with \0
      parseData();
       newData = false;
  } 
  vright = cmd_speed - (0.4*cmd_angle);
  vleft = cmd_speed + (0.4*cmd_angle);

  
  motors.setRightSpeed(vright);
  motors.setLeftSpeed(vleft);
  delay(1);

  imu.read();
  timer = millis();
  Serial.print("!");Serial.print("AN:");
  Serial.print(timer); Serial.print (",");   
  Serial.print (imu.a.x);Serial.print (",");
  Serial.print (imu.a.y); Serial.print (","); 
  Serial.print (imu.a.z);Serial.print (","); 
  Serial.print (imu.m.x);Serial.print (",");
  Serial.print (imu.m.y);Serial.print (",");  
  Serial.print (imu.m.z);Serial.print(",");
  Serial.print (vleft);
  Serial.print (",");
  Serial.print (vright);
  Serial.println(); 
  
}
 
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  cmd_speed = atoi(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ",");
  cmd_angle = atoi(strtokIndx);     // convert this part to a float
}
  
