//improving...
//latest: 2016-04-25

#include <SD.h>
#include <Ethernet.h>
#include <SPI.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <math.h>
#include <Servo.h>


#define SERVOH_PIN 8  //Horizontal servo's pin
#define SERVOV_PIN 9  //Vertical servo's pin
#define SCAN_SPEED 10 //1 = fast, 20 = slow
#define SERVOH_MID 90  
#define SERVOV_MID 95

/************ ETHERNET STUFF ************/
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xD1, 0x87 };
byte ip[] = { 169,254,28,77 };
//byte gateway[] = { 192, 168, 1, 1 };      // internet access via router
//byte subnet[] = { 255, 255, 255, 0 };     //subnet mask
EthernetServer server(80);


/************* LIDAAR and Servo stuff ************/
LIDARLite myLidarLite;
Servo servoH; //Horizontal servo  --min-mid-max: 0 -90-180  right - left
Servo servoV; //Vertical servo    --min-mid-max: 85-95-169  down  - up


/************* SD card stuff ************/
#define FILE_NAME "LIDAR.wrl"//file for SD R/W
#define FILE_TMP "tmp.fab"//.fab file for storing z floats temporarily
File myFile;//file for SD R/W
File myTmpFile;//file for SD R/W
const int chipSelect = 4;



//////////////////
float x = 0, y = 0, z = 0;
int vpoints = 0, hpoints = 0;


int vlow = 85, vhigh = 130, hright = 70, hleft = 110; //scan area
//int vlow = 90, vhigh = 96, hright = 85, hleft = 95; //scan area
boolean crs = 0; //comma removal system =D
//int zCenter = 0; //offset to center the object in the VRML viewer
//char palette[][16] = {"0.07 0.16 0.35,","0.14 0.30 0.65,","0.27 0.49 0.73,","0.30 0.40 0.29,","0.25 0.62 0.18"};
//char palette[][16] = {"0.20 0.20 0.20,","0.24 0.24 0.24,","0.28 0.28 0.28,","0.32 0.32 0.32,","0.36 0.36 0.36,","0.40 0.40 0.40,","0.44 0.44 0.44,","0.48 0.48 0.48,","0.52 0.52 0.52,","0.56 0.56 0.56,","0.60 0.60 0.60,","0.64 0.64 0.64,","0.68 0.68 0.68,","0.72 0.72 0.72,","0.76 0.76 0.76,","0.80 0.80 0.80,","0.84 0.84 0.84,","0.88 0.88 0.88,","0.92 0.92 0.92,","0.96 0.96 0.96"};//white = far, black = near
char palette[][16] = {"1.00 1.00 1.00,","0.96 0.96 0.96,","0.92 0.92 0.92,","0.88 0.88 0.88,","0.84 0.84 0.84,","0.80 0.80 0.80,","0.76 0.76 0.76,","0.72 0.72 0.72,","0.68 0.68 0.68,","0.64 0.64 0.64,","0.60 0.60 0.60,","0.56 0.56 0.56,","0.52 0.52 0.52,","0.48 0.48 0.48,","0.44 0.44 0.44,","0.40 0.40 0.40,","0.36 0.36 0.36,","0.32 0.32 0.32,","0.28 0.28 0.28,","0.24 0.24 0.24"};//black = far, white = near
int nb_step = 20;//palette size
float zmin = 4000;
float zmax = 0;
boolean dir = 0;
float pstep = 0;

unsigned int medianFilter[5];
int previousD;


void setup()//#######################
{
  Serial.begin(115200);
  
  pinMode(10, OUTPUT);     // set the SS pin as an output (necessary!)
  digitalWrite(10, HIGH);  // but turn off the W5100 chip!
  
  pinMode(12, INPUT);      // scan button
  digitalWrite(12, HIGH);  // pullup

  myLidarLite.begin();
  servoH.attach(SERVOH_PIN);
  servoV.attach(SERVOV_PIN);
  
  servoWrite('V',95);
  servoWrite('H',90);

  if (!SD.begin(chipSelect))
  {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");
  
  // Debugging complete, we start the server!
  Ethernet.begin(mac, ip);
  server.begin();
}

// How big our line buffer should be. 100 is plenty!
#define BUFSIZ 100

void loop()
{
  char clientline[BUFSIZ];
  int index = 0;
  EthernetClient client = server.available();

  if(digitalRead(12) == 0)//button pressed
    {
      client.connected();
      client.stop();
      scan(vlow, vhigh, hright, hleft, client);
    }
  
  if (client)
  {
    // an http request ends with a blank line
    boolean current_line_is_blank = true;
    
    // reset the input buffer
    index = 0;
    
    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        
        // If it isn't a new line, add the character to the buffer
        if (c != '\n' && c != '\r')
        {
          clientline[index] = c;
          index++;
          // are we too big for the buffer? start tossing out data
          if (index >= BUFSIZ) 
            index = BUFSIZ -1;
          
          // continue to read more data!
          continue;
        }
        
        // got a \n or \r new line, which means the string is done
        clientline[index] = 0;
        
        // Look for substring such as a request to get the root file
        if (strstr(clientline, "GET / ") != 0)
        {
          webHomePage(client);
        }
        else if (clientline[6] == 'v')//VRML file
        {
          myFile = SD.open(FILE_NAME);
          
          if (!myFile)
          {
            Serial.println("!myFile error");
            webScanOnly(client);
            break;
          }
          
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: model/vrml");//Allows the browser to view directly?
          client.println("Content-Disposition: attachment; filename=\"LIDAR.wrl\"");
          client.println();
          
          Serial.println("downloading");
          while (myFile.available())
            client.print((char)myFile.read());
          
          myFile.close();
          
          delay(1);
          client.stop();
        }
        else if (clientline[6] == '3')//3D scan
        {
          //Program stays in this loop until scan is complete
          Serial.println("Scan called");
          webScanPage(client, 0);//display "scanning: 0%"

          scan(vlow, vhigh, hright, hleft, client);//also pass client so it can update the webpage
        }
        else
          web404(client);// everything else is a 404
        break;
      }
    }
  }
}
