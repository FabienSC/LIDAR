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
#define SCAN_SPEED 15 //1 = fast, 10 = slow
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
Servo servoV; //Vertical servo    --min-mid-max: 85-95-180  down  - up


/************* SD card stuff ************/
#define FILE_NAME "LIDAR.wrl"//file for SD R/W
File myFile;//file for SD R/W
const int chipSelect = 4;



//////////////////
int cntr = 0;
float x = 0, y = 0, z = 0;
char vpoints = 0, hpoints = 0;


char vlow = 95, vhigh = 109, hright = 95, hleft = 109; //scan area
boolean crs = 0; //comma removal system =D
int zCenter = 0; //offset to center the object in the VRML viewer
int avgFilter = 0;
char palette[][16] = {"0.07 0.16 0.35,","0.14 0.30 0.65,","0.27 0.49 0.73,","0.30 0.40 0.29,","0.25 0.62 0.18"};
int nb_step = 5;
int zmin = 4000;
int zmax = 0;
boolean dir = 0;
int cmp = 0;
int temp = 0;
int pstep = 0;



void setup()//#######################
{
  Serial.begin(115200);
  
  pinMode(10, OUTPUT);     // set the SS pin as an output (necessary!)
  digitalWrite(10, HIGH);  // but turn off the W5100 chip!

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

  //avgFilter = d;
}

// How big our line buffer should be. 100 is plenty!
#define BUFSIZ 100

void loop()
{
  char clientline[BUFSIZ];
  int index = 0;
  
  EthernetClient client = server.available();
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
            webScanOnly(client);
            break;
          }
          
          webFileDownload(client);
          
          while (myFile.available())
            client.print((char)myFile.read());
          
          myFile.close();
        }
        else if (clientline[6] == '3')//3D scan
        {
          //Program stays in this loop until scan is complete
          
          webScanPage(client, 0);//display "scanning: 0%"

          scan(vlow, vhigh, hright, hleft);
          
          boolean myFlag = 0;
          int myInt = 0;
          cntr = 0;
          /////////////////////////////////////////////////////////////////////////////////
          while(myFlag)
          {
            delay(10);
            Serial.println(myInt);
            myInt++;

            EthernetClient client = server.available();
            if(clientRequest(client))
            {
              cntr += 10;
              webScanPage(client, cntr);
              
              if(cntr > 99)
                myFlag = 0;
            }
          }
          
        }
        else
        {
          web404(client);// everything else is a 404
        }
        break;
      }
    }
  }
}
