//Arduino LIDAR sensor to VRML
//Created by Fabien SANTOS-CESSAC

#include <Wire.h>
#include <LIDARLite.h>

#include <SD.h>
#include <Ethernet.h>
#include <math.h>

#include <Servo.h>

#define SERVOH_PIN 8  //Horizontal servo's pin
#define SERVOV_PIN 9  //Vertical servo's pin
#define SCAN_SPEED 10  //1 = fast, 10 = slow
#define SERVOH_MID 90  
#define SERVOV_MID 95  


LIDARLite myLidarLite;
File myFile;
Servo servoH; //Horizontal servo  --min-mid-max: 0 -90-180  right - left
Servo servoV; //Vertical servo    --min-mid-max: 85-95-180  down  - up

float x = 0, y = 0, z = 0;

char vlow = 85, vhigh = 115, hright = 65, hleft = 115; //scan area
char vpoints = 0, hpoints = 0;
boolean crs = 0; //comma removal system =D
int zCenter = 0; //offset to center the object in the VRML viewer

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(5000);
  Serial.println("initializing stuff...");
  myLidarLite.begin();
  
  if (!SD.begin(4)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");
  
  myFile = SD.open("LIDAR.wrl", FILE_WRITE);
  
  servoH.attach(SERVOH_PIN);
  servoV.attach(SERVOV_PIN);
  
  servoWrite('V',95);
  servoWrite('H',90);
  int d;
  for (int i = 0; i<15; i++) // take 15 dummy reading to "warm up" the sensor
  {
    d = myLidarLite.distance();
    delay(5);
  }
    
  zCenter = d; //offset to center object in VRML viewer
}

void vrmlHeader()
{
  if (myFile)
  {
    myFile.println("#VRML V2.0 utf8");
    myFile.println("#Mr.Fab LIDAR\n");
    myFile.println("NavigationInfo {type \"EXAMINE\"}");
    myFile.println("Shape {");
    myFile.println("geometry IndexedFaceSet {");
    myFile.println("solid FALSE");
    myFile.println("coord Coordinate {");
    myFile.print("point [");
  }
}

void scan(char vl, char vh, char hl, char hh)//moves the servos and measures distance v:vertical, h: horizontal, l:low point, h: high point
{
  if(SD.remove("LIDAR.wrl"))//delete previous file (if any)
    Serial.println("Removed previous file");  
    
  myFile = SD.open("LIDAR.wrl", FILE_WRITE);//open file for writing
  vrmlHeader();//write header
  
  vpoints = vh - vl + 1;//number of points per collumn
  hpoints = hh - hl + 1;//number of points per line

  boolean dir = 0;
  for (int v=vl; v<vh+1; v +=1)
  {  
    servoWrite('V',v);
    delay(20*SCAN_SPEED);
     
    if (!dir)//if dir = 0
    {
      for (int h=hl; h<hh+1; h +=1)//count up
      {
        servoWrite('H',h);
        delay(20*SCAN_SPEED);
        measureDist(v-5,h);//v is 5 degrees off => send the actual angle to measure function
      }
    }
    else
    {
      for (int h=hh; h>hl-1; h -=1)//count down
      {
        servoWrite('H',h);
        delay(20*SCAN_SPEED);
        measureDist(v-5,h);//v is 5 degrees off => send the actual angle to measure function
      }
    }
    dir = !dir;//toggle dir
  }
  
  if (myFile)
  {
    myFile.println("         ]");//close Coordinate
    myFile.println("      }");//close IndexedFaceSet
    
    //       ### coordIndex [] TO MAKE TRIANGLES ###
    myFile.println("coordIndex [");
    for (int line = 0; line < vpoints-1; line++)
    {
      int offset = line * hpoints;
      for(int i = 1; i<hpoints; i++)
      {
        myFile.print(i + offset);
        myFile.print(" ");
        myFile.print(2 * hpoints - i + offset);
        myFile.print(" ");
        myFile.print(2 * hpoints - i - 1 + offset);
        myFile.print(" ");
        myFile.print("-1, ");
        //
        myFile.print(i - 1 + offset);
        myFile.print(" ");
        myFile.print(i + offset);
        myFile.print(" ");
        myFile.print(2 * hpoints - i + offset);
        myFile.print(" ");
        
        if((line == (vpoints - 2)) && (i == hpoints - 1))//Last Data Point ### UPDATE ###
          myFile.print("-1");
        else
          myFile.print("-1, ");
      }
    }
    myFile.println("]");
    //    ### coordIndex [] TO MAKE TRIANGLES ###
    
    myFile.println("   }\n}");
    myFile.close();
  }
}

void servoWrite(char c, int pos)
{
  if((c == 'H') || (c == 'h'))//Horizontal servo
    servoH.write(pos);
  else if ((c == 'V') || (c == 'v'))//Vertical servo
  {
    if(pos < 85)
      pos = 85;
    servoV.write(pos);
  }
}

void measureDist(int v, int h)//measure distance and store it to SD card
{
  int d = myLidarLite.distance();

  Serial.print(h);
  Serial.print(" - ");
  Serial.print(v);
  Serial.print(" : ");
  Serial.println(d);
  spatialTransform(v,h,d);
  
  if (myFile)
  {
    myFile.print("\t");
    myFile.print(x);
    myFile.print("\t");
    myFile.print(y);
    myFile.print("\t");
    myFile.print(z);
    
    if(((h == hleft)||(h == hright)) && (v == (vhigh-5)))//Last Data Point ### UPDATE ###
    {
      if(crs)
        delay(1);//no damn comma
      else
        crs = 1;//arm comma removal system
    }
    else
      myFile.print(",");
  }
  else
    Serial.println("error opening file");
}

void spatialTransform(int v, int h, int d)//transform polar coordinates to cartesian ones
{
  //use global variable to pass result back to parent function
  x = (-h + 90)/10.0;//don't mirror object
  y =  (v - 95)/10.0;
  z =  (d - zCenter)/10.0;
}

void loop()
{
  scan(vlow, vhigh, hright, hleft);
  Serial.println("done");
  while(1)
    delay(100);
  
}


