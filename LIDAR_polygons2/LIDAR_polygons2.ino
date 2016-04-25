//Arduino LIDAR sensor to VRML
//Created by Fabien SANTOS-CESSAC

#include <Wire.h>
#include <LIDARLite.h>

#include <SD.h>
#include <Ethernet.h>
#include <SPI.h>
#include <math.h>

#include <Servo.h>

#define SERVOH_PIN 8  //Horizontal servo's pin
#define SERVOV_PIN 9  //Vertical servo's pin
#define SCAN_SPEED 15 //1 = fast, 10 = slow
#define SERVOH_MID 90  
#define SERVOV_MID 95
#define error(s) error_P(PSTR(s))
 
/************ ETHERNET STUFF ************/
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xD1, 0x87 };
byte ip[] = { 169,254,28,77 };
EthernetServer server(80);

/************ SDCARD STUFF ************/
//Sd2Card card;
//SdVolume volume;
//SdFile root;
//SdFile file;

LIDARLite myLidarLite;
File myFile;
Servo servoH; //Horizontal servo  --min-mid-max: 0 -90-180  right - left
Servo servoV; //Vertical servo    --min-mid-max: 85-95-180  down  - up

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


//void error_P(const char* str) {
//  PgmPrint("error: ");
//  SerialPrintln_P(str);
//  if (card.errorCode()) {
//    PgmPrint("SD error: ");
//    Serial.print(card.errorCode(), HEX);
//    Serial.print(',');
//    Serial.println(card.errorData(), HEX);
//  }
//  while(1);
//}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  Serial.println("initializing stuff...");
  
  PgmPrint("Free RAM: ");
  Serial.println(FreeRam());  
  pinMode(10, OUTPUT);                       // set the SS pin as an output (necessary!)
  digitalWrite(10, HIGH);                    // but turn off the W5100 chip!
  
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
    d = (2*myLidarLite.distance()+8*d)/10;
    delay(5);
  }
    
  zCenter = d; //offset to center object in VRML viewer
  avgFilter = d;
}

void vrmlHeader()
{
  if (myFile)
  {
    myFile.println("#VRML V2.0 utf8");
    myFile.println("#Projet LIDAR : Fabien SANTOS-CESSAC Loïc OLÇOMENDY\n");
    myFile.println("NavigationInfo {type \"EXAMINE\"}");
    myFile.println("Viewpoint {");
    myFile.println("position    0 0 0 ");
    myFile.println("description \"Position 1\" ");
    myFile.println("fieldOfView 0.55");
    myFile.println("orientation 0 0 1 0");
    myFile.println("}");
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
  int couleur [hpoints*vpoints];
  int colindex [8 * ((hpoints -1) * (vpoints -1)) - 1 ]; //nombre de coins dans les faces + espaces entre les faces
  
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
        if (z>zmax)
         zmax = z;
        if (z<zmin)
         zmin = z;
        couleur [cmp]= z;
        cmp++;
      }
    }
    else
    {
      for (int h=hh; h>hl-1; h -=1)//count down
      {
        servoWrite('H',h);
        delay(20*SCAN_SPEED);
        measureDist(v-5,h);//v is 5 degrees off => send the actual angle to measure function
        if (z>zmax)
         zmax = z;
        if (z<zmin)
         zmin = z;
        pstep = (zmax - zmin)/nb_step;
        cmp++;
        
      }
    }
    //if (z>zmax)
    //  zmax = z;
    //if (z<zmin)
    //  zmin = z;
    //pstep = (zmax - zmin)/nb_step;
    //couleur [cmp]= z;
    //cmp++;
    dir = !dir;//toggle dir
  }
  
  if (myFile)
  {
    myFile.println("         ]");//close point
    myFile.println("      }");//close Coordinate
    
    //       ### coordIndex [] TO MAKE TRIANGLES ###
    myFile.println("coordIndex [");
    int k=0; //compte combien de cases de colindex sont remplies
    for (int line = 0; line < vpoints-1; line++)
    {
      int offset = line * hpoints;
      for(int i = 1; i < hpoints; i++)
      {
        myFile.print(i + offset);
        colindex[k++] = i + offset;
        //Serial.println(colindex[k-1]);
        myFile.print(" ");
        myFile.print(2 * hpoints - i + offset);
        colindex[k++] = 2 * hpoints - i + offset;
        myFile.print(" ");
        myFile.print(2 * hpoints - i - 1 + offset);
        colindex[k++] = 2 * hpoints - i - 1 + offset;
        //Serial.println(colindex[k-1]);
        myFile.print(" ");
        myFile.print("-1, ");
        colindex[k++]= 0;
        //Serial.println(colindex[k-1]);
        //
        myFile.print(i - 1 + offset);
        colindex[k++] = i - 1 + offset;
        //Serial.println(colindex[k-1]);
        myFile.print(" ");
        myFile.print(i + offset);
        colindex[k++] = i + offset;
        //Serial.println(colindex[k-1]);
        myFile.print(" ");
        myFile.print(2 * hpoints - i + offset);
        colindex[k++] = 2 * hpoints - i + offset;
        //Serial.println(colindex[k-1]);
        myFile.print(" ");
        colindex[k++]= 0;
        
        if((line == (vpoints - 2)) && (i == hpoints - 1))//Last Data Point ### UPDATE ###
          myFile.print("-1");
        else
          myFile.print("-1, ");
      }
    }
    myFile.println("]");
    //    ### coordIndex [] TO MAKE TRIANGLES ###

    //       ### color {} TO COLORIZE ###
    
    myFile.println("color Color {\n color[ ");
    myFile.print(palette[0]);
    myFile.print(palette[1]);
    myFile.print(palette[2]);
    myFile.print(palette[3]);
    myFile.print(palette[4]);
    myFile.println("]\n}\n");
    
    //    ### color {} TO COLORIZE ###
   
    //       ### colorIndex {} ###
    myFile.println("colorIndex [ ");
    pstep = (zmax - zmin)/nb_step;
    for (int n=0; n < (hpoints * vpoints - 1); n++)
    {
        if (couleur[n] < (zmin + pstep))
          couleur[n]= 0;
        else if (couleur[n] < (zmin + pstep*2))
          couleur[n]= 1;
        else if (couleur[n] < (zmin + pstep*3))
          couleur[n]= 2;
        else if (couleur[n] < (zmin + pstep*4))
          couleur[n]= 3;
        else
          couleur[n]= 4;
        //Serial.println(couleur[n]);
    }

    int colorfilter = 0;
    
    for (int l=0; l <  (8 * ((hpoints -1) * (vpoints -1)) ); l++)
    {
//        if ((colindex[l] > 4) || (colindex[l] < 0))
//          colindex[l] = couleur[colindex[l] - 1];
        colindex[l] = couleur[colindex[l]];
        if (abs(colindex[l] - colorfilter) > nb_step)
          colindex[l] = colorfilter;
          colorfilter = colindex[l];
        Serial.print("colindex : ");
        Serial.println(colindex[l]);
        myFile.print(colindex[l]);
        myFile.print(" ");
    }
    myFile.println("]");

    //    ### colorIndex {} ###
    myFile.println("colorPerVertex TRUE");
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

 Serial.println(7);
  int d = myLidarLite.distance();
  
//  if(abs(d-avgFilter)>500)//if the new value sticks out too much (over 5 meters from the previous values)
//    d = avgFilter;
//  avgFilter = (avgFilter*8 + 2*d)/10;

  Serial.print(h);
  Serial.print(" - ");
  Serial.print(v);
  Serial.print(" : ");
  spatialTransform(v,h,d);
  
  Serial.println(d);
  //Serial.println(avgFilter);
  if (myFile)
  {
 Serial.println(10);
 Serial.print("\t");
 Serial.print(-x);
 Serial.print("\t");
 Serial.print(y);
 Serial.print("\t");
 Serial.print(-z+zCenter);//put central point in the 0,0,0 position
    myFile.print("\t");
    myFile.print(-x);
    myFile.print("\t");
    myFile.print(y);
    myFile.print("\t");
    myFile.print(-z+zCenter);//put central point in the 0,0,0 position
    
 Serial.println(11);
    
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
  Serial.println(8);
}

void spatialTransform(int v, int h, int d)//transform polar coordinates to cartesian ones
{
  //use global variable to pass result back to parent function
  //slight fluctuations at v = 90
  
float tmp = pow(d,3)/(1+pow(tan((h-90)*2*PI/360.0),3)+pow(tan((v-90)*2*PI/360.0),3));

if(tmp>0)
  z = pow(tmp,1.0/3.0);//don't mirror object
else
  z = -pow(-tmp,1.0/3.0);//don't mirror object

tmp = pow(d,3)-pow(z,3)*(1+pow(tan((h-90)*2*PI/360.0),3));
if(tmp>0)
  y = pow(tmp,1.0/3.0);//don't mirror object
else
  y = -pow(-tmp,1.0/3.0);//don't mirror object

tmp = pow(d,3)-pow(z,3)-pow(y,3);
if(tmp>0)
  x = pow(tmp,1.0/3.0);//don't mirror object
else
  x = -pow(-tmp,1.0/3.0);//don't mirror object 
x = -x;
//z = -z;
}

void loop()
{
  scan(vlow, vhigh, hright, hleft);
  Serial.println("done");
  while(1)
    delay(100);
  
}


