void scan(int vl, int vh, int hl, int hh, EthernetClient client)//moves the servos and measures distance v:vertical, h: horizontal, l:low point, h: high point
{
 crs = 0; //comma removal system =D
 zmin = 4000;
 zmax = 0;
 dir = 0;
 pstep = 0;
 pstep = 0;

int medianFilter[5];
int previousD = -9999;

  if(SD.remove(FILE_NAME))//delete previous file (if any)
    Serial.println("Removed previous file");  
  if(SD.remove(FILE_TMP))//delete previous file (if any)
      Serial.println("Removed previous tmp");  
    
  myFile = SD.open(FILE_NAME, FILE_WRITE);//open file for writing
  myTmpFile = SD.open(FILE_TMP, FILE_WRITE);//open file for writing
  vrmlHeader();//write header
  
  vpoints = vh - vl + 1;//number of points per collumn
  hpoints = hh - hl + 1;//number of points per line
  Serial.println(hpoints*vpoints);
  
  char color [hpoints*vpoints];
  
  int d;
  for (int i = 0; i<15; i++) // take 15 dummy reading to "warm up" the sensor
  {
    d = (2*myLidarLite.distance()+8*d)/10;
    delay(5);
  }
  delay (5);//tiny delay

  Serial.println("About to scan...");
  
  for (int v=vl; v<vh+1; v +=1)
  {
    servoWrite('V',v);
    delay(10*SCAN_SPEED);
     
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
        myTmpFile.println(z);
      }
         // webScanPage(client, ((v-vl)*100)/(vh-vl));//try to update webpage
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
        myTmpFile.println(z);
      }
       //   webScanPage(client, ((v-vl)*100)/(vh-vl));//try to update webpage
    }
    dir = !dir;//toggle dir
  }

  myTmpFile.close();
  myTmpFile = SD.open(FILE_TMP, FILE_READ);//open file for reading
  char fileContents[10];
  byte index;
  char readChar;
  
    pstep = (zmax - zmin)/nb_step;
    for (int n=0; n < (hpoints * vpoints); n++)
    {
      int tmp = 8888;
      
      fileContents[0] = '\0';
      index = 0;
      readChar = 'F';//not '\n' nor '\r'
      
      while ((readChar != '\n' && readChar != '\r')) 
      {
        readChar = myTmpFile.read();
        if(readChar != '\n' && readChar != '\r')
        {
          fileContents[index++] = readChar;
          fileContents[index] = '\0'; // NULL terminate the array
        }
        else // the character is CR or LF ('\n' or '\r')
        {
          
        readChar = myTmpFile.read();//get rid of second '\n' or '\r'
          if(strlen(fileContents) > 0)
          {
            float tmpFloat = atof(fileContents);
            //do stuff//
            for(int i = 0; i < nb_step; i++)
            {
              if ((tmpFloat > ((zmin + i*pstep)-0.01)) && (tmpFloat < ((zmin + (i+1)*pstep)+0.01)))
              {
                tmp = i;
              }
            }
          }
        }
      }
     color[n] = tmp;
    }
      
  if (myFile)
  {
    myFile.println(" ]");//close point
    myFile.println(" }");//close Coordinate
    
    //       ### coordIndex [] TO MAKE TRIANGLES ###
    myFile.println("coordIndex [");
    
    for (int line = 0; line < vpoints-1; line++)
    {
      int offset = line * hpoints;
      for(int i = 1; i < hpoints; i++)
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

    //       ### color {} TO COLORIZE ###
    
    myFile.println("color Color {\n color[ ");
    for(int i = 0; i < nb_step; i++)
      myFile.print(palette[i]);
    myFile.println("]\n}\n");
    
    //    ### color {} TO COLORIZE ###
   
    //       ### colorIndex {} ###
    myFile.println("colorIndex [ ");

for (int line = 0; line < vpoints-1; line++)
    {
      int offset = line * hpoints;
      for(int i = 1; i < hpoints; i++)
      {
        myFile.print((int)color[i + offset]);
        myFile.print(" ");
        myFile.print((int)color[2 * hpoints - i + offset]);
        myFile.print(" ");
        myFile.print((int)color[2 * hpoints - i - 1 + offset]);
        myFile.print(" ");
        myFile.print((int)color[0]);//useless data
        myFile.print(" ");
        
        myFile.print((int)color[i - 1 + offset]);
        myFile.print(" ");
        myFile.print((int)color[i + offset]);
        myFile.print(" ");
        myFile.print((int)color[2 * hpoints - i + offset]);
        myFile.print(" ");
        myFile.print((int)color[0]);
        myFile.print(" ");
      }
    }
    
    myFile.println("]");

    //    ### colorIndex {} ###
    myFile.println("colorPerVertex TRUE");
    myFile.println("   }\n}");
    myFile.close();
  }

  Serial.println("#Scan Complete#");
}


void measureDist(int v, int h)//measure distance and store it to SD card
{
  int d = myLidarLite.distance();
  if ((d - previousD) > 50)//if bigger than 50cm
  {
    for(int i = 0; i < 5; i++)
    {
      medianFilter[i] = myLidarLite.distance();
    }
    for(int i = 4; i > 0; i--)
      for(int j = 0; j < i; j++)
          if(medianFilter[j] > medianFilter[j+1])
          {
            int tmp = medianFilter[j];
            medianFilter[j] = medianFilter[j+1];
            medianFilter[j+1] = tmp;
          }
    d = medianFilter[2];//take the median value
  }


  if (d < 10)
    d = max(zmax,previousD);
  else
    previousD = d;

    
  Serial.print(h);
  Serial.print(" - ");
  Serial.print(v);
  Serial.print(" / d = ");
  spatialTransform(v,h,d);

  
    
  
  Serial.println(d);
  if (myFile)
  {
    Serial.print("\t");
    Serial.print(x/10.0);
    Serial.print("\t");
    Serial.print(y/10.0);
    Serial.print("\t");
    Serial.println((z)/10.0);//put central point in the 0,0,0 position
    myFile.print("\t");
    myFile.print(-x/10.0);
    myFile.print("\t");
    myFile.print(y/10.0);
    myFile.print("\t");
    myFile.print(z/10.0);//put central point in the 0,0,0 position
    
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

/*
void spatialTransform(int v, int h, int d)//transform polar coordinates to cartesian ones
{
  //use global variable to pass result back to parent function
  //slight fluctuations at v = 90
  
float rho = d;
float theta = -1.5707963 + h*3.1415926/180.0;
float phi = 3.1415926 - (v)*3.1415926/180.0;

x = -rho*cos(theta)*sin(phi);
y = rho*sin(theta)*sin(phi);
z = rho*cos(phi);
}*/
