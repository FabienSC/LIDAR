void servoWrite(char c, int pos)
{
  if(pos < 5)
    pos = 5;
  else if(pos > 185)
    pos = 185;
        
  if((c == 'H') || (c == 'h'))//Horizontal servo
      servoH.write(pos);
  else if ((c == 'V') || (c == 'v'))//Vertical servo
  {
    if(pos < 85)
      pos = 85;
    servoV.write(pos);
  }
}
