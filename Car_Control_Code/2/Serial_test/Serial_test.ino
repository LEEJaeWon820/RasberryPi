void setup()
{
  Serial.begin(115200);
}
void loop()
{
  char test = 'A';
  int i;
  
  if(Serial.available() )
  {
    char a;
    a = Serial.read();
    if(a=='A')
    {
      Serial.print("test : A");
    }
    else
    {
      Serial.print("test : not A");
    }
  }
  
  
  /*
  for(i=0;i<16;i++)
  {
    test = test + 1;
    
  Serial.print(test);
  delay(500);
  }
  */
}
