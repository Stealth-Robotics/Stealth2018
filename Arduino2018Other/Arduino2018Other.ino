//----------------------------------------------------------------------------
//
//  $Workfile: Arduino2018$
//
//  $Revision: X$
//
//  Project:    Stealth Robotics 2018
//
//                            Copyright (c) 2018
//                          Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
  
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "Common.h"
#include "JORSUtils.h"
#include <avr/wdt.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library

//----------------------------------------------------------------------------
//  Constants
//----------------------------------------------------------------------------
const long  MAX_TIME = 1000;       // Timeout for comm with the Raspberry Pi
const uint8 LED_PIN  =  7;         // LED is on the 6th digial pin
const int   OUR_I2C_ADDR = 43;     // Out I2C address
const int   COLOR_REDUCTION = 27;  // The amount of color to remove on each steop of the spin
const int   BUZZER_PIN = 6;        // Buzzer
const int   YELLOW_LED_PIN = 13;   // The Yellow LED
const int   YELLOW_LED_BLINK = 1000;  // The Yellow LED blink rate

//----------------------------------------------------------------------------
//  Local Class
//----------------------------------------------------------------------------
class StopWatch
{
  long mLastTime;
  long mWaitTime;

        long Now(void);

  public:
  StopWatch();
  StopWatch(int waitTime);
  void SetTime(int waitTime);
  bool IsExpired(void);
  void Reset(void);
        long GetTimeLeft(void);
};


//----------------------------------------------------------------------------
//  Variables
//----------------------------------------------------------------------------
Adafruit_NeoPixel mStrip    = Adafruit_NeoPixel(MAX_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

uint8 mSpinColor = 0;
byte mAlliance = 'O';
byte mLocation = 1;
byte mEnabled = 'D';
byte mExtra = 1;
long mLastTime = 0;

// Comm with Raspberry Pi vars
uint8 mBuffer[MAX_PACKET];
uint8 mBufferLoc = 0;
uint8 mSend[MAX_SEND];
bool mGoodPacket = false;
uint16 mCommCount = 0;

// LED
StopWatch mYellowLED(YELLOW_LED_BLINK); 
bool mYellowLEDState = true;

//----------------------------------------------------------------------------
//  Purpose:
//      Setup
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  Wire.begin(OUR_I2C_ADDR);              
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  mStrip.begin();
  mStrip.show();
  IdlePatternSet();
  mYellowLED.reset();
  
  Serial.print("Starting up");
}

//----------------------------------------------------------------------------
//  Purpose:
//      Idle Loop
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void loop() 
{
  // Handle comm with the Pi
  if(true == mGoodPacket)
  {
    int otherCommCount = getU16FrombyteArray(mBuffer, LOC_DATA_START);
    mCommCount++;
    putU16IntoU8Array(mSend,LOC_DATA_START,mCommCount);
    Serial.print("got:");
    Serial.println(otherCommCount);
    mAlliance = mBuffer[LOC_LED_STATUS];
    RemoveDataForNextMessage(MAX_RECEIVE, true);
    mGoodPacket = false;
  }

  // Handle the LED ring
  if((millis()-mLastTime)>MAX_TIME)
  {
    ColorSet(COLOR_BRIGHT_GREEN);
  }
  else
  {
    if('B' == mAlliance)
    {
      SpinColor(COLOR_BLUE);      
    }
    else
    {
      if('R' == mAlliance)
      {
        SpinColor(COLOR_RED);      
      }
      else
      {
        ColorSet(COLOR_BRIGHT_GREEN);
      }
    }
  }

  if(true == mYellowLED.isExpired())
  {
    mYellowLEDState = !mYellowLEDState;
    digitalWrite(YELLOW_LED_PIN, mYellowLEDState);
    mYellowLED.reset();
  }

  // Let things settle if need be
  delay(1);
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  LED Ring
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Spin The Color Ring
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void SpinColor(int color)
{ 
  int red   = colorArray[(color*3)];
  int blue  = colorArray[(color*3)+1];
  int green = colorArray[(color*3)+2];
  
  for(int i=0;i<16;i++)
  {
    uint8 realLoc = (mSpinColor+i)%16;
    mStrip.setPixelColor(realLoc, mStrip.Color((uint8)red, (uint8)blue, (uint8)green));

    if(0 > (red-COLOR_REDUCTION))
    {
      red = 0;
    }
    else
    {
      red-=COLOR_REDUCTION;
    }
    if(0 > (blue-COLOR_REDUCTION))
    {
      blue = 0;
    }
    else
    {
      blue-=COLOR_REDUCTION;
    }
    
    if(0 > (green-COLOR_REDUCTION))
    {
      green = 0;
    }
    else
    {
      green-=COLOR_REDUCTION;
    }
  }
  mStrip.show();
  delay(25);
  mSpinColor--;
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  I2C Handler
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Send bytes to the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void requestEvent() 
{
  mSend[LOC_START] = SER_START;
  mSend[MAX_SEND - LOC_CHECK_BYTE] = CalcCheckByte(mSend, LOC_PI_STATUS, MAX_SEND - LOC_DATA_END);
  mSend[MAX_SEND - LOC_END] = SER_END;

  for (uint8 index = 0; index < MAX_SEND; index++)
  {
    Wire.write(mSend[index]);
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Get bytes from the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void receiveEvent(int howMany) 
{
  uint8 theByte = 0;
  uint8 count = 0;

  while ((Wire.available() > 0) && (mBufferLoc < MAX_PACKET))
  {
    theByte = Wire.read();

    // Add the byte to the buffer
    mBuffer[mBufferLoc] = theByte;
    mBufferLoc++;
  }

  count = 0;
  // Trim the garbage from the start
  while ((SER_START != mBuffer[0]) && (count < mBufferLoc))
  {
    count++;
  }

  if (count > 0)
  {
    RemoveDataForNextMessage(count, true);
  }

  // find if we are good or have garbage
  int nextMessage = FindNextMessage();

  if (true == DoWeHaveAGoodMessage())
  {
    mGoodPacket = true;
  }
  else
  {
    // Trim garbage if there is any
    if (nextMessage > 0)
    {
      RemoveDataForNextMessage(nextMessage, true);
    }
  }
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  Comm Utils
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Return if the packet is well formed
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
bool DoWeHaveAGoodMessage()
{
  bool returnValue = false;
  uint8 theLength = MAX_RECEIVE;

  if ((mBufferLoc >= theLength) && (mBufferLoc != 0) && (theLength != 0))
  {
    //Is the preamble where it should be
    if ((mBuffer[LOC_START] == SER_START) && (mBuffer[theLength - LOC_END] == SER_END))
    {
      uint8 checkByte = CalcCheckByte(mBuffer, LOC_PI_STATUS, theLength - LOC_DATA_END);

      if (checkByte == mBuffer[theLength - LOC_CHECK_BYTE])
      {
        returnValue = true;
      }
      else
      {
        RemoveDataForNextMessage(theLength, true);
      }
    }
    else
    {
      RemoveDataForNextMessage(theLength, true);
    }
  }
  return returnValue;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Calc a check byte from the data
//
//  Notes:
//      This xors all the data together
//
//----------------------------------------------------------------------------
uint8 CalcCheckByte(uint8* data, uint8 start, uint8 number)
{
  uint8 checkByte = 0xFF;

  for (uint8 index = 0; index < number; index++)
  {
    checkByte ^= data[start + index];
  }
  return checkByte;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Find the next message after the first one
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint8 FindNextMessage()
{
  uint8 nextMessIndex = 0;
  bool found = false;
  uint8 theLength = MAX_RECEIVE;

  //Is the preamble where it should be
  if ((mBuffer[LOC_START] == SER_START)&&(mBufferLoc>2))
  {
    theLength = MAX_RECEIVE;
    //From the end of the message search the rest of what we have gotten
    //for another preamble.
    for (nextMessIndex = 1; nextMessIndex < mBufferLoc; nextMessIndex++)
    {
      if (mBuffer[nextMessIndex] == SER_START)
      {
        //If we found one stop
        found = true;
        break;
      }
    }
  }

  //If we found one the nextMessIndex should be good.
  //If not then set it to 0 and return 0.
  if (found == false)
  {
    nextMessIndex = 0;
  }
  return nextMessIndex;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Trim the front of the buffer
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void RemoveDataForNextMessage(uint8 offset, bool isBad)
{
  int index;

  //Move the first 'offset' number of bytes forward.
  for (index = 0; index < mBufferLoc - offset; index++)  // JSF162 JSF213 Exception
  {
    mBuffer[index] = mBuffer[offset + index];
  }

  //if we have been asked to remove more bytes than we have set the number
  //of bytes to 0.
  if (offset > mBufferLoc)
  {
    mBufferLoc = 0;
  }
  else
  {
    //If not then reduce the number of bytes we have by the offset.
    mBufferLoc -= offset;
  }

  //Move the rest of the message down to right after the 'offset' bytes.
  // Process the rest of the buffer
  for (; index < MAX_PACKET; index++)   // JSF200 JSF162 Exception
  {
    //If we are under the MAX packet size then move the data.
    if ((offset + index) < (mBufferLoc))
    {
      mBuffer[index] = mBuffer[offset + index];
    }
    else
    {
      //If we are over the MAX packet size then clear out the bytes.
      mBuffer[index] = 0;
    }
  }
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  LED Handler
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************

//----------------------------------------------------------------------------
//  Purpose:
//      Return the combined color
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint32_t GetLEDColor(uint8_t colorIndex)
{
  if(colorIndex>=MAX_COLORS)
  {
    colorIndex = COLOR_BLACK; 
  }

  uint8 red   = colorArray[(colorIndex*3)];
  uint8 blue  = colorArray[(colorIndex*3)+1];
  uint8 green = colorArray[(colorIndex*3)+2];

  return(mStrip.Color(red, blue, green));
}

//----------------------------------------------------------------------------
//  Purpose:
//      
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void ColorSet(uint8 team) 
{
  if(COLOR_NO_COMM == team)
  {
    IdlePatternSet();
  }
  else
  {
    {
      uint8 theColor = team;
      
      uint32_t color = GetLEDColor(theColor);
      
      for(uint8 i=0; i<MAX_LEDS; i++) 
      {
          mStrip.setPixelColor(i, color);
      }
      mStrip.show();
    }
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Set the LEDs to the idle pattern 
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void IdlePatternSet() 
{
  uint32_t color = GetLEDColor(COLOR_BLACK);
  uint8 i;
  
  for(i=0; i<4; i++) 
  {
      mStrip.setPixelColor(i, color);
  }
 
  color = GetLEDColor(COLOR_RED);
  for(i=4; i<8; i++) 
  {
      mStrip.setPixelColor(i, color);
  }

  color = GetLEDColor(COLOR_YELLOW);
  for(i=8; i<12; i++) 
  {
      mStrip.setPixelColor(i, color);
  }

  color = GetLEDColor(COLOR_WHITE);
  for(i=12; i<16; i++) 
  {
      mStrip.setPixelColor(i, color);
  }
  
  mStrip.show();
}

//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
//      
//  Local Class
//
//****************************************************************************
//****************************************************************************
//****************************************************************************
//****************************************************************************
StopWatch::StopWatch() :
  mLastTime(0),
  mWaitTime(1000)
{
}
  
StopWatch::StopWatch(int waitTime) :
  mLastTime(0),
  mWaitTime(waitTime)
{
}

long StopWatch::Now(void)
{
  return millis();
}

void StopWatch::SetTime(int waitTime)
{
  mWaitTime = waitTime;
}

bool StopWatch::IsExpired(void)
{
  if((Now() - mLastTime)>mWaitTime)
  {
     return true;
  }
  return false;
}

void StopWatch::Reset(void)
{
  mLastTime = Now();
}

long StopWatch::GetTimeLeft()
{
  return mWaitTime - (Now()-mLastTime);
}


