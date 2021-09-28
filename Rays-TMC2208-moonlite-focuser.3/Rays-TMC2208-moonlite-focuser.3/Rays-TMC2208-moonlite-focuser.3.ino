/*Focuser for Nano And TMC2208

   I originally trimmed this down from Robert Brown's earlier focuser code
   To make it easier to read and then modified it to suit motor_ needs,
   but this is now an amalgam of several different focuser drivers as none quite matched the options I wanted.
.3
*/

#include <Arduino.h>
#include <EEPROM.h>                   // needed for EEPROM
#include "eepromanything.h"           // needed for EEPROM
#include <AccelStepper.h>
// #define EEPROMSIZE 512              // ATMEGA168 512 EEPROM
#define EEPROMSIZE 1024               // ATMEGA328P 1024 EEPROM

// these are stored in EEPROM - all variables in a structure
struct config_t {
  int validdata;                 // if this is 99 then data is valid
  long fposition;                // last focuser position
  long maxstep;                  // max steps
  int stepmode;                  //stepmode, full, half, 1/4, 1/8. 1/16. 1/32 [1.2.4.8.16.32]
  int accel;               // reused for accel steps per sec * sec
  boolean ReverseDirection;      // reverse direction
  boolean coilPwr;               // coil pwr
  int backlash;              // backlash value
} ep_Storage;

//non saved variable declarations
int datasize;      // will hold size of the struct ep_Storage - 8 bytes
int nlocations;    // number of storage locations available in EEPROM
int currentaddr;   // will be address in eeprom of the data stored
boolean writenow;     // should we update values in eeprom
boolean found;        // did we find any stored values?
char inChar;                  // used to read a character from serial port
long pos;
long previousMillis = 0L;   // used as a delay whenever the EEPROM settings need to be updated
long eprom_interval = 2000L;     // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
const String programName = "Ray's Moonlite";
const String programVersion = "2.1";

// Stepper motor_ stuff, control pins for DRV8825 board
//set DRV sleep and reset to high in setup() to
//allow run due simple to header construction can be hardwired.
#define motor_Dir     5
#define motor_Step    6

#define pdn           8 

#define motor_M1      10
#define motor_M2      11
#define motor_Enable  12

// NOTE: Use Analogue pins
// define IN and OUT LEDS, associated with PB and stepper moves
#define bledIN A1
#define gledOUT A2
// define Buzzer
#define Buzzer A3

// Default initial positions if not set/overriden by Ascom Driver or Winapp
long currentPosition = 15000L;   // current position
long targetPosition = 15000L;    // target position
long maxFocuserLimit = 100000L;  // arbitary focuser limit
long maxSteps = 100000L;         // maximum position of focuser
long maxIncrement = 100000L;      // maximum number of steps permitted in one move
long minimumPosition = 0L;      // minimum position to avoid focuser damage
boolean gotonewposition = false;  //start moving to position set by SN followed by an FG
long millisLastMove = 0;
int maxSpeed = 200; //todo add this to storage
int speed = 200; //for changing speeds during use
int backlash = 20;
int clockDir = 1; // First direction is unknown so this wont do backlash the first move, then changes to cw and ccw
int accel = 700;
int CW = 1;
int CCW = -1;
int stepmode = 1;
#define MAXCOMMAND 20
char motor_cmd[MAXCOMMAND];         // these are for handling and processing serial commands
char param[MAXCOMMAND];
char line[MAXCOMMAND];
int eoc = 0;    // end of command
int idx = 0;    // index into command string
boolean ReverseDirection = false;
int tprobe1 = 0;                  // indicate if there is a probe attached to that channel
double ch1tempval = 20.0;         // temperature value for probe
bool coilPwr = true;
// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, motor_Step, motor_Dir);

// convert hex string to long int
long hexstr2long(char *line)
{
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}

// convert string to int
int decstr2int(char *line)
{
  int ret = 0;
  String Str(line);

  ret = Str.toInt();
  return ret;
}

void setstepmode() {
  switch ( ep_Storage.stepmode )
  {
    case 1:      // Half step --full step not possible with TMC2208 in dir/step mode
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 0);
      break;
    case 2:      
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 0);
      break;
    case 4:
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 1);
      break;
    case 8:
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 0);
      break;
    case 16:
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 1);
      break;
    case 32: //also 16
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 1);
      break;
    default:      // half step if no or improper param
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 0);
      break;
  }
}

// SerialEvent occurs via interupt whenever new data comes in the serial RX. See Moonlite protocol list at end of file
void serialEvent() {

  while (Serial.available() && !eoc)
  {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':')           // : starts the command frame # ends it
    {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND)
      {
        idx = MAXCOMMAND - 1;
      }
    }
    else
    {
      if (inChar == '#')
      {
        eoc = 1;
        idx = 0;
        // process the command string when a hash arrives:
        processCommand(line);
        eoc = 0;
      }
    }
  }
}

// Serial Commands
void processCommand(String command)
{
  memset( motor_cmd, 0, MAXCOMMAND);
  memset(param, 0, MAXCOMMAND);
  int len = strlen(line);
  if (len >= 2)
  {
    strncpy( motor_cmd, line, 2);
  }
  if (len > 2)
  {
    strncpy(param, line + 2, len - 2);
  }

  memset(line, 0, MAXCOMMAND);

  eoc = 0;
  idx = 0;

  // set fullstep mode
  if (!strcasecmp( motor_cmd, "SF"))
  {
    ep_Storage.stepmode = 1;
    setstepmode();
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // set halfstep mode
  else if (!strcasecmp( motor_cmd, "SH"))
  {
    ep_Storage.stepmode = 2;
    setstepmode();
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // whether half-step is enabled or not, moonlite always return "00"
  else if (!strcasecmp( motor_cmd, "GH"))
  {
    if ( ep_Storage.stepmode == 2 )
      Serial.print("FF#");
    else
      Serial.print("00#");
  }

  // set stepmode
  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "SS"))
  {
    pos = hexstr2long(param);
    ep_Storage.stepmode = pos;
    setstepmode();
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // get stepmode
  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "GS"))
  {
    char tempString[6];
    sprintf(tempString, "%02X", ep_Storage.stepmode);
    Serial.print(tempString);
    Serial.print("#");
  }

  // set backlash
  else if (!strcasecmp(motor_cmd, "YB")) {
    backlash = decstr2int(param);
    ep_Storage.backlash = backlash;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // get backlash set by YB
  else if (!strcasecmp(motor_cmd, "ZB")) {
    char tempString[6];
    sprintf(tempString, "%02X", backlash);
    Serial.print(tempString);
    Serial.print("#");
  }

  // get the current focuser position
  else if (!strcasecmp( motor_cmd, "GP"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", stepper.currentPosition());
    Serial.print(tempString);
    Serial.print("#");
  }
  // :SNxxxx# set new target position SNXXXX - this is a move command
  // but must be followed by a FG command to start the move
  else if (!strcasecmp( motor_cmd, "SN"))
  {
    pos = hexstr2long(param);
    if ( pos > maxSteps )
      pos = maxSteps;
    if ( pos < 0 )
      pos = 0;
    //apply backlash
    //Check to see if reversing direction
    if (clockDir == CW && pos < stepper.currentPosition() )              //change from CW to CCW?
    {
      stepper.setCurrentPosition( stepper.currentPosition() + (backlash) );         //apply backlash correction
      clockDir = CCW;
    }

    if (clockDir == CCW && pos > stepper.currentPosition() )  //change from CCW to CW?
    {
      stepper.setCurrentPosition( stepper.currentPosition() - (backlash) );         //apply backlash correction
      clockDir = CW;
    }
    stepper.moveTo(pos);
    targetPosition = pos;
    gotonewposition = false;
  }

  // :FG# initiate a move to the target position
  else if (!strcasecmp( motor_cmd, "FG"))
  {
    stepper.enableOutputs();
    gotonewposition = true; //triggers enable and run in main loop.
  }

  // motor_ is moving - 1 if moving, 0 otherwise
  else if (!strcasecmp( motor_cmd, "GI"))
  {
    if (gotonewposition ) {
      Serial.print("01#");
    }
    else {
      Serial.print("00#");
    }
  }

  // :GT# get the current temperature - moonlite compatible
  else if (!strcasecmp( motor_cmd, "GT"))
  {
    char tempString[6];
    ch1tempval = 20.0;
    int tpval = (ch1tempval * 2);
    sprintf(tempString, "%04X", (int) tpval);
    Serial.print(tempString);;
    Serial.print("#");
  }

  // :GZ# get the current temperature
  else if (!strcasecmp( motor_cmd, "GZ"))
  {
    ch1tempval = 20.0;
    char tempstr[8];
    dtostrf(ch1tempval, 4, 3, tempstr);
    String tempretval(tempstr);
    Serial.print(tempretval);
    Serial.print("#");
  }

  // :GV# firmware value Moonlite
  else if (!strcasecmp(motor_cmd, "GV"))
  {
    Serial.print("10#");
  }

  // :GF# firmware value
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "GF"))
  {
    Serial.println(programName);
    Serial.print(programVersion);
    Serial.print("#");
  }

  // :GM# get the MaxSteps
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "GM"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", maxSteps);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :GY# get the maxIncrement - set to MaxSteps
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "GY"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", maxIncrement);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :GO# get the coilPwr setting
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "GO"))
  {
    String tempString;
    if ( ep_Storage.coilPwr )
      tempString = "01#";
    else
      tempString = "00#";
    Serial.print(tempString);
  }

  // :GR# get the Reverse Direction setting
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "GR"))
  {
    String tempString;
    if ( ep_Storage.ReverseDirection )
      tempString = "01#";
    else
      tempString = "00#";
    Serial.print(tempString);
  }

  // :MX#          None        Save settings to EEPROM
  else if (!strcasecmp( motor_cmd, "MX"))
  {
    // copy current settings and write the data to EEPROM
    ep_Storage.validdata = 99;
    ep_Storage.fposition = currentPosition;
    ep_Storage.maxstep = maxSteps;
    EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
    writenow = false;
  }

  // :SOxx# set the coilPwr setting
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "SO"))
  {
    int pos = decstr2int(param);
    if ( pos == 0 )
      ep_Storage.coilPwr = false;
    else
      ep_Storage.coilPwr = true;
    coilPwr = ep_Storage.coilPwr;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :SRxx# set the Reverse Direction setting
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "SR"))
  {
    int pos = decstr2int(param);
    if ( pos == 0 )
      ep_Storage.ReverseDirection = false;
    else
      ep_Storage.ReverseDirection = true;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }
  // :DMx# set displaystate C or F
  else if ( !strcasecmp( motor_cmd, "DM"))
  {
    // ignore, no lcd
  }

  // :GB# LED backlight value, always return "00" - moonlite
  // not implemented in INDI driver
  else if (!strcasecmp( motor_cmd, "GB"))
  {
    Serial.print("00#");
  }

  // :FQ# stop a move - HALT
  else if (!strcasecmp( motor_cmd, "FQ"))
  {
    gotonewposition = false;
    targetPosition = currentPosition;

    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :PH# home the motor_, hard-coded, ignore parameters
  // not implemented in INDI driver
  else if (!strcasecmp( motor_cmd, "PH"))
  {
    gotonewposition = true;
    targetPosition = 0;
  }

  // :GN# get the new motor_ position (target)
  // not implemented in INDI driver
  else if (!strcasecmp( motor_cmd, "GN"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", targetPosition);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :SPxxxx# set current position to received position - no move SPXXXX
  // in INDI driver, only used to set to 0 SP0000 in reset()
  else if (!strcasecmp( motor_cmd, "SP"))
  {
    pos = hexstr2long(param);
    if ( pos > maxSteps )
      pos = maxSteps;
    if ( pos < 0 )
      pos = 0;
    stepper.setCurrentPosition(pos);
    currentPosition = pos;
    targetPosition = pos;
    // signal that the focuser position has changed and should be saved to eeprom
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
    gotonewposition = false;
  }


  // :GD# get the current motor_ step delay, only values of 02, 04, 08, 10, 20
  // not used so just return 02
  else if (!strcasecmp( motor_cmd, "GD"))
  {
    Serial.print("02");
    Serial.print("#");
  }

  // :SDxx# set step delay, only acceptable values are 02, 04, 08, 10, 20 which
  // correspond to a stepping delay of 250, 125, 63, 32 and 16 steps
  // per second respectively. Moonlite only
  else if (!strcasecmp( motor_cmd, "SD"))
  {
    speed = (maxSpeed / (.5 * hexstr2long(param)));
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :SCxx# set temperature co-efficient XX
  else if (!strcasecmp( motor_cmd, "SC"))
  {
    // do nothing, ignore
  }

  // :GC# get temperature co-efficient XX
  else if (!strcasecmp( motor_cmd, "GC"))
  {
    Serial.print("0#");
  }

  // + activate temperature compensation focusing
  else if (!strcasecmp( motor_cmd, "+"))
  {
    // ignore
  }

  // - disable temperature compensation focusing
  else if (!strcasecmp( motor_cmd, "-"))
  {
    // ignore
  }

  // :PO# temperature calibration offset POXX in 0.5 degree increments (hex)
  else if (!strcasecmp( motor_cmd, "PO"))
  {
    // Moonlite only
    // this adds/subtracts an offset from the temperature reading in 1/2 degree C steps
    // FA -3, FB -2.5, FC -2, FD -1.5, FE -1, FF -.5, 00 0, 01 0.5, 02 1.0, 03 1.5, 04 2.0, 05 2.5, 06 3.0
    // ignore
  }

  // :SMxxx# set new maxSteps position SMXXXX
  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "SM"))
  {
    pos = hexstr2long(param);
    if ( pos > maxFocuserLimit )
      pos = maxFocuserLimit;
    // avoid setting maxSteps too low
    if ( pos < 10000 )
      pos = 10000;
    maxSteps = pos;
    // check maxIncement in case its larger
    if ( maxIncrement > maxSteps )
      maxIncrement = maxSteps;
    // signal that the focuser position has changed and should be saved to eeprom
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :SYxxxx# set new maxIncrement SYXXXX
  else if (!strcasecmp( motor_cmd, "SY"))
  {
    pos = hexstr2long(param);
    // ignore
    maxIncrement = maxSteps;
  }

  // :DSx# disable or enable the display setting
  else if (!strcasecmp( motor_cmd, "DS"))
  {
    // ignore, no display
  }

  // :DG# get display state on or off
  else if (!strcasecmp( motor_cmd, "DG"))
  {
    Serial.print("00#");
  }

  // :GXxxxxx#		      get the time that an LCD screen is displayed for (in milliseconds, eg 2500 = 2.5seconds
  else if ( !strcasecmp( motor_cmd, "GX"))
  {
    char tempString[12];
    sprintf(tempString, "%04X", 2000);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :SXxxxx#	None		Set updatedisplayNotMoving (length of time an LCD page is displayed for in milliseconds
  else if ( !strcasecmp( motor_cmd, "SX"))
  {
    // ignore, no display
  }

  // :TA#  Reboot Arduino
  else if ( !strcasecmp( motor_cmd, "TA"))
  {
    // jump to the start of the program
    asm volatile ( "jmp 0");

  }

  // :PS	  Set temperature precision (9-12 = 0.5, 0.25, 0.125, 0.0625)
  else if ( !strcasecmp( motor_cmd, "PS"))
  {
    // ignore, no probe
  }

  // :PG	  Get temperature precision (9-12)
  else if ( !strcasecmp( motor_cmd, "PG"))
  {
    Serial.print("9#");
  }

  // :PMxx#    None			set update of position on lcd when moving (00=disable, 01=enable)
  else if ( !strcasecmp( motor_cmd, "PM"))
  {
    // ignore
  }

  // :PN#	xx#			get update of position on lcd when moving (00=disable, 01=enable)
  else if ( !strcasecmp( motor_cmd, "PN"))
  {
    Serial.print("00#");
  }

  // :PPxxxx#  None			Set the accel rate - double type, eg 2.1
  else if ( !strcasecmp( motor_cmd, "PP"))
  {
    //convert string to integer
    accel = decstr2int(param);
    /*// convert param to float
      /*String str = param;
      str = str + "";      // add end of string terminator
      double tempaccel = (double) str.toFloat();
    */
    if ( accel < 1 ) {
      accel = 1;
    }
    ep_Storage.accel = accel;
    stepper.setAcceleration(accel);
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }


  // :PR#	  xxxxx#		Get accel rate in steps per secondper second)
  else if ( !strcasecmp( motor_cmd, "PR"))
  {
    Serial.print(accel);
    Serial.print("#");
  }

  // :FM#	  x#			Get Display temp mode (Celsius=0, Fahrenheit=1)
  else if ( !strcasecmp( motor_cmd, "FM"))
  {
    Serial.print("0#");
  }

  // :XY# troubleshooting only - print currentaddr value, use in serial monitor mode is best
  else if (!strcasecmp( motor_cmd, "XY"))
  {
    Serial.print("-#");
  }

  // troubleshooting only - reset focuser defaults
  else if (!strcasecmp(motor_cmd, "XZ"))
  {
    //set all addr invalid
    for (int lp1 = 0; lp1 < nlocations; lp1++ )
    {
      int addr = lp1 * datasize;
      ep_Storage.validdata = 0;
    }
    currentaddr = 0;
    //then reset defaults
    ResetFocuserDefaults();
    loadFromStorage();

    // Set focuser defaults.
    currentPosition = ep_Storage.fposition;
    targetPosition = ep_Storage.fposition;
    maxSteps = ep_Storage.maxstep;

  }
}
void ResetFocuserDefaults()
{

  ep_Storage.validdata = 99;
  ep_Storage.fposition = 15000L;
  ep_Storage.maxstep = 100000L;
  ep_Storage.stepmode = 1;
  ep_Storage.ReverseDirection = false;
  ep_Storage.coilPwr = false;
  ep_Storage.accel = 700;                     //sets accel rate = N steps per second per second.
  ep_Storage.backlash = 0;
  EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
}
void loadFromStorage() {
  currentPosition = ep_Storage.fposition;
  targetPosition = ep_Storage.fposition;
  maxSteps = ep_Storage.maxstep;
  backlash = ep_Storage.backlash;
  accel = ep_Storage.accel;
  stepmode = ep_Storage.stepmode;
  ReverseDirection = ep_Storage.ReverseDirection;

}
////////////////////////// Setup ////////////////////
void setup()
{
  delay(200);

  // initialize serial
  Serial.begin(9600);

  // turn ON the Buzzer - provide power ON beep
  pinMode(Buzzer, OUTPUT);
  digitalWrite( Buzzer, 1);
  // turn ON both LEDS as power on cycle indicator
  pinMode( bledIN, OUTPUT);
  pinMode( gledOUT, OUTPUT);
  digitalWrite( bledIN, 1 );
  digitalWrite( gledOUT, 1 );

  pinMode(pdn, OUTPUT);
  // define pdn low to allowreduced holding current at standstill. 
  digitalWrite(pdn, LOW);

  // start temperature sensor DS18B20
  ch1tempval  = 20.0;
  tprobe1 = 0;        // set probe indicator NOT FOUND
  eoc = 0;
  idx = 0;
  
  gotonewposition = false;
  memset(line, 0, MAXCOMMAND); //clear serial buffer
  millisLastMove = millis(); //init sleep/enable timer
  currentaddr = 0;    // start at 0 if not found later
  found = false;
  writenow = false;
  datasize = sizeof( ep_Storage );    // should be 14 bytes
  nlocations = EEPROMSIZE / datasize;  // for AT328P = 1024 / datasize = 73 locations


  for (int lp1 = 0; lp1 < nlocations; lp1++ )
  {
    int addr = lp1 * datasize;
    EEPROM_readAnything( addr, ep_Storage );
    // check to see if the data is valid
    if ( ep_Storage.validdata == 99 )
    { currentaddr = addr;
      found = true;
    }
  }
  if ( found == true )
  {
    // mark current eeprom address as invalid and use next one
    // each time focuser starts it will read current storage, set it to invalid, goto next location and
    // write values to there and set it to valid - This helps with eprom longevity.
    // using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
    EEPROM_readAnything( currentaddr, ep_Storage );
    ep_Storage.validdata = 0;
    EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
    // goto next free address and write data
    currentaddr += datasize;
    // check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if ( currentaddr >= (nlocations * datasize) ) currentaddr = 0;
    ep_Storage.validdata = 99;
    EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
  }
  else
  {
    // If not "found" reset defaults to eprom
    ResetFocuserDefaults();
  }
  //Reload from storage to variables.
  loadFromStorage();

  //set physical pin arrangement
  pinMode(  motor_Dir, OUTPUT );
  pinMode(  motor_Step, OUTPUT );
  pinMode(  motor_M1, OUTPUT );
  pinMode(  motor_M2, OUTPUT );
  pinMode( motor_Enable, OUTPUT );    // enable the driver board

  // turn off the IN/OUT LEDS and BUZZER
  digitalWrite( bledIN, 0 );
  digitalWrite( gledOUT, 0 );
  digitalWrite( Buzzer, 0);


  //init accelstepper
  stepper.setEnablePin(motor_Enable);
  stepper.setPinsInverted( false, false, true); //direction,step,enable invert - bool
  stepper.setSpeed(speed);
  stepper.setMaxSpeed(speed);
  stepper.setAcceleration(accel);
  stepper.enableOutputs();
  setstepmode();
  writenow = false;

} //end of setup

// Main Loop
void loop()
{
  if (gotonewposition == true)
  {
    stepper.enableOutputs();
    stepper.setMaxSpeed(speed);
    stepper.run();
    currentPosition = stepper.currentPosition();
    millisLastMove = millis();
    previousMillis = millis();         // keep updating previousMillis whilst focuser is moving - for eeprom
    writenow = true;
  }
  else {
    // when stopped release the motor after 2.5sec delay
    if (((millis() - millisLastMove) > 2500) && (coilPwr == true))
    {
      stepper.disableOutputs();
    }
  }
  if (stepper.distanceToGo() == 0) {   // focuser is NOT moving now, move is completed
    stepper.run();
    gotonewposition = false;
  }
  // is it time to update EEPROM settings?
  if ( writenow == true )
  {
    // time to update eeprom
    // decide if we have waited 60s after the last move, if so, update the EEPROM
    long currentMillis = millis();
    if ( (currentMillis - previousMillis > eprom_interval) && (writenow == true) )
    {
      previousMillis = currentMillis;    // update the timestamp

      // copy current settings and write the data to EEPROM
      ep_Storage.validdata = 99;
      ep_Storage.fposition = currentPosition;
      ep_Storage.maxstep = maxSteps;
      ep_Storage.backlash = backlash;
      ep_Storage.coilPwr = coilPwr;
      EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
      writenow = false;
    }
  }
}


//moonlite Protocol command list:
//1  2 3 4 5 6 7 8
//: C #             N/A         Initiate a temperature conversion; the conversion process takes a maximum of 750 milliseconds. The value returned by the :GT# command will not be valid until the conversion process completes.
//: F G #           N/A         Go to the new position as set by the ":SNYYYY#" command.
//: F Q #           N/A         Immediately stop any focus motor_ movement.
//: G C #           XX#         Returns the temperature coefficient where XX is a two-digit signed (2’s complement) hex number.
//: G D #           XX#         Returns the current stepping delay where XX is a two-digit unsigned hex number. See the :SD# command for a list of possible return values.
//: G H #           00# OR FF#  Returns "FF#" if the focus motor_ is half-stepped otherwise return "00#"
//: G I #           00# OR 01#  Returns "00#" if the focus motor_ is not moving, otherwise return "01#"
//: G N #           YYYY#         Returns the new position previously set by a ":SNYYYY" command where YYYY is a four-digit unsigned hex number.
//: G P #           YYYY#         Returns the current position where YYYY is a four-digit unsigned hex number.
//: G T #           YYYY#         Returns the current temperature where YYYY is a four-digit signed (2’s complement) hex number.
//: G V #           DD#         Get the version of the firmware as a two-digit decimal number where the first digit is the major version number, and the second digit is the minor version number.
//: S C X X #       N/A         Set the new temperature coefficient where XX is a two-digit, signed (2’s complement) hex number.
//: S D X X #       N/A         Set the new stepping delay where XX is a two-digit, unsigned hex number. Valid values to send are 02, 04, 08, 10 and 20, which correspond to a stepping delay of 250, 125, 63, 32 and 16 steps per second respectively.
//: S F #           N/A         Set full-step mode.
//: S H #           N/A         Set half-step mode.
//: S N Y Y Y Y #   N/A         Set the new position where YYYY is a four-digit unsigned hex number.
//: S P Y Y Y Y #   N/A         Set the current position where YYYY is a four-digit unsigned hex number.
//: + #             N/A         Activate temperature compensation focusing.
//: - #             N/A         Disable temperature compensation focusing.
//: P O X X #       N/A         Temperature calibration offset, XX is a two-digit signed hex number, in half degree increments.
//:     Y       M       #                                               N/A             Enhance temperature reading (0.125 degree)
//:     Y       B       X       X       #                               N/A             Set backlash where XX is a two-digit unsigned hex number
//:     Z       B       #                                               XX#             Get backlash
//:     Y       T       Y       Y       Y       Y       #               N/A             Set max steps where YYYY is a four-digit unsigned hex number
//:     Z       T       #                                               YYYY#           Get max steps
//:     Y       X       X       X       #                               N/A             Set TempComp threshold where XX is a two-digit unsigned hex number in unit of 0.25 degree
//:     Z       X       #                                               XX#             Get TempComp threshold
//: Y       + #           N/A         Activate temperature compensation focusing.
//: Y       - #           N/A         Disable temperature compensation focusing.
//: Z       + #           00 or 01#       Get temperature compensation.
//: Z A #           YYYY#         Returns the average temperature * 100 where YYYY is a four-digit signed (2’s complement) hex number.
//Example 1: :PO02# offset of +1°C
//Example 2: :POFB# offset of -2.5°C
