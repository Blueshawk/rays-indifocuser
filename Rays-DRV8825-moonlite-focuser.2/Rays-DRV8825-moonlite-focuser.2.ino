/*Focuser for Nano And DRV8825

   I originally trimmed this down from Robert Brown's earlier focuser code
   To make it easier to read and then modified it to suit motor_ needs,
   but this is now an amalgam of several different focuser drivers as none quite matched the options I wanted.
*/

#include <Arduino.h>
#include <EEPROM.h>                   // needed for EEPROM
#include "eepromanything.h"           // needed for EEPROM

// #define EEPROMSIZE 512              // ATMEGA168 512 EEPROM
#define EEPROMSIZE 1024               // ATMEGA328P 1024 EEPROM

// define serial port speed - valid values are 9600 14400 19200 28800 38400 57600 115200
#define SerialPortSpeed 9600

// Variables stored in EEPROM - all variables in a struct accessed with ep_storage.(varname)
struct config_t {
  int validdata;                  // if this is 99 then data is valid
  long fposition;                 // last focuser position
  long maxstep;                   // maximum step position
  int stepmode;                   // stepping mode
  // indicates stepmode, full, half, 1/4, 1/8. 1/16. 1/32 [1.2.4.8.16.32] (recommend full to save power)
  double stepsize;                // the step size in microns, ie 7.2
  boolean ReverseDirection;       // relative Forward/Reverse control
  boolean coilPwr;                // controls power to the stepper
  boolean stepsizeenabled;        // if true, controller returns step size
} ep_Storage;

int datasize;                     // will hold size of the struct ep_Storage
int nlocations;                   // number of storage locations available in EEPROM
int currentaddr;                  // address in eeprom of the data store. This increments to reduce eeprom wear
boolean writenow;                 // set true to update values in eeprom when a change is made
boolean found;                    // true if stored data reads okay
long previousMillis = 0L;         // used as a delay whenever the EEPROM settings need to be updated
long motor_interval = 60000L;     // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
int Backlash = 0;                 // backlash value

const String programName = "Ray's Moonlite";
const String programVersion = "1.6.8"; // Does indi use this?

//Control pins for DRV8825 board
//set DRV sleep and reset to high in setup() for simple header pin connection. Can also be hardwired high.

#define motor_Dir     5
#define motor_Step    6
#define slp           7
#define rst           8
#define motor_M0      9  // microstepping lines
#define motor_M1      10
#define motor_M2      11
#define motor_Enable  12

// NOTE: If using microstepping, coil power should always be on or the motor will settle to the nearest full step.

  //Setting CoilPwr false in setup() allows remote enable switching
int boardstate;

// stepontime - time in microseconds that coil power is ON for one step, board requires 2us pulse
int stepontime = 10;

// motor_Speed - time in miliseconds of delay between stepper pulses
const int    motor_SpeedSlowDelay = 20;
const int    motor_SpeedMedDelay = 10;
const int    motor_SpeedFastDelay = 5;
int motor_SpeedDelay = motor_SpeedMedDelay;    // the delay between steps
int motor_Speed = 2;                     // the motor_speed setting 0=slow, 1=medium, 2=fast, default=slow on startup only
int savedmotor_Speed = motor_Speed;       // used to save original speed if slowing down when nearing target position

// NOTE: Use Analogue pins
// define IN and OUT LEDS, associated with PB and stepper moves
#define bledIN A1
#define gledOUT A2
// define Buzzer
#define Buzzer A3

// Default initial positions if not set/overriden by Ascom Driver or Winapp
long currentPosition = 10000L;   // current position
long targetPosition = 10000L;    // target position
long maxFocuserLimit = 100000L;  // arbitary focuser limit
long maxSteps = 100000L;         // maximum position of focuser
long maxIncrement = 100000L;      // maximum number of steps permitted in one move
long minimumPosition = 0L;      // minimum position to avoid focuser damage
boolean gotonewposition = false;  // used by moonlite after an SN command followed by a FG

char inChar;                  // used to read a character from serial port
boolean isMoving = false;     // is the motor_ currently moving
long pos ;                     //this one needs cleanup as it is redefined locally in a routine.

#define MAXCOMMAND 20
char motor_cmd[MAXCOMMAND];         // these are for handling and processing serial commands
char param[MAXCOMMAND];
char line[MAXCOMMAND];
int eoc = 0;    // end of command
int idx = 0;    // index into command string

int tprobe1 = 0;                  // indicate if there is a probe attached to that channel
double ch1tempval = 20.0;         // temperature value for probe

int TSWTHRESHOLD = 20;                 // position at which stepper slows down at it approaches home position
int motor_speedchange = 0; //1= go to slow speed

void updatemotor_SpeedDelay()
{
  switch ( motor_Speed )
  {
    case 0: motor_SpeedDelay = motor_SpeedSlowDelay; break;
    case 1: motor_SpeedDelay = motor_SpeedMedDelay; break;
    case 2: motor_SpeedDelay = motor_SpeedFastDelay; break;
    default: motor_SpeedDelay = motor_SpeedFastDelay; break;
  }
}

void software_Reboot()
{
  //reset DRV8825 driver board
  digitalWrite(rst, LOW);
  // jump to the start of the program
  asm volatile ( "jmp 0");
}

// disable the stepper motor_ outputs - coil power off controlled via ENABLE pin. Avoid if microstepping.
void disableoutput() {
  digitalWrite(motor_Enable, HIGH);
}

// enable the stepper motor_ outputs - coil power on
void enableoutput() {
  digitalWrite(motor_Enable, LOW);
}

// Move stepper anticlockwise
void anticlockwise() {

  if ( !ep_Storage.ReverseDirection )
  {
    digitalWrite(gledOUT, 1 ); // indicate a pulse by lighting the green LED
    digitalWrite(motor_Dir, LOW );
    digitalWrite(motor_Step, 1 );
    delayMicroseconds(stepontime);
    digitalWrite(motor_Step, 0 );
    digitalWrite(gledOUT, 0 ); // turn LED off
  }
  else
  {
    digitalWrite(bledIN, 1 );  // indicate a pulse by lighting the blue LED
    digitalWrite(motor_Dir, HIGH );
    digitalWrite(motor_Step, 1 );
    delayMicroseconds(stepontime);
    digitalWrite(motor_Step, 0 );
    digitalWrite(bledIN, 0 );  // turn LED off
  }
}

// Move stepper clockwise
void clockwise() {
  if ( !ep_Storage.ReverseDirection )
  {
    digitalWrite(bledIN, 1 );  // indicate a pulse by lighting the blue LED
    digitalWrite(motor_Dir, HIGH );
    digitalWrite(motor_Step, 1 );
    delayMicroseconds(stepontime);
    digitalWrite(motor_Step, 0 );
    digitalWrite(bledIN, 0 );  // turn LED off
  }
  else
  {
    digitalWrite(gledOUT, 1 ); // indicate a pulse by lighting the green LED
    digitalWrite(motor_Dir, LOW );
    digitalWrite(motor_Step, 1 );
    delayMicroseconds(stepontime);
    digitalWrite(motor_Step, 0 );
    digitalWrite(gledOUT, 0 ); // turn LED off
  }
}

// Set full or microstepping mode by switching m0,m1,m2 on drv8825
// m0/m1/m2 sets stepping mode 000 = F, 100 = 1/2, 010 = 1/4, 110 = 1/8, 001 = 1/16, 101 = 1/32
// Set the current limit to run in current regulation for microstepping to work correctly.

void setstepmode() {
  switch ( ep_Storage.stepmode )
  {
    case 1:      // full step
      digitalWrite(motor_M0, 0);
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 0);
      break;
    case 2:      // half step
      digitalWrite(motor_M0, 1);
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 0);
      break;
    case 4:
      digitalWrite(motor_M0, 0);
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 0);
      break;
    case 8:
      digitalWrite(motor_M0, 1);
      digitalWrite(motor_M1, 1);
      digitalWrite(motor_M2, 0);
      break;
    case 16:
      digitalWrite(motor_M0, 0);
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 1);
      break;
    case 32:
      digitalWrite(motor_M0, 1);
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 1);
      break;
    default:      // full step if no or improper param
      digitalWrite(motor_M0, 0);
      digitalWrite(motor_M1, 0);
      digitalWrite(motor_M2, 0);
      //ep_Storage.stepmode = 1; //changed to not store this.
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
    writenow = true;             //Update EEPROM
    previousMillis = millis();   // start time interval
  }

  // set halfstep mode
  else if (!strcasecmp( motor_cmd, "SH"))
  {
    ep_Storage.stepmode = 2;
    setstepmode();
    writenow = true;             //Update EEPROM
    previousMillis = millis();   // start time interval
  }

  // returns step mode - half(2) or full(1)
  else if (!strcasecmp( motor_cmd, "GH"))
  {
    if ( ep_Storage.stepmode == 2 )
      Serial.print("FF#");
    else
      Serial.print("00#");
  }

  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "SS"))
  {
    pos = hexstr2long(param);
    ep_Storage.stepmode = pos;
    setstepmode();
    writenow = true;             //Update EEPROM
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

  // :MS# set motor_Speed - time delay between pulses, acceptable values are 00, 01 and 02 which
  // correspond to a slow, med, high
  // ep_Storage Command
  else if (!strcasecmp(motor_cmd, "MS"))
  {
    int pos = decstr2int(param);
    if ( pos == 0 )
      motor_Speed = 0;                    // slow
    else if ( pos == 1 )
      motor_Speed = 1;                    // medium
    else if (pos == 2 )
      motor_Speed = 2;                    // fast
    else
      motor_Speed = 1;
    savedmotor_Speed = motor_Speed;        // remember the speed setting
    updatemotor_SpeedDelay();
  }
  // set backlash
  else if (!strcasecmp(motor_cmd, "YB")) {
    Backlash = decstr2int(param);
  }

  // get backlash set by YB
  else if (!strcasecmp(motor_cmd, "ZB")) {
    char tempString[6];
    sprintf(tempString, "%02X", Backlash);
    Serial.print(tempString);
    Serial.print("#");
  }



  // get the current focuser position
  else if (!strcasecmp( motor_cmd, "GP"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", ep_Storage.fposition);
    Serial.print(tempString);
    Serial.print("#");
  }

  // motor_ is moving - 1 if moving, 0 otherwise
  else if (!strcasecmp( motor_cmd, "GI"))
  {
    if (isMoving ) {
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

  // :MR# get motor_ Speed
  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "MR"))
  {
    Serial.print(motor_Speed);
    Serial.print("#");
  }

  // :MTxxx# set the motor_Speed Threshold
  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "MT"))
  {
    int pos = decstr2int(param);
    if ( pos < 50 )
    {
      pos = 50;
    }
    else if ( pos > 200 )
    {
      pos = 200;
    }
    TSWTHRESHOLD = pos;
  }

  // :MU# Get the motor_Speed Threshold
  // ep_Storage command
  else if (!strcasecmp( motor_cmd, "MU"))
  {
    Serial.print(TSWTHRESHOLD);
    Serial.print("#");
  }

  // :MVx#           None         Set Enable/Disable motor_speed change when moving
  else if (!strcasecmp( motor_cmd, "MV"))
  {
    int pos = decstr2int(param);
    motor_speedchange = pos;
  }

  // :MW#         xxx#      Get if motor_speedchange enabled/disabled
  else if (!strcasecmp( motor_cmd, "MW"))
  {
    Serial.print(motor_speedchange);
    Serial.print("#");
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
    writenow = true;             //Update EEPROM
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
    writenow = true;             //Update EEPROM
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

  // :FG# initiate a move to the target position
  else if (!strcasecmp( motor_cmd, "FG"))
  {
    gotonewposition = true;
    isMoving = true;
  }

  // :FQ# stop a move - HALT
  else if (!strcasecmp( motor_cmd, "FQ"))
  {
    gotonewposition = false;
    isMoving = false;
    targetPosition = currentPosition;
    writenow = true;             //Update EEPROM
    previousMillis = millis();   // start time interval
  }

  // :PH# home the motor_, hard-coded, ignore parameters
  // not implemented in INDI driver
  else if (!strcasecmp( motor_cmd, "PH"))
  {
    gotonewposition = true;
    isMoving = true;
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
    //if ( pos > maxSteps )
    // pos = maxSteps;
    if ( pos < 0 )
      pos = 0;
    currentPosition = pos;
    targetPosition = pos;
    // signal that the focuser position has changed and should be saved to eeprom
    writenow = true;                           //Update EEPROM
    previousMillis = millis();                 // start time interval
    gotonewposition = false;
    isMoving = false;
  }

  // :SNxxxx# set new target position SNXXXX - this is a move command
  // but must be followed by a FG command to start the move
  else if (!strcasecmp( motor_cmd, "SN"))
  {
    pos = hexstr2long(param);
    //if ( pos > maxSteps )
    // pos = maxSteps;
    if ( pos < 0 )
      pos = 0;
    targetPosition = pos;
    gotonewposition = false;
    isMoving = false;
  }

  // :GD# get the current motor_ step delay, only values of 02, 04, 08, 10, 20
  // not used so just return 02
  else if (!strcasecmp( motor_cmd, "GD"))
  {
    Serial.print("02#");
  }

  // :SDxx# set step delay, only acceptable values are 02, 04, 08, 10, 20 which
  // correspond to a stepping delay of 250, 125, 63, 32 and 16 steps
  // per second respectively. Moonlite only
  else if (!strcasecmp( motor_cmd, "SD"))
  {
    // ignore
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
    //if ( pos > maxFocuserLimit )
    //  pos = maxFocuserLimit;
    // avoid setting maxSteps too low
    if ( pos < 10000 )
      pos = 10000;
    // for NEMA17 at 400 steps this would be 5 full rotations of focuser knob
    // for 28BYG-28 this would be less than 1/2 a revolution of focuser knob
    maxSteps = pos;
    // check maxIncement in case its larger
    if ( maxIncrement > maxSteps )
      maxIncrement = maxSteps;
    // signal that the focuser position has changed and should be saved to eeprom
    writenow = true;             //Update EEPROM
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
    software_Reboot();
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

  // :PZxx#	  None			Set the return of user specified stepsize to be OFF - default (0) or ON (1)
  else if ( !strcasecmp( motor_cmd, "PZ"))
  {
    int pos = decstr2int(param);
    if ( pos == 1 )
    {
      ep_Storage.stepsizeenabled = true;
    }
    else   //disable
    {
      ep_Storage.stepsizeenabled = false;
    }
    writenow = true;             //Update EEPROM
    previousMillis = millis();   // start time interval
  }

  // :PPxxxx#  None			Set the step size value - double type, eg 2.1
  else if ( !strcasecmp( motor_cmd, "PP"))
  {
    // convert param to float
    String str = param;
    str = str + "";      // add end of string terminator
    double tempstepsize = (double) str.toFloat();
    if ( tempstepsize < 0 )
    {
      tempstepsize = 0;
    }
    ep_Storage.stepsize = tempstepsize;
    writenow = true;             //Update EEPROM
    previousMillis = millis();   // start time interval
  }

  // :PQ#	  None			Get if stepsize is enabled in controller (true or false, 0/1)
  else if ( !strcasecmp( motor_cmd, "PQ"))
  {
    if (ep_Storage.stepsizeenabled == true)
      Serial.print("01#");
    else
      Serial.print("00#");
  }

  // :PR#	  xxxxx#		Get step size in microns (if enabled by controller)
  else if ( !strcasecmp( motor_cmd, "PR"))
  {
    Serial.print(ep_Storage.stepsize);
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
    currentaddr = 0;
    ResetFocuserDefaults();
    // Set focuser defaults.
    currentPosition = ep_Storage.fposition;
    targetPosition = ep_Storage.fposition;
    maxSteps = ep_Storage.maxstep;
  }
}

void ResetFocuserDefaults()
{
  ep_Storage.validdata = 99;
  ep_Storage.fposition = 10000L;
  ep_Storage.maxstep = 90000L;
  ep_Storage.stepmode = 1;
  ep_Storage.ReverseDirection = false;
  ep_Storage.coilPwr = false;
  ep_Storage.stepsizeenabled = false;
  ep_Storage.stepsize = 10;
  // now write the data to EEPROM
  EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
}

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

// Setup
void setup()
{
  delay(200);

  // initialize serial
  Serial.begin(SerialPortSpeed);

  // turn ON the Buzzer - provide power ON beep
  pinMode(Buzzer, OUTPUT);
  digitalWrite( Buzzer, 1);
  // turn ON both LEDS as power on cycle indicator
  pinMode( bledIN, OUTPUT);
  pinMode( gledOUT, OUTPUT);
  digitalWrite( bledIN, 1 );
  digitalWrite( gledOUT, 1 );
  pinMode(rst, OUTPUT);
  pinMode(slp, OUTPUT);
  //set DRV sleep and reset always high due to simple header construction 5,6,7,8,9,10,11,12
  digitalWrite(rst, HIGH);
  digitalWrite(slp, HIGH);
  // if insalled, a temperature sensor DS18B20
  ch1tempval  = 20.0;
  tprobe1 = 0;        // set probe indicator NOT FOUND

  eoc = 0;
  idx = 0;
  isMoving = false;
  gotonewposition = false;
  memset(line, 0, MAXCOMMAND);

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
    {
      // data was erased so write some default values
      currentaddr = addr;
      found = true;
    }
  }
  if ( found == true )
  {
    // set the focuser back to the previous settings
    // done after this in one hit
    // mark current eeprom address as invalid and use next one
    // each time focuser starts it will read current storage, set it to invalid, goto next location and
    // write values to there and set it to valid - so it doesnt always try to use same locations over and
    // over and destroy the eeprom
    // using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
    EEPROM_readAnything( currentaddr, ep_Storage );
    ep_Storage.validdata = 0;
    EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM

    // goto next free address and write data
    currentaddr += datasize;
    // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if ( currentaddr >= (nlocations * datasize) ) currentaddr = 0;

    ep_Storage.validdata = 99;
    EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
  }
  else
  {
    // set defaults because not found
    ResetFocuserDefaults();
  }

  // Set focuser defaults from saved values in EEPROM.
  currentPosition = ep_Storage.fposition;
  targetPosition = ep_Storage.fposition;
  maxSteps = ep_Storage.maxstep;

  TSWTHRESHOLD = 100;              // stepper slows down when going to Home Position when it reaches this value;

  pinMode(  motor_Dir, OUTPUT );
  pinMode(  motor_Step, OUTPUT );
  pinMode(  motor_M0, OUTPUT );
  pinMode(  motor_M1, OUTPUT );
  pinMode(  motor_M2, OUTPUT );

  // set direction and step to low
  digitalWrite( motor_Dir, 0 );
  digitalWrite( motor_Step, 0 );

  boardstate = 1;
  pinMode( motor_Enable, OUTPUT );    // enable the driver board
  enableoutput();

  setstepmode();

  // remember speed setting
  savedmotor_Speed = motor_Speed;
  TSWTHRESHOLD = 100;                 // position at which stepper slows down at it approaches home position
  motor_speedchange = 1;
  ep_Storage.coilPwr = false;
  writenow = false;

  // turn off the IN/OUT LEDS and BUZZER
  digitalWrite( bledIN, 0 );
  digitalWrite( gledOUT, 0 );
  digitalWrite( Buzzer, 0);
}

// Main Loop
void loop()
{
  // Move the position by a single step if target <> current position
  if ((targetPosition != currentPosition) && (gotonewposition == true))
  {
    if ( boardstate == 0 )       // if board is not enabled, we need to enable it else
    { // it will not step
      boardstate = 1;
      enableoutput();              // have to enable driver board
    }
    // Slow down if approaching home position
    if ( motor_speedchange == 1 )
    {
      // Slow down if approaching home position
      int nearinghomepos = currentPosition - targetPosition;
      nearinghomepos = abs(nearinghomepos);
      if ( nearinghomepos < TSWTHRESHOLD )
      {
        motor_Speed = 0;                   // slow
        updatemotor_SpeedDelay();          // set the correct delay      }
      }
    }

    isMoving = true;
    writenow = true;             //Update EEPROM
    previousMillis = millis();    // prevent update until done moving.

    // Going Anticlockwise to lower position
    if (targetPosition < currentPosition)
    {
      anticlockwise();
      currentPosition--;
    }

    // Going Clockwise to higher position
    if (targetPosition > currentPosition)
    {
      // do not need to check if > maximumPosition as its done when a target command is receieved
      clockwise();
      currentPosition++;
    }
    delay( motor_SpeedDelay );  // required else stepper will not move
  }
  else
  {
    // focuser is NOT moving now, move is completed
    isMoving = false;
    gotonewposition = false;

    // reset motor_Speed
    motor_Speed = savedmotor_Speed;
    updatemotor_SpeedDelay();

    // is it time to update EEPROM settings?
    if ( writenow == true )
    {
      // time to update eeprom
      // decide if we have waited 60s after the last move, if so, update the EEPROM
      long currentMillis = millis();
      if ( (currentMillis - previousMillis > motor_interval) && (writenow == true) )
      {
        previousMillis = currentMillis;    // update the timestamp

        // copy current settings and write the data to EEPROM
        ep_Storage.validdata = 99;
        ep_Storage.fposition = currentPosition;
        ep_Storage.maxstep = maxSteps;
        EEPROM_writeAnything(currentaddr, ep_Storage);    // update values in EEPROM
        writenow = false;
      }
    }

    if ( ep_Storage.coilPwr == false )
    {
      boardstate = 0;
      disableoutput();      // release the stepper coils to save power
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
