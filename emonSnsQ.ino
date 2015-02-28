// This sketch provides continuous single phase monitoring of real power on four CT channels.
// The interrupt-based kernel was kindly provided by Jorg Becker.
// NOTE: This sketch is specifically for a 50 Hz system.
//
// Original Author: Robin Emley (calypso_rae on Open Energy Monitor Forum)
// Addition of Wh totals by: Trystan Lea

#define QUEUE_LENGTH 42

#include <Arduino.h>        // may not be needed, but it's probably a good idea to include this
#include <avr/eeprom.h>

#include <TimerOne.h>
#include <Time.h>

#include "oem_types.h"
#include "mqueue.h"

#include <SPI.h>
#include "RF24.h"
#include "printf.h"

#define HEARTBEAT_PORT  PORTB
#define HEARTBEAT_INP   PINB
#define HEARTBEAT_DDR   DDRB
#define HEARTBEAT_LED   PB1

/*************  USER Configuration *****************************/
                                          // Hardware configuration
//RF24 radio(7,10);                        // Set up nRF24L01 radio on Due SPI bus plus pins 7 & 10
RF24 radio( 6, 10);                        // Set up nRF24L01 radio on Uno SPI bus plus pins 7 & 8
//RF24 radio(9,12); 

#define ADC_TIMER_PERIOD 125 // uS
#define MAX_INTERVAL_BETWEEN_CONSECUTIVE_PEAKS 12 // mS

// In this sketch, the ADC is free-running with a cycle time of ~104uS.

//  WORKLOAD_CHECK is available for determining how much spare processing time there 
//  is.  To activate this mode, the #define line below should be included: 
//#define WORKLOAD_CHECK  

// ----------------- RF setup  ---------------------

unsigned long sendinterval = 1000; // milliseconds

const int UNO = 1;  // Set to 0 if you're not using the UNO bootloader (i.e using Duemilanove)

// aligned to 32-byte adresses, afaik the minimum writing block
unsigned int ee_addresses[] = { 0x00, 0x20, 0x40, 0x60 };
byte eepromSlot = 0;

sStatus oem;
const unsigned long DEFAULT_TIME = 1357041600;

const uint8_t pipes[][6] = { "1emon", "2emon", "3emon", "4emon", "5emon", "6emon", };

//int deloop;
//byte rx_buffer[32];
unsigned long sync;

mQueue mqueue;
//mQueue mtemp;
mQueue* msmts;
mQueueEntry* msmt;
// -----------------------------------------------------

enum polarities {NEGATIVE, POSITIVE};
enum LEDstates {LED_OFF, LED_ON};   
enum voltageZones {NEGATIVE_ZONE, MIDDLE_ZONE, POSITIVE_ZONE};

enum voltageStates { VOLTAGE_ERROR, VOLTAGE_RECOVER, VOLTAGE_FAIL, VOLTAGE_OK};

// ----------- Pinout assignments  -----------
//
// digital pins:
// dig pin 0 is for Serial Rx
// dig pin 1 is for Serial Tx
// dig pin 2 is for the RFM12B module (IRQ) 
// dig pin 10 is for the RFM12B module (SEL) 
// dig pin 11 is for the RFM12B module (SDI) 
// dig pin 12 is for the RFM12B module (SDO) 
// dig pin 13 is for the RFM12B module (CLK) 

// analogue input pins 
const byte voltageSensor = 0;      
const byte currentSensor_CT1 = 1;  
const byte currentSensor_CT2 = 2;      
const byte currentSensor_CT3 = 3;  
const byte currentSensor_CT4 = 4;  


// --------------  general global variables -----------------
//
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'
//
boolean beyondStartUpPhase = false;    // start-up delay, allows things to settle
const byte startUpPeriod = 3;      // in seconds, to allow LP filter to settle
const int DCoffset_I = 512;        // nominal mid-point value of ADC @ x1 scale

int phaseCal_int_CT1;                  // to avoid the need for floating-point maths
int phaseCal_int_CT2;                  // to avoid the need for floating-point maths
int phaseCal_int_CT3;                  // to avoid the need for floating-point maths
int phaseCal_int_CT4;                  // to avoid the need for floating-point maths
long DCoffset_V_long;              // <--- for LPF
long DCoffsetV_min;               // <--- for LPF
long DCoffsetV_max;               // <--- for LPF

long vSamples[ 40];                // provide a round-robin buffer large enough for a whole sine form
byte sampleIdx = 0;                // index into buffer
short samplesPerPhase = 38;        // educated guess for buffer depth, suited for 50Hz systems

short phaseOff2 = samplesPerPhase / 3;          // phase2 is off one third of a buffer
short phaseOff3 = (samplesPerPhase << 1) / 3;   // phase3 is off two/thirds of a buffer
short phaseOff4 = 0;                            // in sync with phase1 (sensor4 currently not used in my setup)

// for interaction between the main processor and the ISR 
/*
volatile boolean dataReady = false;
int sample_V;
int sample_CT1;
int sample_CT2;
int sample_CT3;
int sample_CT4;
*/

// Calibration values
//-------------------
// Two calibration values are used in this sketch: powerCal, and phaseCal. 
// With most hardware, the default values are likely to work fine without 
// need for change.  A compact explanation of each of these values now follows:

// When calculating real power, which is what this code does, the individual 
// conversion rates for voltage and current are not of importance.  It is 
// only the conversion rate for POWER which is important.  This is the 
// product of the individual conversion rates for voltage and current.  It 
// therefore has the units of ADC-steps squared per Watt.  Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).
// 
// powerCal is the RECIPR0CAL of the power conversion rate.  A good value 
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)
//

// Voltage calibration constant:

// AC-AC Voltage adapter is designed to step down the voltage from 230V to 9V
// but the AC Voltage adapter is running open circuit and so output voltage is
// likely to be 20% higher than 9V (9 x 1.2) = 10.8V. 
// Open circuit step down = 230 / 10.8 = 21.3

// The output voltage is then steped down further with the voltage divider which has 
// values Rb = 10k, Rt = 120k (which will reduce the voltage by 13 times.

// The combined step down is therefore 21.3 x 13 = 276.9 which is the 
// theoretical calibration constant entered below.

// Current calibration constant:
// Current calibration constant = 2000 / 22 Ohms burden resistor (The CT sensor has a ratio of 2000:1)

const float ac = 246.0;
const float pcal = 60.606;
const float vcc = 5.0;
const float powerVRatio = (ac*(vcc/1023));
const float powerCal = powerVRatio*(pcal*(vcc/1023));

const float powerCal_CT1 = powerCal; // <---- powerCal value
const float powerCal_CT2 = powerCal; // <---- powerCal value
const float powerCal_CT3 = powerCal; // <---- powerCal value
const float powerCal_CT4 = powerCal; // <---- powerCal value (2000 / 120R burden resistor)

//const float powerCal_CT1 = 0.0416;  // <---- powerCal value  
//const float powerCal_CT2 = 0.0416;  // <---- powerCal value  
//const float powerCal_CT3 = 0.0416;  // <---- powerCal value  
//const float powerCal_CT4 = 0.0416;  // <---- powerCal value  
  
                        
// phaseCal is used to alter the phase of the voltage waveform relative to the
// current waveform.  The algorithm interpolates between the most recent pair
// of voltage samples according to the value of phaseCal. 
//
//    With phaseCal = 1, the most recent sample is used.  
//    With phaseCal = 0, the previous sample is used
//    With phaseCal = 0.5, the mid-point (average) value in used
//
// NB. Any tool which determines the optimal value of phaseCal must have a similar 
// scheme for taking sample values as does this sketch!
// http://openenergymonitor.org/emon/node/3792#comment-18683
const float  phaseCal_CT1 = 0.0;
const float  phaseCal_CT2 = 0.4;
const float  phaseCal_CT3 = 0.6;
const float  phaseCal_CT4 = 0.0;

float phaseCal_voltage;
int phaseCal_voltage_int;

const byte vGoodCycles = 5;
byte voltageGoodDelay;
enum voltageStates voltState;
const unsigned int voltHystLower = 50;
const unsigned int voltHystUpper = 220;

int joules_CT1 = 0;
int joules_CT2 = 0;
int joules_CT3 = 0;
int joules_CT4 = 0;

// for voltage failure detection logic
int voltageThresholdOffset = 220; // in ADC steps from mid-point
long voltageThresholdUpper_long;  // determined once in setup()
long voltageThresholdLower_long;  // determined once in setup()
unsigned long timeOfLastPeakEntry; // would be better as a static in the processing function
enum voltageZones voltageZoneOfLastSampleV; // would be better as a static in the processing function
enum voltageZones lastPeakZoneVisited; // would be better as a static in the processing function

boolean laststate = true;
unsigned long lastsendtime = 0;

void setup()
{  
  msmts = &mqueue;
  msmt = mqFirst( msmts);

  delay(20);
  
  Timer1.initialize( 1000000);
  Timer1.attachInterrupt( timeTick);

  Serial.begin(115200);                                      // initialize Serial interface
  printf_begin();
  Serial.println("emonTx");
  Serial.print( "int-kwh");
  Serial.print( "-vphase");
  Serial.print( "-dbg");
  Serial.print( "-q32");
  Serial.println( "A.");
  
  radio.begin();                           // Setup and configure rf radio
  radio.setChannel(32);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);                     // Ensure autoACK is enabled
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries( 4, 8);                   // Optionally, increase the delay between retries & # of retries
  radio.setCRCLength(RF24_CRC_16); 
  
  radio.openWritingPipe( pipes[1]);
  radio.openReadingPipe( 1, pipes[0]);
  
  radio.startListening();                 // Start listening
  radio.stopListening();

  Serial.println("----------------------------------");
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  Serial.println("----------------------------------");
  
  radio.powerUp(); 
  
  // Setup indicator LED
  HEARTBEAT_DDR &= ~(1<< HEARTBEAT_LED);
  HEARTBEAT_PORT &= ~(1<< HEARTBEAT_LED);
       
  // When using integer maths, calibration values that have supplied in floating point 
  // form need to be rescaled.  
  //
  phaseCal_int_CT1 = phaseCal_CT1 * 256; // for integer maths
  phaseCal_int_CT2 = phaseCal_CT2 * 256; // for integer maths
  phaseCal_int_CT3 = phaseCal_CT3 * 256; // for integer maths
  phaseCal_int_CT4 = phaseCal_CT4 * 256; // for integer maths
    
  // Define operating limits for the LP filters which identify DC offset in the voltage 
  // sample streams.  By limiting the output range, these filters always should start up 
  // correctly.
  DCoffset_V_long = 512L * 256; // nominal mid-point value of ADC @ x256 scale
  DCoffsetV_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
  DCoffsetV_max = (long)(512L + 100) * 256; // mid-point of ADC minus a working margin
  
  Serial.println ("ADC mode:       free-running");
  
  // Set up the ADC to be free-running 
  ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                 // Enable the ADC 
  
  ADCSRA |= (1<<ADATE);  // set the Auto Trigger Enable bit in the ADCSRA register.  Because 
                         // bits ADTS0-2 have not been set (i.e. they are all zero), the 
                         // ADC's trigger source is set to "free running mode".
                         
  ADCSRA |=(1<<ADIE);    // set the ADC interrupt enable bit. When this bit is written 
                         // to one and the I-bit in SREG is set, the 
                         // ADC Conversion Complete Interrupt is activated. 

  ADCSRA |= (1<<ADSC);   // start ADC manually first time 
  sei();                 // Enable Global Interrupts  
        
  Serial.print ( "CT1 power: "); Serial.print(powerCal_CT1,4);
  Serial.print ( "  phase: "); Serial.println(phaseCal_CT1);
  Serial.print ( "CT2 power: "); Serial.print(powerCal_CT2,4);
  Serial.print ( "  phase: "); Serial.println(phaseCal_CT2);
  Serial.print ( "CT3 power: "); Serial.print(powerCal_CT3,4);
  Serial.print ( "  phase: "); Serial.println(phaseCal_CT3);
  Serial.print ( "CT4 power: "); Serial.print(powerCal_CT4,4);
  Serial.print ( "  phase: "); Serial.println (phaseCal_CT4);
  
  Serial.println ("----");    

  voltageThresholdUpper_long = (long)voltageThresholdOffset << 8;
  voltageThresholdLower_long = -1 * voltageThresholdUpper_long;
  Serial.println("voltage thresholds:");
  Serial.print("  upper: ");
  Serial.print(voltageThresholdUpper_long >> 8);
  Serial.print("  lower: ");
  Serial.println(voltageThresholdLower_long >> 8);

  Serial.print("phaseDelta [ ");
  Serial.print( 0);
  Serial.print(" : ");
  Serial.print( phaseOff2);
  Serial.print(" : ");
  Serial.print( phaseOff3);
  Serial.print(" : ");
  Serial.print( phaseOff4);
  Serial.print(" #");
  Serial.print( samplesPerPhase);
  Serial.println( "]");

  Serial.println("----------------------------------");
  
  // Read in last stored values
//  eeprom_read_block((void*)&oem, (void*)0, sizeof(oem));

  eepromSlot = findLatestEESlot();

  Serial.println( "Restoring from EEprom ...");

  if ( eepromSlot >= 0) {
    memcpy_from_eeprom_with_checksum( (char*) &oem.energy, ee_addresses[ eepromSlot], sizeof( oem.energy));
    
    oem.energy.timestamp += oem.energy.duration +1;
    oem.energy.duration = 0;
    
    Serial.print( "  slot[");
    Serial.print( eepromSlot);
    Serial.println( "]");
  } else {
    Serial.println( "  no valid slot! Resetting ...");
    oem.energy.timestamp = 0;
    oem.energy.duration = 0;
    oem.energy.wh_CT1 = 0;
    oem.energy.wh_CT2 = 0;
    oem.energy.wh_CT3 = 0;
    oem.energy.wh_CT4 = 0;
  }
  
  eepromSlot = ++eepromSlot % 4;
  Serial.println("----------------------------------");
}

int findLatestEESlot() {
  byte maxSlot = -1;
  oem_energy temp;
  long maxTs = 0, ts;
  
  Serial.println( "findSlot [ ");
  for( byte i = 0; i < 4; i++) {
    Serial.print( "#");
    Serial.print( i);
    Serial.print( " [");
    if ( memcpy_from_eeprom_with_checksum( (char*) &temp, ee_addresses[ i], sizeof( oem.energy))) {
      ts = temp.timestamp + temp.duration;
      Serial.print( temp.timestamp);
      Serial.print( ":");
      Serial.print( temp.duration);
      Serial.print( "][");
      Serial.print( temp.wh_CT1);
      Serial.print( "-");
      Serial.print( temp.wh_CT2);
      Serial.print( "-");
      Serial.print( temp.wh_CT3);
      Serial.print( "-");
      Serial.print( temp.wh_CT4);
      Serial.print( "] ");
      if ( ts > maxTs) {
        Serial.print( "mx ");
        maxTs = ts;
        maxSlot = i;
      }
    } else Serial.print( "-- ");
    Serial.println();
  }
  Serial.print( " max[ ");
  Serial.print( maxSlot);
  Serial.println( "]");
  
  return maxSlot;
}

/*
void dumpQueue() {
  memcpy( &mtemp, &mqueue, sizeof( mqueue));
  
  Serial.print( "queue [ ");
  Serial.print( mtemp.head);
  Serial.print( " : ");
  Serial.print( mtemp.tail);
  Serial.println( "] [ ");
  for( byte i = 0; i < QUEUE_LENGTH; i++) {
    Serial.print( "#");
    Serial.print( i);
    Serial.print( " [");
    Serial.print( mtemp.queue[i].voltage-512);
    Serial.print( " V  ");
    Serial.print( mtemp.queue[i].currP1 ? mtemp.queue[i].currP1-512 : 0);
    Serial.print( " : ");
    Serial.print( mtemp.queue[i].currP2 ? mtemp.queue[i].currP2-512 : 0);
    Serial.print( " : ");
    Serial.print( mtemp.queue[i].currP3 ? mtemp.queue[i].currP3-512 : 0);
    Serial.print( " : ");
    Serial.print( mtemp.queue[i].currP4 ? mtemp.queue[i].currP4-512 : 0);
    Serial.print( "] ");
    Serial.println();
  }
  Serial.println( "]");
}
*/
// promote the device clock
void timeTick() {
  oem.energy.duration++;
  oem.power.timestamp++;
}

// An Interrupt Service Routine is now defined in which the ADC is instructed to perform 
// a conversion of the voltage signal and each of the signals for current.  A "data ready" 
// flag is set after each voltage conversion has been completed, it being the last one in
// the sequence.  
//   Samples for current are taken first because the phase of the waveform for current is 
// generally slightly advanced relative to the waveform for voltage.  The data ready flag 
// is cleared within loop().

// This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished, approx every 104 us.  In 
// free-running mode, the ADC has already started its next conversion by the time that
// the ISR is executed.  The ISR therefore needs to "look ahead". 
//   At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR 
// which runs at this point therefore needs to capture the results of conversion Type N , 
// and set up the conditions for conversion Type N+2, and so on.  
// 
ISR(ADC_vect)  
{                                         
  static unsigned char sample_index = 0;
  
  switch(sample_index)
  {
    case 0:
      msmt->voltage = ADC;
      ADMUX = 0x40 + currentSensor_CT2; // set up the next-but-one conversion
      sample_index++; // advance the control flag             
      msmt = mqEnqueue( msmts); 
      break;
    case 1:
      msmt->currP1 = ADC;
      ADMUX = 0x40 + currentSensor_CT3; // for the next-but-one conversion
      sample_index++; // advance the control flag                
      break;
    case 2:
      msmt->currP2 = ADC;
      ADMUX = 0x40 + currentSensor_CT4; // for the next-but-one conversion
      sample_index++; // advance the control flag                
      break;
    case 3:
      msmt->currP3 = ADC;
      ADMUX = 0x40 + voltageSensor; // for the next-but-one conversion
      sample_index++; // advance the control flag                 
      break;
    case 4:
      msmt->currP4 = ADC;
      ADMUX = 0x40 + currentSensor_CT1; // for the next-but-one conversion
      sample_index = 0; // reset the control flag                
      break;
    default:
      sample_index = 0;                 // to prevent lockup (should never get here)      
  }}


// When using interrupt-based logic, the main processor waits in loop() until the 
// dataReady flag has been set by the ADC.  Once this flag has been set, the main
// processor clears the flag and proceeds with all the processing for one pair of 
// V & I samples.  It then returns to loop() to wait for the next pair to become 
// available.
//   If the next pair of samples become available before the processing of the 
// previous pair has been completed, data could be lost.  This situation can be 
// avoided by prior use of the WORKLOAD_CHECK mode.  Using this facility, the amount
// of spare processing capacity per loop can be determined.  
//
void loop()             
{ 
  bool packet_ready = false;
  byte pCount = 0;
  
  mQueueEntry* mq;
  while ( mq = mqLast( msmts)) {
    packet_ready |= process( mq); // executed once for each pair of V&I samples
    pCount++;
  }

  if ( packet_ready && (millis()-lastsendtime)>sendinterval)
  {
    lastsendtime = millis();
    HEARTBEAT_INP |= (1<< HEARTBEAT_LED);

    send_packet( OEM_ENERGY, (char*) &(oem.energy), sizeof( oem_energy));
    handleRx();
    
    if ( oem.energy.duration % 5) {
      send_packet( OEM_POWER, (char*) &(oem.power), sizeof( oem_power));
      handleRx();
    }
    
    HEARTBEAT_INP |= (1<< HEARTBEAT_LED);
  }
} // end of loop()

void send_packet( byte packetid, char* packet, byte psize) {
  static oem_packet buffer;
  buffer.packet_type = packetid;
  memcpy( buffer.data, packet, psize);
  
  bool ok = radio.writeFast( &buffer, psize+1);
//  bool ok_tx = radio.txStandBy( 20);
//  radio.startListening();
}

uint8_t handleRx() {
  static oem_packet rx_buffer;
  uint8_t psz = 0;
  if ( radio.available() || radio.isAckPayloadAvailable()){       
    psz = radio.getDynamicPayloadSize();
    radio.read( &rx_buffer, psz);
    
    switch( rx_buffer.packet_type) {
      case OEM_TIMESTAMP:
        if ( rx_buffer.timestamp.timestamp > DEFAULT_TIME) {
          setTime( rx_buffer.timestamp.timestamp);
          oem.ts.timestamp = rx_buffer.timestamp.timestamp;
/*          
          oem.power.timestamp = rx_buffer.timestamp.timestamp;
          oem.energy.timestamp = rx_buffer.timestamp.timestamp;
          oem.energy.duration = 0;
*/
          Serial.print( "rx time[");
          Serial.print(hour());
          printDigits(minute());
          printDigits(second());
          Serial.print(" ");
          Serial.print(day());
          Serial.print(" ");
          Serial.print(month());
          Serial.print(" ");
          Serial.print(year()); 
          Serial.println( "]");
        }
      break;
      
      default:
        Serial.print( "rx[");
        Serial.print( rx_buffer.packet_type);
        Serial.print( " #");
        Serial.print( psz);
        Serial.println( "]");
    }
  }
    
  return psz;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

bool saveToEEprom() {
  static unsigned long lastTimestamp; 

  if ( oem.ts.timestamp > lastTimestamp + 10) {
    memcpy_to_eeprom_with_checksum( ee_addresses[ eepromSlot], (char*) &oem.energy, sizeof( oem.energy));
    lastTimestamp = oem.ts.timestamp;
    Serial.print( "EE_WRITE#");
    Serial.println( eepromSlot);
    eepromSlot = ++eepromSlot % 4;
    
    return true;
  } else {
    Serial.println( "EE_WRITE defer ...");
    return false;
  }
}
// This routine is called to process each pair of V & I samples.  Note that when using 
// interrupt-based code, it is not necessary to delay the processing of each pair of 
// samples as was done in Mk2a builds.  This is because there is no longer a strict 
// alignment between the obtaining of each sample by the ADC and the processing that can 
// be done by the main processor while the ADC conversion is in progress.  
//   When interrupts are used, the main processor and the ADC work autonomously, their
// operation being only linked via the dataReady flag.  As soon as data is made available
// by the ADC, the main processor can start to work on it immediately.  
//
bool process( mQueueEntry* mq) {
  
  static long sumP_CT1;                         
  static long sumP_CT2;                              
  static long sumP_CT3;                           
  static long sumP_CT4;                            
  static enum polarities polarityOfLastSampleV;  // for zero-crossing detection
  static long cumV_deltasThisCycle_long;    // for the LPF which determines DC offset (voltage)
  static long lastSampleV_minusDC_long;     //    for the phaseCal algorithm

  static int sequenceCount = 0;
  static int samplesDuringThisWindow_CT1;            
  static int samplesDuringThisWindow_CT2;            
  static int samplesDuringThisWindow_CT3;            
  static int samplesDuringThisWindow_CT4;
  static int samplesDuringThisWindow_Voltage;
  static int lastVoltage;
  static long vSum;
  bool packet_ok = false;

  // remove DC offset from the raw voltage sample by subtracting the accurate value 
  // as determined by a LP filter.
  long sampleV_minusDC_long = ((long)mq->voltage<<8) - DCoffset_V_long;

  // for AC failure detection 
  enum voltageZones voltageZoneNow; 
  boolean nextPeakDetected = false;
  
  switch( voltState) {
    case VOLTAGE_OK:
      if ( sampleIdx >= 40 || oem.power.voltage < voltHystUpper) {
        voltState = VOLTAGE_FAIL;
        Serial.print( "Fail");
        
        saveToEEprom();
      }
    break;
    
    case VOLTAGE_RECOVER:
      if ( sampleIdx >= 40 || oem.power.voltage < voltHystLower) {
        voltState = VOLTAGE_FAIL;
        Serial.println( "Fail");
      } else {
        if ( oem.power.voltage > voltHystUpper) {
          if ( ++voltageGoodDelay > vGoodCycles) {
            voltState = VOLTAGE_OK;
            Serial.println( "OK");
          } else {
            Serial.print( "-");
          }
        } else voltageGoodDelay = 0;
      } 
    break;
    
    case VOLTAGE_FAIL:
      if ( sampleIdx >= 40 || oem.power.voltage < voltHystLower) {
        voltState = VOLTAGE_ERROR;
        Serial.println( "Error");
      } else {
        if ( oem.power.voltage > voltHystUpper) {
          voltState = VOLTAGE_RECOVER;
          voltageGoodDelay = 0;
          Serial.println( "RECOVER");
        }
      }
    break;
    
    case VOLTAGE_ERROR:
    default:
      if ( sampleIdx < 40 && oem.power.voltage > voltHystLower) {
        voltState = VOLTAGE_RECOVER;
        Serial.println( "RECOVER");
      }
  }
    
/*
  if ( sequenceCount == 0) {
        Serial.print("vpeak min[ ");
        Serial.print( voltageThresholdLower_long / 256.0);
        Serial.print(" ]  cur[ ");
        Serial.print( sampleV_minusDC_long / 256.0);
        Serial.print(" ]  max[ ");
        Serial.print( voltageThresholdUpper_long / 256.0);
        Serial.println(" ]");        
  }
*/  

  // determine polarity, to aid the logical flow
  enum polarities polarityNow;   
  if(sampleV_minusDC_long > 0) { 
    polarityNow = POSITIVE; }
  else { 
    polarityNow = NEGATIVE; }

  if (polarityNow == POSITIVE) 
  {                           
    if (beyondStartUpPhase)
    {  
      if (polarityOfLastSampleV != POSITIVE)
      {
        // This is the start of a new +ve half cycle (just after the zero-crossing point)
        sequenceCount++;
        long realPower_long;
        long v1;
//        byte i;

        // calculate the offset of the actual zero crossing
        // lastSample was negative, current sample is positive
        // 0 = ylast + zp * (ynow - ylast)
        // -ylast / (ynow - ylast) = zp
        // -28 / ( 32 - -28) = -28 / 60 = -0.5
        phaseCal_voltage_int = (lastSampleV_minusDC_long <<4) / (( sampleV_minusDC_long - lastSampleV_minusDC_long)>>4);
/*
        Serial.print("vphase[ ");
        Serial.print( phaseCal_voltage_int / 256.0);
        Serial.print(" ]  last[ ");
        Serial.print( lastSampleV_minusDC_long / 256.0);
        Serial.print(" ]  now[ ");
        Serial.print( sampleV_minusDC_long / 256.0);
        Serial.println(" ]");
*/
        samplesPerPhase = sampleIdx;       
        sampleIdx = 0;
        
        oem.power.voltage = powerVRatio * sqrt( vSum / samplesDuringThisWindow_Voltage);        
        vSum = 0;
        
        switch(sequenceCount)
        {
          // 100 cycles = 2 seconds
          case 20: 
            realPower_long = sumP_CT1 / samplesDuringThisWindow_CT1; 
            oem.power.realPower_CT1 = realPower_long * powerCal_CT1;
            
            joules_CT1 += oem.power.realPower_CT1 * 2;  // Joules elapsed in 100 cycles @ 50Hz is power J.s x 2 seconds
            
            oem.energy.wh_CT1 += joules_CT1 / 3600;
            joules_CT1 = joules_CT1 % 3600;
/*
            Serial.print("vphase[ ");
            Serial.print( phaseCal_voltage_int / 256.0);
            Serial.print(" ]  last[ ");
            Serial.print( lastSampleV_minusDC_long / 256.0);
            Serial.print(" ][");
            for( byte i = 0; i < samplesPerPhase; i++) {
              Serial.print(" ");
              Serial.print( vSamples[ i] >> 8);
            }
            Serial.println("] ");
*/
/*
            Serial.print("[ ");
            Serial.print( ( samplesPerPhase + sampleIdx) % samplesPerPhase);
            Serial.print("-");
            Serial.print( ( samplesPerPhase + sampleIdx - 1) % samplesPerPhase);
            Serial.print(" ");
            Serial.print( (samplesPerPhase + sampleIdx - phaseOff2) % samplesPerPhase);
            Serial.print("-");
            Serial.print( (samplesPerPhase + sampleIdx - phaseOff2 - 1) % samplesPerPhase);
            Serial.print(" ");
            Serial.print( (samplesPerPhase + sampleIdx - phaseOff3) % samplesPerPhase);
            Serial.print("-");
            Serial.print( (samplesPerPhase + sampleIdx - phaseOff3 - 1) % samplesPerPhase);
            Serial.print(" ");
            Serial.print( (samplesPerPhase + sampleIdx - phaseOff4) % samplesPerPhase);
            Serial.print("-");
            Serial.print( (samplesPerPhase + sampleIdx - phaseOff4 - 1) % samplesPerPhase);
            Serial.println("] ");
*/            
            Serial.print("#[");
            Serial.print( oem.power.timestamp);
            Serial.print("][");
            Serial.print( oem.energy.timestamp+oem.energy.duration);
            Serial.print("] ");

            Serial.print("Hz[");           
            Serial.print( samplesDuringThisWindow_Voltage);
//            Serial.print("] spp[");
//            Serial.print( samplesPerPhase);            
            Serial.print("] Vrms[");
            Serial.print( oem.power.voltage);            
            Serial.print("V] ");

            sumP_CT1 = 0;
            samplesDuringThisWindow_CT1 = 0;

            Serial.print("P_1[");
            Serial.print(oem.power.realPower_CT1);
            Serial.print(':');
            Serial.print(oem.energy.wh_CT1);
            Serial.print("] ");
            break;
          case 40: 
            realPower_long = sumP_CT2 / samplesDuringThisWindow_CT2; 
            oem.power.realPower_CT2 = realPower_long * powerCal_CT2;
            
            joules_CT2 += oem.power.realPower_CT2 * 2;  // Joules elapsed in 100 cycles @ 50Hz is power J.s x 2 seconds
            
            oem.energy.wh_CT2 += joules_CT2 / 3600;
            joules_CT2 = joules_CT2 % 3600;
            
            sumP_CT2 = 0;
            samplesDuringThisWindow_CT2 = 0;
            Serial.print("P_2[");
            Serial.print(oem.power.realPower_CT2); 
            Serial.print(':');
            Serial.print(oem.energy.wh_CT2);          
            Serial.print("] ");
            break;
          case 60: 
            realPower_long = sumP_CT3 / samplesDuringThisWindow_CT3; 
            oem.power.realPower_CT3 = realPower_long * powerCal_CT3;
            
            joules_CT3 += oem.power.realPower_CT3 * 2;  // Joules elapsed in 100 cycles @ 50Hz is power J.s x 2 seconds
            
            oem.energy.wh_CT3 += joules_CT3 / 3600;
            joules_CT3 = joules_CT3 % 3600;
            
            sumP_CT3 = 0;
            samplesDuringThisWindow_CT3 = 0;
            Serial.print("P_3[");
            Serial.print(oem.power.realPower_CT3);
            Serial.print(':');
            Serial.print(oem.energy.wh_CT3);           
            Serial.print("] ");
           break;
          case 80: 
            realPower_long = sumP_CT4 / samplesDuringThisWindow_CT4; 
            oem.power.realPower_CT4 = realPower_long * powerCal_CT4;
            
            joules_CT4 += oem.power.realPower_CT4 * 2;  // Joules elapsed in 100 cycles @ 50Hz is power J.s x 2 seconds
            
            oem.energy.wh_CT4 += joules_CT4 / 3600;
            joules_CT4 = joules_CT4 % 3600;
            
            sumP_CT4 = 0;
            samplesDuringThisWindow_CT4 = 0;
            Serial.print("P_A[");
            Serial.print(oem.power.realPower_CT4); 
            Serial.print(':');
            Serial.print(oem.energy.wh_CT4);          
            Serial.print("] ");
           break;
          default: 
            if (sequenceCount >= 100) {
              Serial.println(); 
              sequenceCount = 0;
              
              oem.ts.timestamp = now();
//              oem.power.timestamp = oem.ts.timestamp;

//              dumpQueue();
              
              packet_ok = true;
            }         
        }
        
        samplesDuringThisWindow_Voltage = 0;
          
        // Calculate the average real power during the measurement period.
        //
        // sumP contains the sum of many individual calculations of instantaneous power.  In  
        // order to obtain the average power during the relevant period, sumP must be divided 
        // by the number of samples that have contributed to its value.
        //
        
        // The next stage is to apply a calibration factor so that real power can be expressed 
        // in Watts.  That's fine for floating point maths, but it's not such
        // a good idea when integer maths is being used.  To keep the numbers large, and also 
        // to save time, calibration of power is omitted at this stage.  realPower_long is 
        // therefore (1/powerCal) times larger than the actual power in Watts.
        //
        long realPower_long_CT1 = sumP_CT1 / samplesDuringThisWindow_CT1; 
        long realPower_long_CT2 = sumP_CT2 / samplesDuringThisWindow_CT2; 
        long realPower_long_CT3 = sumP_CT3 / samplesDuringThisWindow_CT3; 
        long realPower_long_CT4 = sumP_CT4 / samplesDuringThisWindow_CT4; 
   
 
      } // end of processing that is specific to the first Vsample in each +ve half cycle   
    }
    else
    {  
      // wait until the DC-blocking filters have had time to settle
      if(millis() > startUpPeriod * 1000) 
      {
        beyondStartUpPhase = true;
        Serial.print ("Go! ");
        Serial.print("spp[");
        Serial.print( samplesPerPhase);            
        Serial.println("]");
      }
    }
  } // end of processing that is specific to samples where the voltage is positive
  
  else // the polarity of this sample is negative
  {     
    if (polarityOfLastSampleV != NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      //
      // This is a convenient point to update the twin Low Pass Filters for DC-offset removal,
      // one on each voltage channel.  This needs to be done right from the start.
      long previousOffset; 
      
      previousOffset = DCoffset_V_long; // for voltage source V
      DCoffset_V_long = previousOffset + (cumV_deltasThisCycle_long>>6); // faster than * 0.01
      cumV_deltasThisCycle_long = 0;
      
      // To ensure that each of these LP filters will always start up correctly when 240V AC is 
      // available, its output value needs to be prevented from drifting beyond the likely range 
      // of the voltage signal.  This avoids the need to include a HPF as is often used for 
      // sketches of this type.
      //
      if (DCoffset_V_long < DCoffsetV_min) {  // for voltage source V
        DCoffset_V_long = DCoffsetV_min; }
      else  
      if (DCoffset_V_long > DCoffsetV_max) {
        DCoffset_V_long = DCoffsetV_max; }
                
    } // end of processing that is specific to the first Vsample in each -ve half cycle
  } // end of processing that is specific to samples where the voltage is positive
  
  // Processing for EVERY pair of samples. Most of this code is not used during the 
  // start-up period, but it does no harm to leave it in place.  Accumulated values 
  // are cleared when beyondStartUpPhase is set to true.
  //
  // remove most of the DC offset from the current sample (the precise value does not matter)
  long sampleIminusDC_long_CT1 = ((long)(msmt->currP1 - DCoffset_I))<<8;
  long sampleIminusDC_long_CT2 = ((long)(msmt->currP2 - DCoffset_I))<<8;
  long sampleIminusDC_long_CT3 = ((long)(msmt->currP3 - DCoffset_I))<<8;
  long sampleIminusDC_long_CT4 = ((long)(msmt->currP4 - DCoffset_I))<<8;

  vSum += ( sampleV_minusDC_long>>8) * (sampleV_minusDC_long>>8);    
  samplesDuringThisWindow_Voltage++;
  
  if ( sampleIdx < 40) vSamples[ sampleIdx] = sampleV_minusDC_long;
  byte sampleIndex = samplesPerPhase + sampleIdx;
  
  if ( sampleIdx < 40) sampleIdx++;

//  sampleV_minusDC_long_p2 = vSamples[ (sampleIdx - phaseOff2) % samplesPerPhase];
//  sampleV_minusDC_long_p2_last = vSamples[ (sampleIdx - phaseOff2 - 1) % samplesPerPhase];

  // phase-shift the voltage waveform so that it aligns with the current when a 
  // resistive load is used
  long  phaseShiftedSampleV_minusDC_long_CT1 = lastSampleV_minusDC_long
         + (((sampleV_minusDC_long - lastSampleV_minusDC_long)*phaseCal_int_CT1)>>8);  
         
  long  p2_V = vSamples[ (sampleIndex - phaseOff2) % samplesPerPhase];
  long  p2_V_last = vSamples[ (sampleIndex - phaseOff2 - 1) % samplesPerPhase];
  long  phaseShiftedSampleV_minusDC_long_CT2 = p2_V_last + (((p2_V - p2_V_last)*phaseCal_int_CT2)>>8);
         
  long  p3_V = vSamples[ (sampleIndex - phaseOff3) % samplesPerPhase];
  long  p3_V_last = vSamples[ (sampleIndex - phaseOff3 - 1) % samplesPerPhase];
  long  phaseShiftedSampleV_minusDC_long_CT3 = p3_V_last + (((p3_V - p3_V_last)*phaseCal_int_CT3)>>8);  
         
  long  p4_V = vSamples[ (sampleIndex - phaseOff4) % samplesPerPhase];
  long  p4_V_last = vSamples[ (sampleIndex - phaseOff4 - 1) % samplesPerPhase];
  long  phaseShiftedSampleV_minusDC_long_CT4 = p4_V_last + (((p4_V - p4_V_last)*phaseCal_int_CT4)>>8);
  
  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4_CT1 = phaseShiftedSampleV_minusDC_long_CT1>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT1 = sampleIminusDC_long_CT1>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT1 = filtV_div4_CT1 * filtI_div4_CT1;  // 32-bits (now x4096, or 2^12)
  instP_CT1 = instP_CT1>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)  

  if ( msmt->currP1==0) instP_CT1 = 0;  
  sumP_CT1 +=instP_CT1; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  long filtV_div4_CT2 = phaseShiftedSampleV_minusDC_long_CT2>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT2 = sampleIminusDC_long_CT2>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT2 = filtV_div4_CT2 * filtI_div4_CT2;  // 32-bits (now x4096, or 2^12)
  instP_CT2 = instP_CT2>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)

  if ( msmt->currP2==0) instP_CT2 = 0;  
  sumP_CT2 +=instP_CT2; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  long filtV_div4_CT3 = phaseShiftedSampleV_minusDC_long_CT3>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT3 = sampleIminusDC_long_CT3>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT3 = filtV_div4_CT3 * filtI_div4_CT3;  // 32-bits (now x4096, or 2^12)
  instP_CT3 = instP_CT3>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)     

  if ( msmt->currP3==0) instP_CT3 = 0;  
  sumP_CT3 +=instP_CT3; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  long filtV_div4_CT4 = phaseShiftedSampleV_minusDC_long_CT4>>2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4_CT4 = sampleIminusDC_long_CT4>>2; // reduce to 16-bits (now x64, or 2^6)
  long instP_CT4 = filtV_div4_CT4 * filtI_div4_CT4;  // 32-bits (now x4096, or 2^12)
  instP_CT4 = instP_CT4>>12;     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)  

  if ( msmt->currP4==0) instP_CT4 = 0;  
  sumP_CT4 +=instP_CT4; // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  
  samplesDuringThisWindow_CT1++;
  samplesDuringThisWindow_CT2++;
  samplesDuringThisWindow_CT3++;
  samplesDuringThisWindow_CT4++;
  
  // store items for use during next loop
  cumV_deltasThisCycle_long += sampleV_minusDC_long; // for use with LP filter
  lastSampleV_minusDC_long = sampleV_minusDC_long;  // required for phaseCal algorithm
  
  polarityOfLastSampleV = polarityNow;  // for identification of half cycle boundaries
  voltageZoneOfLastSampleV = voltageZoneNow; // for voltage failure detection
  
  return packet_ok;
}
// end of allGeneralProcessing()


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_write_byte( (uint8_t *) destination++, *(source++)); 
  }
  eeprom_write_byte( (uint8_t *) destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_read_byte( (uint8_t *) source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return (checksum == eeprom_read_byte( (uint8_t *) source));
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

