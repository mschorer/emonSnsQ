enum packetTypes { OEM_ENERGY, OEM_POWER, OEM_TIMESTAMP, OEM_NOP};

typedef struct { 
  unsigned long timestamp; 
  unsigned long duration; 
  long wh_CT1;
  long wh_CT2;
  long wh_CT3;
  long wh_CT4;
} oem_energy;    // revised data for RF comms

typedef struct { 
  unsigned long timestamp; 
  int realPower_CT1;
  int realPower_CT2;
  int realPower_CT3;
  int realPower_CT4; 
  unsigned int voltage;
  unsigned int padding; 
} oem_power;    // revised data for RF comms

typedef struct { 
  unsigned long timestamp; 
} oem_timestamp;    // revised data for RF comms

// - All Atmega's shipped from OpenEnergyMonitor come with Arduino Uno bootloader

typedef struct {
  oem_timestamp ts;
  oem_power power;
  oem_energy energy;
} sStatus;    // revised data for RF comms

typedef struct {
  byte packet_type;
  union {
    byte data[];
    oem_energy energy;
    oem_power power;
    oem_timestamp timestamp;
  };
} oem_packet;


