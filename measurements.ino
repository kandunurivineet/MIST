#include <SPI.h> 


//--------------------------------------CS5463 Register Addresses--------------------------------------------- 
  //  B7  B6  B5  B4  B3  B2  B1  B0
  //  0 W/R RA4 RA3 RA2 RA1 RA0 0
  //  W=1 R=0
enum CS5463_register_t {
  
  //Register Page 0
  CONFIG              =    0,     // Configuration
  CURRENT_DC_OFFSET       =    1,     // Current DC Offset
  CURRENT_GAIN          =    2,     // Current Gain
  VOLTAGE_DC_OFFSET       =    3,     // Voltage DC Offset
  VOLTAGE_GAIN          =    4,     // Voltage Gain
  CYCLE_COUNT           =    5,     // Number of A/D conversions used in one computation cycle (N)).
  PULSE_RATE_E          =    6,     // Sets the E1, E2 and E3 energy-to-frequency output pulse rate.
  CURRENT             =  7,     // Instantaneous Current
  VOLTAGE             =    8,     // Instantaneous Voltage
  POWER             =  9,     // Instantaneous Power
  POWER_ACTIVE          =    10,  // Active (Real) Power
  CURRENT_RMS           =    11,  // RMS Current
  VOLTAGE_RMS           =    12,  // RMS Voltage
  EPSILON             =  13,  // Ratio of line frequency to output word rate (OWR)
  POWER_OFFSET          =    14,  // Power Offset
  STATUS              =    15,  // Status
  CURRENT_AC_OFFSET       =    16,  // Current AC (RMS) Offset
  VOLTAGE_AC_OFFSET       =    17,  // Voltage AC (RMS) Offset
  MODE              =    18,  // Operation Mode
  TEMPERATURE           =    19,  // Temperature
  POWER_REACTIVE_AVERAGE      =    20,  // Average Reactive Power
  POWER_REACTIVE          =    21,  // Instantaneous Reactive Power
  CURRENT_PEAK          =    22,  // Peak Current
  VOLTAGE_PEAK          =    23,  // Peak Voltage
  POWER_REACTIVE_TRIANGLE     =    24,  // Reactive Power calculated from Power Triangle
  POWERFACTOR           =    25,  // Power Factor
  MASK_INTERUPT         =    26,  // Interrupt Mask
  POWER_APPARENT          =    27,  // Apparent Power
  CONTROL             =    28,  // Control
  POWER_ACTIVE_HARMONIC     =    29,  // Harmonic Active Power
  POWER_ACTIVE_FUNDAMENTAL    =    30,  // Fundamental Active Power
  POWER_REACTIVE_FUNDAMENTAL    =    31,  // Fundamental Reactive Power / Page
  
    
  //Register Page 1
  PULSE_WIDTH           = 0,
  LOAD_MIN            = 1,
  TEMPERATURE_GAIN        = 2,
  TEMPERATURE_OFFSET        = 3,
  
  //Register Page 3
  VOLTAGE_SAG_DURATION      = 6,
  VOLTAGE_SAG_LEVEL       = 7,
  CURRENT_SAG_DURATION      = 10,
  CURRENT_SAG_LEVEL       = 11,

  // COMMANDS
  Read              =  0,
  Write             =  64,
  SYNC0             =  254, //SYNC 0 Command: Last byte of a serial port re-initialization sequence
  SYNC1             =  255, //SYNC 1 Command: Used during reads and serial port initialization.
  SELECT_PAGE           =  0x1F,
  START_CONTINOUS         =  0xE8,
  VOLTAGE_DC_OFFSET_CALIBRATION   =   0b11011101,
 ACG_CAL=0XDE,
 ACOFF_CAL=0XDD,
  
}
;

//--------------------------------------Variables--------------------------------------------- 
const int CS = 12;      // Assign the Chip Select signal to pin 8
const int RESET_PIN = 13; // Assign Reset to pin 13
const int MODE_PIN = 11;  // Assign Mode to pin 9
const int RELAY_SW = 10;    // Assign RELAY to pin 9

//Bytes after register read
byte H_Byte;             //High Byte 
byte M_Byte;              //Middle Byte 
byte L_Byte;              //Low Byte

//DEFAULTS Voltage Gain and offset
//byte vgh=0x40;byte vgm=0x00;byte vgl=0x00;byte voffh=0x00;byte voffm=0x00;byte voffl=0x00;byte ioffh=0x00;byte ioffm=0x00;byte ioffl=0x00;byte igh=0x40;byte igm=0x00;byte igl=0x00;
//VOLTAGE GAIN
byte vgh=0x8E;
byte vgm=0x38;
byte vgl=0xE3;
//VOLTAGE OFFSET
byte voffh=0x00;
byte voffm=0x10;
byte voffl=0x00;
//CURRENT OFFSET
byte ioffh=0x00;
byte ioffm=0x06;
byte ioffl=0xE6;
//CURRENT GAIN
byte igh=0x40;
byte igm=0x00;
byte igl=0x00;

int time_delay=2000;
float Full_Scale_V =250   ;///197.5;//237.71;  //full scale Value
float Full_Scale_I =237.71;   //full scale Value


void setup() {
  // put your setup code here, to run once:
  pinMode(CS, OUTPUT);      //initalize the chip select pin;
  pinMode(RESET_PIN, OUTPUT); //initalize the RESET pin;
  pinMode(MODE_PIN, OUTPUT);  //initialize the MODE pin;
  pinMode(RELAY_SW, OUTPUT);    //initialize the MODE pin;
  digitalWrite(CS, HIGH);
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(MODE_PIN, LOW);  
  digitalWrite(RELAY_SW, HIGH);   //swich power on and off
  delay (100);
 //Create a serial connection to display the data on the terminal.   
  Serial.begin(9600);
  //start the SPI library;
  SPI.begin();
  SPI.beginTransaction(SPISettings(100000,MSBFIRST,SPI_MODE0)); 
 hardware_reset();
 software_reset();    
//--------------------------------------WRITE REGISTER--------------------------------------------- 
 set_config_reg();
 set_mask_reg();
 set_mode_reg();
 set_ctrl_reg();
 set_status_reg();
 reset_offset_gain();
 cal();
set_offset_gain();
 cal();
 start_conversions();  
}

void loop() {
  
  // put your main code here, to run repeatedly:
  //vgain();
  //igain();
  //voff();
  //ioff();
  vrms();
  irms();
  Serial.println("\n\n\n");
  //Serial.println(x,2);
  delay(1100);
}

void irms()
{
  digitalWrite(CS, LOW);
  SPI.transfer(Read | (CURRENT_RMS<<1));
  H_Byte=SPI.transfer(SYNC1);
   M_Byte=SPI.transfer(SYNC1);
   L_Byte=SPI.transfer(SYNC1);
    digitalWrite(CS, HIGH);
   Serial.print("\nH_Byte:");
Serial.println(H_Byte,16);
Serial.print("M_Byte:");
Serial.println(M_Byte,16);
Serial.print("L_Byte:");
Serial.println(L_Byte,16);
    Serial.print("IRMS : ");
    double v;
    v=H_Byte/256.0000;
    v+=M_Byte/65536.0000;
     v=v*1000;
    Serial.println(v);
    Serial.println((0.0158*v)+0.0208);
}

void vrms()
{
  digitalWrite(CS, LOW);
  SPI.transfer(Read | (VOLTAGE_RMS<<1));
  H_Byte=SPI.transfer(SYNC1);
   M_Byte=SPI.transfer(SYNC1);
   L_Byte=SPI.transfer(SYNC1);
    digitalWrite(CS, HIGH);
   Serial.print("VRMS-H_Byte:");
Serial.println(H_Byte,16);
Serial.print("VRMS-M_Byte:");
Serial.println(M_Byte,16);
Serial.print("VRMS-L_Byte:");
Serial.println(L_Byte,16);
    Serial.print("VRMS : ");
    double v;
    v=H_Byte/256.0000;
    v+=M_Byte/65536.0000;
    //v=v*1000;
    Serial.println(v);
    Serial.println((188.043*v)+0.89);
    //Full_Scale_V;                                                                                                                                                                                                                                                                                                                                                                               Serial.println(0.2366*v-22.20834);//*Full_Scale_V
}

double vins()
{
  double coff;
  int8_t sign=1;
  digitalWrite(CS, LOW);
  SPI.transfer(Read | (VOLTAGE<<1));
  H_Byte=SPI.transfer(SYNC1);
   M_Byte=SPI.transfer(SYNC1);
   L_Byte=SPI.transfer(SYNC1);
    digitalWrite(CS, HIGH);
//Serial.print("H_Byte:");
//Serial.println(H_Byte);
//Serial.print("M_Byte:");
//Serial.println(M_Byte);
//Serial.print("L_Byte:");
//Serial.println(L_Byte);
 
    if(H_Byte&128 == 128){
    sign=-1;//H_Byte/128.00;
  }
  H_Byte=H_Byte&127;
    coff=H_Byte/128.00;
  coff+=M_Byte/32768.00;
  Serial.print("Vins:");
  Serial.println(coff*sign*Full_Scale_V); //*Full_Scale_V
return coff*sign;//*Full_Scale_V;
}

void vgain()
{
   double vg;
  digitalWrite(CS, LOW);
  SPI.transfer(Read | (VOLTAGE_GAIN<<1));
  H_Byte=SPI.transfer(SYNC1);
   M_Byte=SPI.transfer(SYNC1);
   L_Byte=SPI.transfer(SYNC1);
    digitalWrite(CS, HIGH);
//   Serial.print("H_Byte:");
//Serial.println(H_Byte);
//Serial.print("M_Byte:");
//Serial.println(M_Byte);
//Serial.print("L_Byte:");
//Serial.println(L_Byte);
vg=H_Byte/64.0000;
vg+=M_Byte/16384.0000;

  Serial.print("V GAIN:");
  Serial.println(vg); 
}
void voff()
{
  double coff;
  int8_t sign=1;
  digitalWrite(CS, LOW);
  SPI.transfer(Read | (VOLTAGE_AC_OFFSET<<1));
  H_Byte=SPI.transfer(SYNC1);
   M_Byte=SPI.transfer(SYNC1);
   L_Byte=SPI.transfer(SYNC1);
    digitalWrite(CS, HIGH);
//Serial.print("H_Byte:");
//Serial.println(H_Byte);
//Serial.print("M_Byte:");
//Serial.println(M_Byte);
//Serial.print("L_Byte:");
//Serial.println(L_Byte);
 
    if(H_Byte&128 == 128){
    sign=-1;//H_Byte/128.00;
  }
  H_Byte=H_Byte&127;
    coff=H_Byte/128.00;
  coff+=M_Byte/32768.00;
  Serial.print("V OFF:");
  Serial.println(coff*sign); 
}

void software_reset()
{
  digitalWrite(CS, LOW);
   SPI.transfer(SYNC1);    
   SPI.transfer(SYNC1);     
   SPI.transfer(SYNC1);    
   SPI.transfer(SYNC0); 
   
   digitalWrite(CS, HIGH);
  
  delay (100);
}
void hardware_reset()
 {
 //Run a Hardware Reset of the CS5463
  digitalWrite(CS, LOW);
  digitalWrite(RESET_PIN, LOW);
  delay (100);
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(CS, HIGH);
  delay (100);
}
void set_mask_reg()
{
  //Set Mask register
  digitalWrite(CS, LOW);          
  SPI.transfer( Write | (MASK_INTERUPT<<1) );
  SPI.transfer(0x00);           //3 bytes of data to set 24bits of mask register (Set for no interrupts)  
  SPI.transfer(0x00);     
  SPI.transfer(0x00); 
  digitalWrite(CS, HIGH);  
  }
void set_mode_reg()
{
    digitalWrite(CS, LOW);          
  SPI.transfer( Write | (MODE<<1) );
  SPI.transfer(0x00);           //Sets High pass filters on Voltage and Current lines, sets automatic line frequency measurements     
  SPI.transfer(0x00);     
  SPI.transfer(0x61); 
  digitalWrite(CS, HIGH); 
}

void set_config_reg()
{
  //Set Config register
  digitalWrite(CS, LOW);          //Chip select to low to initialise commands with CS5463 
  SPI.transfer( Write | (CONFIG<<1) );
  SPI.transfer(0x01);           //3 bytes of data to set 24bits of config register        
  SPI.transfer(0x00);     
  SPI.transfer(0x01);           // Set K value to 1 (clock devider for measuring cycles)
  digitalWrite(CS, HIGH);         //Chip select to HIGH to disable comms with CS5463  

}

void set_ctrl_reg()
 { //Set Control register
  digitalWrite(CS, LOW);          
  SPI.transfer( Write | (CONTROL<<1) );
  SPI.transfer(0x00);           //~Disables CPUCLK 
  SPI.transfer(0x00);     
  SPI.transfer(0x00); 
  digitalWrite(CS, HIGH);
}

void set_status_reg()
 { //Set Control register
  digitalWrite(CS, LOW);          
  SPI.transfer( Write | (STATUS<<1) );
  SPI.transfer(0x80);           //~Disables CPUCLK 
  SPI.transfer(0x00);     
  SPI.transfer(0x01); 
  digitalWrite(CS, HIGH);
}

void set_offset_gain()
{
    // Set offsets to default value (no offset)
  digitalWrite(CS, LOW);
  SPI.transfer( Write | (CURRENT_AC_OFFSET << 1 ));     
  SPI.transfer(ioffh); 
  SPI.transfer(ioffm);
  SPI.transfer(ioffl);
  SPI.transfer( Write | (VOLTAGE_AC_OFFSET  << 1 ));    
  SPI.transfer(voffh);
  SPI.transfer(voffm);//B2
  SPI.transfer(voffl); //E2
  SPI.transfer( Write | (CURRENT_GAIN   << 1 ));    
  SPI.transfer(igh);
  SPI.transfer(igm);
  SPI.transfer(igl);
  SPI.transfer( Write | (VOLTAGE_GAIN  << 1 ));    
  SPI.transfer(vgh);
  SPI.transfer(vgm);//asjdh
  SPI.transfer(vgl);
  digitalWrite(CS, HIGH);
}

void reset_offset_gain()
{
    // Set offsets to default value (no offset)
  digitalWrite(CS, LOW);
  SPI.transfer( Write | (CURRENT_AC_OFFSET << 1 ));     
  SPI.transfer(0x00); 
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer( Write | (VOLTAGE_AC_OFFSET  << 1 ));    
  SPI.transfer(0x00); 
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer( Write | (CURRENT_GAIN   << 1 ));    
  SPI.transfer(0x40); 
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer( Write | (VOLTAGE_GAIN  << 1 ));    
  SPI.transfer(0x40); 
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
}


void cal()
{
  digitalWrite(CS, LOW);          
  SPI.transfer(ACG_CAL);//Voltage channel ac gain
  SPI.transfer(SYNC1);
  SPI.transfer(SYNC1);
  SPI.transfer(SYNC1);
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);          
  SPI.transfer(ACOFF_CAL);//Voltage channel ac offset
  SPI.transfer(SYNC1);
  SPI.transfer(SYNC1);
  SPI.transfer(SYNC1);
  digitalWrite(CS, HIGH);
}

void start_conversions()
{
  digitalWrite(CS, LOW);
  SPI.transfer(START_CONTINOUS);
  SPI.transfer(SYNC1);
  SPI.transfer(SYNC1);
  SPI.transfer(SYNC1);
  digitalWrite(CS, HIGH);
}
