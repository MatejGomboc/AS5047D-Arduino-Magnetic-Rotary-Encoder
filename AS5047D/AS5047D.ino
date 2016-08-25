#include <SPI.h>
unsigned int reading = 0;

// AS4047D Registers
#define AS5047D_select_pin 10

/** volatile **/
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG   0x0003
#define DIAAGC 0x3FFC
#define CORDICMAG 0x3FFD
#define ANGLEUNC  0x3FFE
#define ANGLECOM  0x3FFF

/** non-volatile **/
#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

#define RD  0x40    // bit 14 "1" is Read + parity even
#define WR  0x3F    //bit 14 ="0" is Write

//Op Arduino: D10 CS, D11 MOSI, D12 MISO, D13 SCK

void setup() {
  pinMode(AS5047D_select_pin, OUTPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1); // properties chip
  SPI.setBitOrder(MSBFIRST);  //properties chip

  Serial.begin(115200);  // start serial for output
  Serial.println(" AS5047D:");

  AS5047D_Write( AS5047D_select_pin , SETTINGS1, 0x0004);
  AS5047D_Write( AS5047D_select_pin , SETTINGS2, 0x0000);
  AS5047D_Write( AS5047D_select_pin , ZPOSM, 0x0000); // is it really possible to initially set angle at 0 degrees??
  AS5047D_Write( AS5047D_select_pin , ZPOSL, 0x0000);
}

void loop()
{
  DumpRegisterValues();
  Serial.println();
  delay(200);
}

void DumpRegisterValues()
{
  Serial.print("NOP: "); Serial.println(AS5047D_Read( AS5047D_select_pin, NOP) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("ERRFL: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ERRFL) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("PROG: "); Serial.println(AS5047D_Read( AS5047D_select_pin, PROG) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("DIAAGC: "); Serial.println(AS5047D_Read( AS5047D_select_pin, DIAAGC) & 0x3FFF, BIN); // strip bit 14..15
  
  Serial.print("CORDICMAG: "); Serial.println(AS5047D_Read( AS5047D_select_pin, CORDICMAG) & 0x3FFF, DEC); // strip bit 14..15
  Serial.print("ANGLEUNC: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ANGLEUNC) & 0x3FFF, DEC); // strip bit 14..15
  Serial.print("ANGLECOM: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ANGLECOM) & 0x3FFF, DEC); // strip bit 14..15

  Serial.print("ZPOSM: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ZPOSM) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("ZPOSL: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ZPOSL) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("SETTINGS1: "); Serial.println(AS5047D_Read( AS5047D_select_pin, SETTINGS1) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("SETTINGS2: "); Serial.println(AS5047D_Read( AS5047D_select_pin, SETTINGS2) & 0x3FFF, BIN); // strip bit 14..15
}

// ************************Write to AS5047D **************************
void AS5047D_Write( int SSPin, int address, int value)
{
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  Serial.println(value, HEX);
  
  //  send in the address via SPI:
  
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  //v_h = v_h & (WR | 0x80);  // its  a write command and don't change the parity bit (0x80)
  
  Serial.print( " parity:  "); Serial.println(parity(address & 0x3F));
  Serial.print(v_h, HEX); Serial.print(" A ");  Serial.println(v_l, HEX);
  
  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //  send value via SPI:
  
  v_l = value & 0x00FF;
  v_h = (unsigned int)(value & 0x3F00) >> 8;
  
  if (parity(value & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  //v_h = v_h & (WR | 0x80); // its a write command and don't change the parity bit (0x80)
  
  Serial.print(v_h, HEX); Serial.print(" D ");  Serial.println(v_l, HEX);
  
  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(SSPin, HIGH);
}

//*******************Read from AS5047D ********************************
unsigned int AS5047D_Read( int SSPin, unsigned int address)
{
  unsigned int result = 0;   // result to return
  
  byte res_h = 0;
  byte res_l = 0;
  
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  //  send in the address and value via SPI:
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address | (RD << 8)) == 1) v_h = v_h | 0x80; // set parity bit
  
  v_h = v_h | RD; // its  a read command
  
  // Serial.print( " parity:  ");Serial.println(parity(address | (RD <<8)));
  // Serial.print(v_h, HEX); Serial.print(" A ");  Serial.print(v_l, HEX);  Serial.print(" >> ");
  
  res_h = SPI.transfer(v_h);
  res_l = SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //if (parity(0x00 | (RD <<8))==1) res_h = res_h | 0x80;  // set parity bit
  //res_h = res_h | RD;
  
  res_h = (SPI.transfer(0x00));
  res_l = SPI.transfer(0x00);
  
  res_h = res_h & 0x3F;  // filter bits outside data
  
  //Serial.print(res_h, HEX);   Serial.print(" R  ");  Serial.print(res_l, HEX);   Serial.print("  ");
  
  digitalWrite(SSPin, HIGH);
  
  return (result = (res_h << 8) | res_l);
}

//*******************check parity ******************************************
int parity(unsigned int x) {
  int parity = 0;
  while (x > 0) {
    parity = (parity + (x & 1)) % 2;
    x >>= 1;
  }
  return (parity);
}

