//UNR CPE 301 Final
//Hunter Arends, Shang, Miguel
//12/2/2023

#include <LiquidCrystal.h>
#include <DHT.h>
#include <RTClib.h>
#include <Stepper.h>

#define RDA 0x80
#define TBE 0x20
RTC_DS1307 rtc;

const int stepsPerRevolution = 2038;  // change this to fit the number of steps per revolution

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8,10,9,11);

// UART Pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int  *)0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;
// GPIO Pointers
volatile unsigned char *portB     = (unsigned char*) 0x25;
volatile unsigned char *ddr_b     = (unsigned char*) 0x24;
volatile unsigned char *portE     = (unsigned char*) 0x2E;
volatile unsigned char *ddr_e     = (unsigned char*) 0x2D;
volatile unsigned char *pin_e     = (unsigned char*) 0x2C;

//Water Sensor Pointers: Analog pin 0
//volatile unsigned char 
//volatile unsigned char

//Motor Pointers:
  //volatile unsigned char        input 1 port  
//volatile unsigned char        input 1 ddr 
//volatile unsigned char        input 1 pin 
  //volatile unsigned char        input 2 port
//volatile unsigned char        input 2 ddr
//volatile unsigned char        input 2 pin
  //volatile unsigned char        input 3
//volatile unsigned char        input 3 port
//volatile unsigned char        input 3 ddr
//volatile unsigned char        input 3 pin
//volatile unsigned char        input 4 port
//volatile unsigned char        input 4 ddr
//volatile unsigned char        input 4 pin
  //will use pot to control speed
//volatile unsigned char        potentiometer port
//volatile unsigned char        potentiometer ddr
//volatile unsigned char        potentiometer pin


// Timer Pointers
volatile unsigned char *myTCCR1A  = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B  = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C  = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1  = (unsigned char *) 0x6F;
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   = (unsigned  int *) 0x84;
// Analog Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//global variables
double temperature = -1;
double humidity = -1;
double waterlevel = -1;
int prevState = 0;

//state of the refrigerator
enum State{
  disabled = 0,
  idle = 1,
  running = 2,
  error = 3
};

State state = disabled;

//setup temperature sensor
DHT dht(2, DHT11);


//global ticks counter
void setup() 
{ 
  //set PE5 (start button) to input             
  *ddr_e &= 0xDF;
  *portE |= 0x20;


  // set PB5,6,7 (RGB) to output
  *ddr_b |= 0x20;
  *ddr_b |= 0x40;
  *ddr_b |= 0x80;


  // setup the Timer for Normal Mode, with the TOV interrupt enabled
  //setup_timer_regs();
  // Start the UART
  U0Init(9600);
  //setup the ADC
  adc_init();
  //Serial.begin(9600);
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  attachInterrupt(digitalPinToInterrupt(3), start, RISING);
}

void loop() 
{
  if(state != prevState){
    displayTimeStamp();
    prevState = state;
  }

  switch(state){
    case 1://idle
      temperature = dht.readTemperature();
      
      //change LED to green
      *portB &= !0x01 << 5;
      *portB |= 0x01 << 6;
      *portB &= !0x01 << 7;

      //delay(1000); for motor to have time to change; need to change this to funky delay
      //myStepper.setSpeed(adc_read(potentiometer pin));

      break;
    case 2://running
    
      //delay(1000); for motor to have time to change
      //myStepper.setSpeed(adc_read(potentiometer pin));
      break;
    case 3://error

      //change LED to red
      *portB &= !0x01 << 5;
      *portB &= !0x01 << 6;
      *portB |= 0x01 << 7;
      
      //delay(1000); for motor to have time to change
      //myStepper.setSpeed(adc_read(potentiometer pin));
      break;
    case 0://disabled

      //change LED to yellow 
      *portB &= !0x01 << 5;
      *portB |= 0x01 << 6;
      *portB |= 0x01 << 7;
      break;
  }

}

void start(){
  if(state == disabled){
    state = idle;
  }else{
    state = disabled;
  }
}

/*
 * UART FUNCTIONS
 */
void U0Init(unsigned int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char U0getChar()
{
  while (!(*myUCSR0A & RDA)) ;
  return *myUDR0;
}

void U0putChar(unsigned char U0pdata)
{
  while (!(*myUCSR0A & TBE));
  *myUDR0 = U0pdata;
}


/* Analog Port functions*/
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void displayTimeStamp(){
  DateTime now = rtc.now();
  String date;
  date = now.toString("YYYY-MM-DD hh:mm:ss");
  for(int i=0; i<date.length(); ++i){
    U0putChar(date[i]);
  }
}
