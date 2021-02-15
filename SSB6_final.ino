/*
 *   Final destination of the SSB6.1 experiments.  Build this into a very low power WSPR digital radio.
 *   SI5351, SSB6.1 highly modified, OLED 128x64, rotary encoder.
 *   
 *   Using the Nokia library modified for OLED.
 */

 // functions wanted.  Use menu or tap, double tap, long press of switch
 //   tuning step change  10k 1k 100hz
 //   band change         80 40 30 20 15 10 meters
 //   mode change         CW, USB, LSB, WSPR stand alone

#include <Arduino.h>
#include <avr/interrupt.h>
#include <OLED1306_Basic.h>

#define ROW0 0          // text based rows for the 128x64 OLED
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56

#define SI5351   0x60    // i2c address
#define PLLA 26          // register address offsets for PLL's
#define PLLB 34
#define CLK0_EN   1
#define CLK1_EN   2
#define CLK2_EN   4
#define CLOCK_FREQ  2500009000ULL   //2500000000LL  792 692
//uint64_t CLOCK_FREQ = 2500009000ULL;

#define CW 0       // modes
#define USB 1
#define LSB 2
#define WSPR 3

  /* switch states */
#define IDLE_ 0
#define ARM  1
#define DARM 2
#define DONE 3
#define TAP  4
#define DTAP 5
#define LONGP 6

#define CW_PIN 2          // cw mode or wspr mode to unbalance the modulator
#define PTT_PIN 3         // ssb transmit via fet
#define KEY_PIN 4         // cw or wspr transmit via fet
#define WSPR_OFFSET 1436  // wspr audio freq

uint32_t freq;
int stp = 1000;
uint32_t bfo;
const int  bfo_divider = 88;
int band = 0;
int mode;
const uint32_t bfo_freq[] = { 9004300, 9004300, 9000000, 9004300 };  // bfo per mode, all usb except LSB

OLED1306 LCD;            // a modified version of LCD_BASIC by Rinky-Dink Electronics

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

//  I2C buffers and indexes
#define I2TBUFSIZE 128             // size power of 2. Size 128 can buffer a full row
#define I2RBUFSIZE 4               // set to expected #of reads in row, power of 2. Not doing reads this program.
#define I2INT_ENABLED 1            // 0 for polling in loop or timer, 1 for TWI interrupts
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
uint8_t gi2state;                  // global copy of i2c state

struct BAND {
  uint32_t freq;     
  int divider;    // si5351 output divider( always an even number for this application )
  int mode;       // cw usb lsb
  int pin;        // pin to assert to enable the bandpass filters
};

// band stacking registers.  Dividers for high side vfo.
struct BAND band_info[6] = {
  { 3928000,56,LSB,7 },
  { 7185000,44,LSB,8 },
  {10110000,38,CW,9 },
  {14060000,32,CW,10 },
  {21260000,24,USB,11 },
  {28500000,20,USB,12 }
  };

uint32_t wspr_freq[6] = { 3592600, 7038600, 10138700, 14095600, 21094600, 28124600 };

int8_t sstate[1];    //state of each switch, only have one for this radio

uint8_t min_,sec_,hour_;         // uptime and wspr timing
uint8_t wspr_tx_enable;          // wspr tx active

//      Download WSPRcode.exe from  http://physics.princeton.edu/pulsar/K1JT/WSPRcode.exe   and run it in a dos window 
//      Type (for example):   WSPRcode "K1ABC FN33 37"    37 is 5 watts, 30 is 1 watt, 33 is 2 watts, 27 is 1/2 watt
//      ( Use capital letters in your call and locator when typing in the message string.  No extra spaces )
//      Using the editing features of the dos window, mark and copy the last group of numbers
//      Paste into notepad and replace all 3 with "3,"  all 2 with "2," all 1 with "1," all 0 with "0,"
//      Remove the comma on the end
//  the current message is   "K1URC FN54 23"
const char wspr_msg[] = { 
 3, 3, 2, 2, 2, 0, 0, 2, 1, 2, 0, 2, 1, 1, 1, 2, 2, 2, 3, 0, 0, 1, 0, 1, 1, 3, 3, 2, 2, 0,
 2, 2, 0, 0, 3, 2, 0, 3, 0, 1, 2, 0, 2, 0, 0, 2, 3, 0, 1, 3, 2, 0, 1, 1, 2, 1, 0, 2, 0, 3,
 3, 2, 3, 2, 2, 2, 2, 1, 3, 2, 3, 0, 3, 0, 3, 0, 3, 2, 0, 3, 2, 0, 3, 0, 3, 1, 0, 2, 0, 1,
 1, 2, 1, 2, 3, 0, 2, 2, 3, 2, 2, 0, 2, 2, 1, 0, 2, 1, 2, 0, 3, 3, 1, 2, 3, 1, 2, 2, 3, 1,
 2, 1, 2, 0, 0, 1, 1, 3, 2, 2, 2, 2, 0, 1, 0, 1, 2, 0, 3, 1, 0, 2, 0, 0, 2, 2, 0, 1, 3, 0,
 1, 2, 3, 1, 0, 2, 2, 1, 3, 0, 2, 2
 };



void setup(void){
int i;
  
  Serial.begin(38400);                  // for debug
  i2init();

  pinMode(13,OUTPUT);
  // encoder
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(CW_PIN,OUTPUT);        // mode, controls qsk delay and balance of modulator
  pinMode(PTT_PIN,OUTPUT);
  pinMode(KEY_PIN,OUTPUT);

  digitalWrite(PTT_PIN,LOW);     // FET's to switch 8 volt pullup signals
  digitalWrite(KEY_PIN,LOW);

  // generate an interrupt for the I2C processing, timer 0 compare.
  //OCR0A = 0xAF;
  //TIMSK0 |= _BV(OCIE0A);

  LCD.InitLCD();                        // using a modified Nokia library for the OLED
  LCD.setFont(SmallFont);
  LCD.clrScr();
  LCD.print("SSB6.1 WSPR",0,ROW0);
  i2flush();
  delay(2000);
 // LCD.clrRow(0);   // ? clear to end from current position 
  LCD.clrScr();

  // band select pins
  for( i = 0; i < 6; ++i ){
    pinMode( band_info[i].pin, OUTPUT);
    digitalWrite( band_info[i].pin, LOW );
  }  
  
  // load the working freq values
  freq = band_info[band].freq;
  mode = band_info[band].mode;
  bfo = bfo_freq[mode];

// SI5351 init
  i2cd(SI5351,16,0x4f);   // clock 0, PLLA
  i2cd(SI5351,17,0x4f);   // clock 1, PLLA
  i2cd(SI5351,18,0x6f);   // clock 2, PLLB

  // set some divider registers that will never change
  for( i = 0; i < 3; ++i ){
    i2cd(SI5351,42+8*i,0);
    i2cd(SI5351,43+8*i,1);
    i2cd(SI5351,47+8*i,0);
    i2cd(SI5351,48+8*i,0);
    i2cd(SI5351,49+8*i,0);
  }

  digitalWrite(band_info[band].pin, HIGH);    // select front end filter for default band
  digitalWrite(CW_PIN,(mode == CW || mode == WSPR)?HIGH:LOW);

  si_pll_x(PLLB,bfo,bfo_divider,0);  
  si_load_divider(bfo_divider,2,0,1);
  si_pll_x(PLLA,freq + bfo ,band_info[band].divider,0);
  si_load_divider(band_info[band].divider,0,1,1);      // load divider and reset pll's
  delay(200);                                          // sometimes fails to init
  si_load_divider(band_info[band].divider,0,1,1);      // load divider and reset pll's
  
  
  i2cd(SI5351,3,0xff ^ (CLK0_EN + CLK2_EN) );   // turn on clocks vfo and bfo
  //  i2cd(SI5351,3,0xff ^ (CLK0_EN + CLK1_EN + CLK2_EN));   // testing only all on, remove tx PWR

  op_display();
}

// timer polling.  about 1000 cps or 8k baud screen writes
//ISR(TIMER0_COMPA_vect){
//   i2poll();
//}

// TWI interrupt version
// if twi interrupts enabled, then need a handler
// Stop does not produce an interrupt, so need to also poll in loop or via the timer 
ISR(TWI_vect){
  i2poll();
  if( gi2state == 0 ) i2poll();   // needed to get out of state zero, else double ints
 // ++ints;
}


void loop(void){
int8_t t;

    noInterrupts();
    i2poll();
    interrupts();
    t = encoder();
    if( t ) freq_change(t);

    t = switches();
    // act on switch sstate[0]
    if( t > DONE){
       switch( t ){
          case TAP:
             stp /= 10;
             if( stp == 1 ) stp = 10000; 
          break;
          case DTAP:   mode_change();  break;
          case LONGP:  band_change();  break;
       }
       op_display();
       sstate[0] = DONE;
    }

    keep_time();
    if( wspr_tx_enable ) wspr_tx( millis() );
    if( mode == WSPR ) wspr_frame();

}

//   ***************   WSPR standalone tx
void wspr_frame(){      // fixed 20 minute frame

   if( min_ == 20 && wspr_tx_enable == 0 ){
       wspr_tx_enable = 1;
       min_ = 0;
   }   
}

void wspr_tx( unsigned long t ){
static int i;
static unsigned long timer;
static uint8_t mod;
   
   if( (t - timer) < 683 ) return;   // baud time is 682.66666666 ms
   timer = t;
   if( ++mod == 3 ) mod = 0, ++timer;    // delay 683, 683, 682, etc.

   if( i == 162 ){
      tx_off();
      i = 0;                 // setup for next time to begin at zero index
      wspr_tx_enable = 0;    // flag done
      return;
   }

   // set the frequency, modulate the bfo into the passband
   si_pll_x(PLLB,bfo+WSPR_OFFSET,bfo_divider,146*wspr_msg[i]);  
   if( i == 0 ) tx_on();
   ++i; 
}


void tx_on(){

  if( mode == WSPR ) digitalWrite(KEY_PIN,HIGH);
  else digitalWrite(PTT_PIN,HIGH);
}

void tx_off(){
  
  if( mode == WSPR ){
    digitalWrite(KEY_PIN,LOW);
    si_pll_x(PLLB,bfo,bfo_divider,0);  
  }
  else digitalWrite(PTT_PIN,LOW);

}



// print out the operating parameters
void op_display(){
const uint8_t b[6] = {80,40,30,20,15,10};

  LCD.print("Step",0,ROW3);  LCD.printNumI(stp,35,ROW3,5,' ');
  LCD.print("Mode",0,ROW5);
  switch(mode){
      case CW:  LCD.print("CW  ",40,ROW5);  break;
      case USB: LCD.print("USB ",40,ROW5);  break;
      case LSB: LCD.print("LSB ",40,ROW5);  break;
      case WSPR: LCD.print("WSPR",40,ROW5); break;
  }
  LCD.print("Band",0,ROW7);
  LCD.printNumI(b[band],40,ROW7);
  
  LCD.print("Tap ",RIGHT,ROW3);
  LCD.print("DTap",RIGHT,ROW5);
  LCD.print("Long",RIGHT,ROW7);
  freq_display();
}

void freq_change(int8_t t ){

//uint32_t temp;                            // finding correct value of the 25mhz crystal
//  CLOCK_FREQ = CLOCK_FREQ + stp*(int)t;
//  temp = CLOCK_FREQ/100;
//  Serial.println(temp);
  
  freq = freq + stp*(int)t;
  si_pll_x(PLLA,freq + bfo ,band_info[band].divider,0);
  freq_display();
}

void freq_display(){
int rem;
  
   LCD.setFont(MediumNumbers);
   LCD.printNumI(freq/1000,0,ROW0,5,'/');       // '/' is a leading space with this font table
   LCD.setFont(SmallFont);
   rem = freq % 1000;
   LCD.printNumI(rem,64,ROW0,3,'0');   
 //  LCD.gotoRowCol(0,108);
 //  if( mode == CW ) LCD.puts(" CW");
 //  if( mode == LSB ) LCD.puts("LSB");
 //  if( mode == USB || mode == WSPR) LCD.puts("USB");
}

void time_display(){

   LCD.setFont(SmallFont);
   LCD.printNumI(min_,98,ROW1,2,' ');
   LCD.putch(':');
   LCD.printNumI(sec_,116,ROW1,2,'0');
   LCD.printNumI(hour_,RIGHT,ROW0);
}

void keep_time(){
static uint32_t tm;
static uint16_t tm_adj;

   if( millis() - tm < 1000 ) return;
   tm += 1000;

   if( ++sec_ >= 60 ){
      sec_ -= 60;
      if( ++min_ == 60 ) min_ = 0, ++hour_ ;
   }

   // 16 mhz clock error ?
   if( ++tm_adj >=  30 ){      // 1 millisecond adjustment in so many seconds to stay on time
      ++tm;                    // --tm running slow,  ++tm running fast.
      tm_adj = 0;
   }

   time_display();
   
}

// save current band params and load new vfo and dividers
// wspr freq not saved
void band_change(){

  digitalWrite(band_info[band].pin,LOW);
  // save current
  if( mode != WSPR ) band_info[band].freq = freq;
  band_info[band].mode = mode;
  
  if( ++band > 5 ) band = 0;
  
  digitalWrite(band_info[band].pin,HIGH);
  mode_band_change();

}

void mode_change(){

   if( mode != WSPR ) band_info[band].freq = freq;
   if( ++mode > WSPR ) mode = 0;
   band_info[band].mode = mode;
   mode_band_change();
}

// load up the si5351
void mode_band_change(){

  // load the working freq values
  mode = band_info[band].mode;
  if( mode != WSPR ) freq = band_info[band].freq;
  else freq = wspr_freq[band];

  bfo = bfo_freq[mode];

  si_pll_x(PLLB,bfo,bfo_divider,0);  
  si_load_divider(bfo_divider,2,0,1);
  si_pll_x(PLLA,freq + bfo ,band_info[band].divider,0);
  si_load_divider(band_info[band].divider,0,1,1);      // load divider and reset pll's  

  digitalWrite(CW_PIN,(mode == CW || mode == WSPR)?HIGH:LOW);

}

int8_t encoder(){   /* read encoder, return 1, 0, or -1 */
  
static int8_t mod;     /* encoder is divided by 4 because it has detents */
static int8_t dir;     /* need same direction as last time, effective debounce */
static int8_t last;    /* save the previous reading */
int8_t new_;     /* this reading */
int8_t b;

   new_ = PINC & 3;  
   if( new_ == last ) return 0;       /* no change */

   b = ( (last << 1) ^ new_ ) & 2;  /* direction 2 or 0 from xor of last shifted and new data */
   last = new_;
   if( b != dir ){
      dir = b;
      return 0;      /* require two in the same direction serves as debounce */
   }
   mod = (mod + 1) & 3;       /* divide by 4 for encoder with detents */
   if( mod != 2 ) return 0;

   if( dir == 2 ) return 1;   /* swap return values if it works backwards */
   else return -1;
}

      /* run the switch state machine, generic code for multiple switches even though have only one here */
int8_t switches(){
static uint8_t press_, nopress;
static uint32_t tm;
int  i,j;
int8_t sw;
int8_t s;

   if( tm == millis() ) return 0;      // run once per millisecond
   tm = millis();
   
   /* get the switch readings, low active but invert bits */
   sw = ((PINC & 0x04) >> 2) ^ 0x01;                 
   
   if( sw ) ++press_, nopress = 0;       /* only acting on one switch at a time */
   else ++nopress, press_ = 0;           /* so these simple vars work for all of them */

   /* run the state machine for all switches in a loop */
   for( i = 0, j = 1; i < 1; ++i ){
      s = sstate[i];
      switch(s){
         case DONE:  if( nopress >= 100 ) s = IDLE_;  break;
         case IDLE_:  if( ( j & sw ) && press_ >= 30 ) s = ARM;  break; /* pressed */
         case ARM:
            if( nopress >= 30 ) s = DARM;                      /* it will be a tap or double tap */
            if( press_ >= 240 ) s = LONGP;                     // long press
         break;
         case DARM:
            if( nopress >= 240 )  s = TAP;
            if( press_ >= 30 )    s = DTAP;
         break;
      }      
      sstate[i] = s; 
      j <<= 1;
   }
   
   return sstate[0];      // only one switch implemented so can return its value
}



/*****  Non-blocking  I2C  functions   ******/
void i2init(){
  TWBR = 12;   //8 500k,  12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
  TWDR = 0xFF;
  PRR &= 0x7F;
  TWSR = 1<<TWEN;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2start( unsigned char adr ){
unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}


void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t  next;

  // check for buffer full
  next = (i2in + 1) & (I2TBUFSIZE-1);
  if( next == i2out ){
     digitalWrite(13,HIGH);         // see if hangs here
     while( next == i2out ){
         noInterrupts();
         i2poll();        // wait for i2out to move
         interrupts();
     }
    //++waits;
    digitalWrite(13,LOW);
  }
  
  i2buf[i2in++] = data;
  i2in &= (I2TBUFSIZE - 1);
  
  noInterrupts();
  i2poll();         // wake up interrupts when new data arrives in case stopped in state 0 or 3
  interrupts();     // using polling, but can keep this here

}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // call flush to empty out the buffer, waits on I2C transactions completed 
uint8_t  ex;

  ex = 1;
  while(ex){
     noInterrupts();
     ex = i2poll(); 
     interrupts();
  }
}

// queue a read that will complete later
void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty ){
unsigned int dat;

   i2start( adr );
   i2send( reg );
   // i2stop();     // or repeated start
   dat = ((unsigned int)qty << 10) | ( adr << 1 ) | ISTART | 1;   // a start with the read bit set
   i2send( dat );
   i2stop();        // stop to complete the transaction
}

int i2read_int(){     // returns 2 values in i2c read queue as a signed integer
int data;

      if( i2rout == i2rin ) return 0;
      data = i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      data <<= 8;
      if( i2rout == i2rin ) return data;
      data |= i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      return data;
}

uint8_t i2available(){
uint8_t qty;

     qty = i2rin - i2rout;
     if( qty > I2RBUFSIZE ) qty += I2RBUFSIZE;    // some funky unsigned math
     return qty;
}



uint8_t i2poll(){    // everything happens here.  Call this from loop. or interrupt
static  uint8_t state = 0;
static unsigned int data;
//static uint8_t delay_counter;
static unsigned int read_qty;
static uint8_t first_read;
   
   switch( state ){
     
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2TBUFSIZE - 1 );
         
           if( data & ISTART ){   // start
              if( data & 1 ) read_qty = data >> 10;     // read queued
              data &= 0xff;
              // set start condition
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (I2INT_ENABLED); 
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (I2INT_ENABLED);
              state = 3;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
              //delay_counter = 5;   // delay for transmit active to come true
              state = 2;
           }
        }
        else TWCR = (1<<TWEN);      // stop interrupts, keep enabled
      break; 
      case 1:  // wait for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
            //delay_counter = 5;
         }
      break;
      case 2:  // wait for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear. TWINT does not return to high and no interrupts happen on stop.
         if( (TWCR & (1<<TWSTO)) == 0 ){
            state = 0;
         } 
      break;
      case 4:  // read until count has expired
         if( (TWCR & (1<<TWINT)) ){
            // read data
            if( first_read ) first_read = 0;       // discard the 1st read, need 8 more clocks for real data
            else{
               i2rbuf[i2rin++] = TWDR;
               i2rin &= ( I2RBUFSIZE - 1 );
               if( --read_qty == 0 ) state = 0;    // done
            }
            
            if( read_qty ){                        // any left ?
               if( read_qty > 1 ) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (I2INT_ENABLED);  // not the last read
               else TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);                            // nack the last read
            }
            //delay_counter = 5;
         }
      break;    
   }
   
   gi2state = state;
   if( i2in != i2out ) return (state + 8);
   else return state;
}

/******   SI5351  functions   ******/

void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){
  // direct register writes.  A possible speed up could be realized if one were
  // to use the auto register inc feature of the SI5351

   i2start(addr);
   i2send(reg);
   i2send(dat);
   i2stop();
}

void  si_pll_x(unsigned char pll, uint32_t freq, uint32_t out_divider, uint32_t fraction ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t cl_freq;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;

   cl_freq = CLOCK_FREQ;
   
   //if( pll == PLLA ) cl_freq += drift;               // drift not applied to 3 mhz calibrate freq
  // cl_freq += drift;                                 // drift applied to 3 mhz.  Pick one of these.
   
   c = 1000000;     // max 1048575
   pll_freq = 100ULL * (uint64_t)freq + fraction;    // allow fractional frequency for wspr
   pll_freq = pll_freq * out_divider;
   a = pll_freq / cl_freq ;
   r = pll_freq - a * cl_freq ;
   b = ( c * r ) / cl_freq;
   bc128 =  (128 * r)/ cl_freq;
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ) P2 = 0;        // avoid negative numbers 
   P3 = c;

   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
   
 //  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset  
}


// load new divider for specified clock, reset PLLA and PLLB if desired
void si_load_divider( uint32_t val, uint8_t clk , uint8_t rst, uint8_t Rdiv){
uint8_t R;

   R = 0;
   Rdiv >>= 1;      // calc what goes in the R divisor field
   while( Rdiv ){
      ++R;
      Rdiv >>= 1;
   }
   R <<= 4;     
   
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, ((val >> 16 ) & 3) | R );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset needed
}
