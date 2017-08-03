#include<avr/interrupt.h>
#include<avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <compat/deprecated.h>
#define BAUD 9600                                 // define baud
#define BAUDRATE ((16000000)/(BAUD*16UL)-1)            // set baud rate value for UBRR,16000000 is the clock frequency
static int lux_threshold_lower=0,lux_threshold_higher=0;    //to be set via setup policy
static int temp_threshold=0;                                //to be set via setup policy
volatile int count=0;
int LDR_Value;
volatile char command[4];
char *lux1, *lux2, *temperature, *power;
char bright = 'a';
char off='0'; // when LED has to be switched off
uint16_t v,current,current_temp,current_lux;
enum State{Fallback,Active,Waiting};
State state=Fallback;
ISR(USART_RXC_vect)
{
  char ReceivedByte[4];
  *ReceivedByte = UDR;
  //UDR=*ReceivedByte;
  *command=*ReceivedByte;
  count=0;
}
ISR(TIMER1_COMPA_vect)
{
  count++;
}
uint16_t ReadADC(uint8_t ch)
//unsigned char ReadADC(uint8_t ch)
{
   //Select ADC Channel ch must be 0-7
   uint8_t channel;
   channel=ch&0b00000111;
   ADMUX|=channel;

   //Start Single conversion
   ADCSRA|=(1<<ADSC);

   //Wait for conversion to complete
   while(ADCSRA & (1<<ADSC));
   while(!(ADCSRA & (1<<ADIF)));

   //Clear ADIF by writing one to it
   //Note you may be wondering why we have write one to clear it
   //This is standard way of clearing bits in io as said in datasheets.
   //The code writes '1' but it results in setting bit to '0' !!!

   ADCSRA|=(1<<ADIF);

   return(ADC);
   //return(ADCH);
}
void InitADC()
{
  ADMUX=(1<<REFS0);                         // For Aref=AVcc;
  ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescalar div factor =128, pg
}
void set_brightness(char b)
{
	if(b=='0'){		
		sbi(PORTD,7);
		sbi(PORTB,2);
		sbi(PORTB,5);
		sbi(PORTD,5);
		sbi(PORTD,6);
		sbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}
    else if(b=='1'){
		cbi(PORTD,7);
		sbi(PORTB,2);
		sbi(PORTB,5);
		sbi(PORTD,5);
		sbi(PORTD,6);
		sbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}
	else if(b=='2'){
		cbi(PORTD,7);
		cbi(PORTB,2);
		sbi(PORTB,5);
		sbi(PORTD,5);
		sbi(PORTD,6);
		sbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}
	else if(b=='3'){
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		sbi(PORTD,5);
		sbi(PORTD,6);
		sbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}
    else if(b=='4'){
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		sbi(PORTD,6);
		sbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}	     
	else if(b=='5'){
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		cbi(PORTD,6);
		sbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}
	else if(b=='6'){
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		cbi(PORTD,6);
		cbi(PORTB,3);
		sbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}  	
    else if(b=='7'){
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		cbi(PORTD,6);
		cbi(PORTB,3);
		cbi(PORTB,4);
		sbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}	        
	else if(b=='8'){  
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		cbi(PORTD,6);
		cbi(PORTB,3);
		cbi(PORTB,4);
		cbi(PORTD,4);
		sbi(PORTD,2);
		sbi(PORTD,3);}
	else if(b=='9'){	 
		cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		cbi(PORTD,6);
		cbi(PORTB,3);
		cbi(PORTB,4);
		cbi(PORTD,4);
		cbi(PORTD,2);
		sbi(PORTD,3);}	
	else if(b=='a'){	 
                cbi(PORTD,7);
		cbi(PORTB,2);
		cbi(PORTB,5);
		cbi(PORTD,5);
		cbi(PORTD,6);
		cbi(PORTB,3);
		cbi(PORTB,4);
		cbi(PORTD,4);
		cbi(PORTD,2);
		cbi(PORTD,3);}
}
uint16_t get_temperature()

{
  uint16_t adc_result1;
  ADCSRA &= ~((1<<ADEN));
  InitADC();
  _delay_ms(10);
  adc_result1=ReadADC(4);
  itoa(adc_result1,temperature,10);
  UDR=*temperature;
  return(adc_result1);    //to compare with threshold limit;
}
uint16_t get_temperature_Fallback()

{
  uint16_t adc_result1;
  ADCSRA &= ~((1<<ADEN));
  InitADC();
  _delay_ms(10);
  adc_result1=ReadADC(4);
  return(adc_result1);    //to compare with threshold limit;
}
uint16_t get_lux()
{
  uint16_t adc_result1,adc_result2;
  ADCSRA &= ~((1<<ADEN));
  InitADC();
  _delay_ms(10);
  
  adc_result1=ReadADC(2);
  
  ADCSRA &= ~((1<<ADEN));
  InitADC();	
  _delay_ms(10);
  
  adc_result2=ReadADC(0);
  itoa(adc_result1,lux1,10);
  itoa(adc_result2,lux2,10);
  UDR = *lux1;
  UDR = *lux2;
  return (adc_result1);      // assuming ambient lux sensor is connected to channel 2
}
uint16_t get_lux_Fallback()
{
  uint16_t adc_result1,adc_result2;
  ADCSRA &= ~((1<<ADEN));
  InitADC();
  _delay_ms(10);
  
  adc_result1=ReadADC(2);
  
  ADCSRA &= ~((1<<ADEN));
  InitADC();	
  _delay_ms(10);
  
  adc_result2=ReadADC(0);
  return (adc_result1);      // assuming ambient lux sensor is connected to channel 2
}
void get_power()
{
  uint16_t adc_result1,adc_result2;
  uint32_t adc_result3;
  ADCSRA &= ~((1<<ADEN));
  InitADC();
  _delay_ms(10);

  adc_result1=ReadADC(1);
  ADCSRA &= ~((1<<ADEN));
  InitADC();
  _delay_ms(10);
  adc_result2=ReadADC(3);
  adc_result3 = (uint32_t)adc_result1*adc_result2;
  itoa(adc_result3,power,10);
  UDR = *power;
}
void Setup_Policy()
{
  //setup policy
}
void setup()
{
  Setup_Policy();
  UBRRH = (BAUDRATE>>8);                      // shift the register right by 8 bits
  UBRRL = BAUDRATE;                           // set baud rate
  UCSRB  = (1<<TXEN)|(1<<RXEN);                // enable receiver and transmitter
  UCSRC  = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);   // 8bit data format
  UCSRB |= (1<<RXCIE);
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A =15624;// = [(16*10^6) / (1*1024) - 1] (must be <65536) for 1 second 
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK |= (1 << OCIE1A);
  sei();
  DDRB |= 0x3C; 
  DDRD |= 0xFC;
}
void loop()
{ 
  if(count==0) state=Active;
  switch(state){
    case(Active):
    { 
      current_temp=get_temperature();
      current_lux=get_lux();
      get_power();
      (command[1]=='1')? set_brightness(command[2]): set_brightness(off);
      state=Waiting;
      break;
    }
    case(Fallback):
    {
      current_lux=get_lux_Fallback();
      current_temp=get_temperature_Fallback();
      if(current_lux<lux_threshold_lower)
      {
        set_brightness(bright);  
      }
        if(current_lux>lux_threshold_higher)
      {
        set_brightness(off);  
      }
        if(current_temp<temp_threshold)
      {
        set_brightness(off);  
      }
      break;
    }
    case(Waiting):
    {
      break;
    }
  }
  if(count>60)
    state=Fallback;
}
