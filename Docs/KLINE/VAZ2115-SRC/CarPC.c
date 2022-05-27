/*****************************************************
This program was produced by the
CodeWizardAVR V2.05.0 Professional
Automatic Program Generator
© Copyright 1998-2010 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : CarPC
Version : 1.0
Date    : 15.04.2012
Author  : mr_smit
Company : 
Comments: 


Chip type               : ATmega16
Program type            : Application
AVR Core Clock frequency: 16,000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*****************************************************/

#include <mega16.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <delay.h>
#include <spi.h>
#include <LCD.c>

#ifndef RXB8
#define RXB8 1
#endif

#ifndef TXB8
#define TXB8 0
#endif

#ifndef UPE
#define UPE 2
#endif

#ifndef DOR
#define DOR 3
#endif

#ifndef FE
#define FE 4
#endif

#ifndef UDRE
#define UDRE 5
#endif

#ifndef RXC
#define RXC 7
#endif

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)

#define Connect      1
#define ReadData     2
#define ReadError    3
#define ReadADC      4
#define SpeedSample  5
#define Menu         6

// подключение тачскрина
#define Y_minus       PORTA.6
#define Y_plus        PORTA.4
#define X_minus       PORTA.7
#define X_plus        PORTA.5

int mode;
int x,y;
bit push, press, draw_rashod, fill_rashod = 0;
int press_count = 0;

unsigned int svetlii_color = 0x1A12;   // цвет заднего фона (светлый)
unsigned int temnii_color  = 0x218E;   // цвет заднего фона (темный)
unsigned int text_parametr = 0xFFE0;   // цвет параметров
unsigned int text_tablo    = 0xFFFF;   // цвет надписей

char convert[32];

#define BUFFER_SIZE 200
unsigned char buffer[BUFFER_SIZE];                       // приемный буффер

flash unsigned char startCommunication[]                 = {0x81,0x10,0xf1,0x81,0x03};
flash unsigned char readDataByLocalIdentifier_RLI_ASS[]  = {0x82,0x10,0xF1,0x21,0x01,0xA5};
flash unsigned char readDTCByStatus[]                    = {0x84,0x10,0xF1,0x18,0x00,0x00,0x00,0x9D};
flash unsigned char clearDiagnosticInformation[]         = {0x83,0x10,0xf1,0x14,0x00,0x00,0x98}; 
flash unsigned char testerPresent[]                      = {0x82,0x10,0xf1,0x3E,0x01,0xC2};
flash unsigned char readDataByLocalIdentifier_RLI_FT[]   = {0x82,0x10,0xF1,0x21,0x03,0xA7}; 

volatile bit             StartSend, first_draw, go, zamer_finish;
volatile unsigned char   counter;
volatile unsigned int    ms;
volatile unsigned int    dsec;
unsigned char            Connect_Delay;
float rashod = 0;
int rashod_middle = 0;
eeprom int ekran;

// USART Receiver interrupt service routine
interrupt [USART_RXC] void usart_rx_isr(void)
{
char status,data;
status=UCSRA;
data=UDR;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   buffer[counter] = data;
   if (++counter == BUFFER_SIZE) {
     counter = 0;
     }
   }
}

void SendCommand (flash unsigned char *command, unsigned char length) {
  counter = 0; 
  while (length--) {
    while(!(UCSRA & (1<<UDRE)));  // ждем окончания передачи байта
    UDR = *command++;
  }
}


// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
// Reinitialize Timer 0 value
TCNT0=0x06;
if (++ms > 250) {
    ms = 0;
    StartSend = 1;       // разрешаем отправку запроса через каждые 250 мс
  }
}

// Timer1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
// Reinitialize Timer1 value
TCNT1H=0xE796 >> 8;           // таймер срабатывает каждые 0,1 сек
TCNT1L=0xE796 & 0xff;

if (++dsec > 300) {           // даем на разгон 30 сек
  TCCR1B=0x00;
  go = 0;
  zamer_finish = 1;
  }
}

#define ADC_VREF_TYPE 0x40

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input) {
  ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
  delay_us(10);                  // Delay needed for the stabilization of the ADC input voltage
  ADCSRA|=0x40;                  // Start the AD conversion
  while ((ADCSRA & 0x10)==0);    // Wait for the AD conversion to complete
  ADCSRA|=0x10;
  return ADCW;
}

void TouchScan (void) {
  
  push = 0;       
  DDRA =  0b10100000;            //  X_minus, X_plus на выход, сажаем Х пленку на землю          
  PORTA = 0b01011111;            //  остальные выводы как входы с подтяжкой   
  delay_ms(1);

  if (read_adc(6) < 100) {       // если есть нажатие ( проверяем на 0 вывод Y- )
      push = 1;
      press_count++;
      if (press_count > 1) {
        press = 1;
      } 
	  PORTA = 0b01111111;        // cчитываем X координату, X_minus на землю, X_plus на +5 вольт 
      delay_ms(1);  
      x = abs((int) (232 - 0.276*read_adc(4))); 
      
      // записываем значения АЦП по 2-м точкам
      // сопоставляем координаты
      // и через уравнение прямой по 2-м точкам находим промежуточные значения нажатия 
      // x = 232 - 0.276*ADC
      // y = 0.25*ADC - 67.5              
  
	  DDRA = 0b01011111;         // cчитываем Y координату, Y_minus, Y_plus на выход  
      PORTA = 0b10111111;        //  Y_minus на землю, Y_plus на +5 вольт
      delay_ms(1);
      y = abs((int) (0.25*read_adc(5)-67.5));           
  }
  else {
    press = 0;
    press_count = 0; 
  }
}

void main(void)
{
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=T State2=T State1=P State0=T 
PORTA=0x02;
DDRA=0x00;

// Port B initialization
// Func7=Out Func6=In Func5=Out Func4=Out Func3=In Func2=In Func1=In Func0=In 
// State7=0 State6=T State5=0 State4=0 State3=T State2=T State1=T State0=T 
PORTB=0x00;
DDRB=0xFF;

// Port C initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T 
PORTC=0x00;
DDRC=0x00;

// Port D initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T 
PORTD=0x00;
DDRD=0x00;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 250,000 kHz
// Mode: Normal top=0xFF
// OC0 output: Disconnected
TCCR0=0x03;
TCNT0=0x06;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 62,500 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x00;
//TCCR1B=0x04;
TCNT1H=0xE7;
TCNT1L=0x96;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
ASSR=0x00;
TCCR2=0x00;
TCNT2=0x00;
OCR2=0x00;

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
MCUCR=0x00;
MCUCSR=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x05;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 10400
UCSRA=0x00;
UCSRB=0x98;
UCSRC=0x86;
UBRRH=0x00;
UBRRL=0x5F;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
SFIOR=0x00;

// ADC initialization
// ADC Clock frequency: 125,000 kHz
// ADC Voltage Reference: AVCC pin
// ADC Auto Trigger Source: ADC Stopped
ADMUX=ADC_VREF_TYPE & 0xff;
ADCSRA=0x87;

// SPI initialization
// SPI Type: Master
// SPI Clock Rate: 2*4000,000 kHz
// SPI Clock Phase: Cycle Start
// SPI Clock Polarity: Low
// SPI Data Order: MSB First
SPCR=0x50;
SPSR=0x01;

// TWI initialization
// TWI disabled
TWCR=0x00;

// Global enable interrupts
#asm("sei")

LCD_init();
fill_screen(bgcolor);
mode = Connect;
first_draw = 1;

while (1)
      { 
      if (PINA.1 == 1) {                                 // если пропало напряжение питания, 
          delay_ms(5);                                   // то выключаем дисплей командой выключения, 
          if (PINA.1 == 1) {                             // чтобы продлить срок его службы
            LCD_PowerOff();
          }
        }
		
      TouchScan();                                       // обрабатываем тачскрин
             
        switch (mode) {                                  // смотрим какой режим
          case Connect:                                  // если только что включились => подключаемся к ЭБУ
            if (StartSend == 1) { 
              put_string(32,60,"  ПОДКЛЮЧЕНИЕ  ",0xAFE0,1);
              delay_ms(100);
              SendCommand(startCommunication,5);
              StartSend = 0;
              if (++Connect_Delay > 8) {                       // делаем 8 попыток подключиться                       
                put_string(32,60,"НЕТ СВЯЗИ С ЭБУ",0xAFE0,1);  // если нет ответа, то выводим соответствующее сообщение
                delay_ms(2000);                                // и через 2 сек пробуем подключиться вновь
                Connect_Delay = 0;
              }
            }
            else {
              if (counter > 11) {                        // парсим  запрос + ответ 
                unsigned char i = 0;
                unsigned int crc = 0;
                for (i=5;i<11;i++) {                     // запрос + ответ = 12 байт
                  crc = crc + buffer[i];                 // считаем контрольную сумму
                }
                i =  crc & 0xFF;                         // берем 2 младших разряда 
                if ( buffer[11] == i ) {                 // проверяем контрольную сумму
                  if ( buffer[8] == 0xC1 ) {             // положительный ответ startCommunication
				    if (ekran == 1) {                    // если последний раз были в режиме данных, то
                      mode = ReadData;                   // переходим в режим чтения данных 
                      first_draw = 1;  
					}
				    else {                               // если последний раз были в режиме отображения расхода, то
				      mode = ReadData;                   
					  draw_rashod = 1;                   // переходим в режим отображения расхода
					  fill_rashod = 1;
                      first_draw = 1;  
				    }
                  }                                         
                }
              counter = 0;                               // сбрасываем counter иначе опять попадем под условие if (counter > 11)    
              }
            }
            break;
            
          case ReadData:                                 // если режим чтения данных 
            if (first_draw == 1) {                       // если перешли в этот режим, то первый раз рисуем             
              draw_rect(0,0,87,32,svetlii_color);        // задний фон, а при следующем вызове прорисовываем 
              draw_rect(88,0,175,32,temnii_color);       // только данные
              bgcolor = temnii_color;
              put_string(-3,0,"ТЕМПЕРАТУРА",text_tablo,1); 
              bgcolor = svetlii_color; 
              put_string(98,0,"ДРОССЕЛЬ",text_tablo,1);
              
              draw_rect(0,33,87,65,temnii_color);         
              draw_rect(88,33,175,65,svetlii_color);
              bgcolor = svetlii_color;      
              put_string(12,33,"ОБОРОТЫ",text_tablo,1);
              bgcolor = temnii_color;
			  put_string(98,33,"СКОРОСТЬ",text_tablo,1);
              
              draw_rect(0,66,87,98,svetlii_color);         
              draw_rect(88,66,175,98,temnii_color);
              bgcolor = temnii_color;      
              put_string(1,66,"НАПРЯЖЕНИЕ",text_tablo,1);
              bgcolor = svetlii_color;
              put_string(106,66,"ВПРЫСК",text_tablo,1);
              
              draw_rect(0,99,175,131,0x0000);         
              bgcolor = 0x0000;    
              put_string(17,99,"МГНОВЕННЫЙ РАСХОД",0xFFFF,1); 
              
			  ekran = 1;                        // записываем в eeprom что мы в этом режиме, чтобы при следующем включении мы в него и вернулись
              first_draw = 0;
            };                                           
            if (StartSend == 1) {                          // если прошло 250 мс, то есть флаг разрешения отправки запроса ЭБУ
              SendCommand(readDataByLocalIdentifier_RLI_ASS,6);
              StartSend = 0;
            }
            else  { 
              if (counter>10) {                            //  если начали принимать ответ
                unsigned char i = 0;
                unsigned int crc = 0;
                unsigned char drossel = 0;
			    unsigned char otvet_length = 0;          
                otvet_length = buffer[9];                  // длина пакета данных (4-й байт в посылке)
                if (counter > (otvet_length+10)) {         // ждем пока примется вся посылка                
                  for (i=6;i<(otvet_length+10);i++) {                    
                    crc = crc + buffer[i];                 // считаем контрольную сумму
                  }
                  i =  crc & 0xFF;                         // берем 2 младших разряда 
                  if ( buffer[(otvet_length+10)] == i ) {  // проверяем контрольную сумму 

                   if (!draw_rashod) {				       // если отображаем набор данных
                    bgcolor = temnii_color;
                    i = buffer[20] - 0x28;                 // вычисляем температуру двигателя
                    if (i & 0x80) {                        // если она ниже нуля
                      i = 256 - i;
                      sprintf(convert,"-%d°",i); 
                      if (strlen(convert) <= 3)  {         // стираем лишнее справа от параметра в случае если он стал одинарным или двойным          
                        put_string(25,11,strcat(convert, " "),text_parametr,2);
                      }
                      else {
                        put_string(25,11,convert,text_parametr,2);
                      }                   
                    }
                    else {                                 // если выше нуля
                      sprintf(convert,"%d°",i);
                      if (strlen(convert) <= 3)  {
                        put_string(25,11,strcat(convert, " "),text_parametr,2);
                      }
                      else {
                        put_string(25,11,convert,text_parametr,2);
                      }
                    }
                  
                  drossel = buffer[22];                  // положение дроссельной заслонки
                  sprintf(convert,"%d%",drossel);
                  bgcolor = svetlii_color;  
                  if (strlen(convert) < 3)  {                
                    put_string(117,11,strcat(convert, " "),text_parametr,2);
                    }
                  else {
                    put_string(117,11,convert,text_parametr,2);
                    }
                  
                  bgcolor = svetlii_color;
                  if (drossel != 0) {                     // обороты если НЕ холостой ход
                    sprintf(convert,"%d",((buffer[23])*40));
                    if (strlen(convert) < 4)  {                
                      put_string(20,44,strcat(convert, " "),text_parametr,2);
                    }
                    else {
                      put_string(20,44,convert,text_parametr,2);
                    };
                  }
                  else {                                 // обороты на холостом ходу
                    sprintf(convert,"%d",((buffer[24])*10));
                    if (strlen(convert) < 4)  {                
                      put_string(20,44,strcat(convert, " "),text_parametr,2);
                    }
                    else {
                      put_string(20,44,convert,text_parametr,2);
                    }
                  }
                  
                  bgcolor = temnii_color;                
                  sprintf(convert,"%d",buffer[29]);     // скорость
                  if (strlen(convert) < 3)  {                
                    put_string(125,44,strcat(convert, " "),text_parametr,2);
                  }
                  else {
                    put_string(125,44,convert,text_parametr,2);
				  }     
                  
                    // необходимо включить sprintf => float в настройках проекта
                    // 
                  bgcolor = temnii_color;
                  sprintf(convert,"%.1f",(5.2 + (float) buffer[30]/20));     // напряжение 
                  if (strlen(convert) < 4)  {                
                    put_string(16,77,strcat(convert, " "),text_parametr,2);
				  }
				  else {
				    put_string(16,77,convert,text_parametr,2);
				  }                                
                  
                  bgcolor = svetlii_color;                
                  sprintf(convert,"%.1f",(float) ((buffer[35] << 8) + buffer[34])/125 );     // длительность впрыска  
                  if (strlen(convert) < 4)  {                
                    put_string(114,77,strcat(convert, "  "),text_parametr,2);
				  }
				  else {
				    put_string(114,77,convert,text_parametr,2);
				  } 

				  bgcolor = 0x0000; 
				  
				  rashod = rashod + abs((float) ((buffer[43] << 8) + buffer[42])/128 );
				  if (++rashod_middle > 5) {
				    rashod = rashod/6;
					if (rashod < 20)  {
					  sprintf(convert,"%.1f",rashod);     // путевой расход топлива 
                      if (strlen(convert) < 4)  {                
                        put_string(70,110,strcat(convert, " "),text_parametr,2);
				      }
				      else {
				        put_string(70,110,convert,text_parametr,2);
				      }
                    }
                    else {
                      put_string(70,110,"19.9",text_parametr,2);
                    }
                    rashod_middle = 0;
                    rashod = 0;					
				  } 
				 }
				 else {                                  // если в режиме отображения только расхода
				   if (fill_rashod) {
				     fill_screen(svetlii_color);
                     bgcolor = svetlii_color;
					 put_string(90,103,"Л/100",0x07E0,2);
					 fill_rashod = 0;
					 ekran = 0;                           // записываем в eeprom что мы в режиме отображения только расхода
				   }
				   bgcolor = svetlii_color; 
                  
				  rashod = rashod + abs((float) ((buffer[43] << 8) + buffer[42])/128 );
				  if (++rashod_middle > 5) {
				    rashod = rashod/6;
					if (rashod < 20)  {
					  sprintf(convert,"%.1f",rashod);     // путевой расход топлива 
                      if (strlen(convert) < 4)  {                
                        put_string(50,42,strcat(convert, " "),text_parametr,4);
				      }
				      else {
				        put_string(50,42,convert,text_parametr,4);
				      }
                    }
                    else {
                      put_string(50,42,"19.9",text_parametr,4);
                    }
                    rashod_middle = 0;
                    rashod = 0;					
				  }
				   
				 }
				}
			  counter = 0;                               // сбрасываем counter иначе опять попадем под условие if (counter > (otvet_length+10))
			  }
			  }
			}
            
            if (push && !press) {                       // если ткнули стилусом
			  if (!draw_rashod) {
                if (y > 90) {                           // и угодили в расход
                  draw_rashod = 1;                      // то переключаемся на отображение только его
				  fill_rashod = 1; 
			    }
			    else {
			      first_draw = 1;
                  mode = Menu;
				  draw_rashod = 0;
			    }
			  }
			  else {
			    first_draw = 1;
                mode = ReadData;
				draw_rashod = 0;
			  }
			  rashod = 0;
              rashod_middle = 0;
            }
			break;            
			
          case ReadADC:                                  // если режим чтения АЦП
            if (first_draw == 1) {
              fill_screen(bgcolor);
              put_string(63,5,"ДМРВ",text_tablo,1);
              put_string(20,68,"ДАТЧИК КИСЛОРОДА",text_tablo,1);
              first_draw = 0;
            }           
            if (StartSend == 1) {                      
              SendCommand(readDataByLocalIdentifier_RLI_FT,6);
              StartSend = 0;
            }
            else  {
              if (counter>23) {                          //  6 байт запрос + 18 байт ответ (4-й байт в ответе - число байтов в посылке)
			    unsigned char i = 0;                     // взял фиксированную длину под себя, а вообще надо считать по 4-му байту как и с данными
			    unsigned int crc = 0;
			    for (i=6;i<23;i++) {                    
                  crc = crc + buffer[i];                 // считаем контрольную сумму
                }
                i =  crc & 0xFF;                         // берем 2 младших разряда 
			    if ( buffer[23] == i ) {                 // проверяем контрольную сумму  
                  sprintf(convert,"%.4f В", (float) (buffer[14]*5)/256);     // напряжение ДМРВ                
                  put_string(30,30,convert,text_parametr,2);
                  
                  sprintf(convert,"%.4f В", (float) (buffer[16]*5)/256);     // напряжение на датчике кислорода                
                  put_string(30,90,convert,text_parametr,2);
                }
              }
            }
            
            if (push && !press) {                        // если ткнули в любом месте экрана, то выходим в меню
              first_draw = 1;
              mode = Menu;
            }
			break;
             
          case ReadError:                                // если режим чтения ошибок 
            if (first_draw == 1) {
              fill_screen(svetlii_color);
			  draw_rect(53,105,123,127,0xF800);
              bgcolor = svetlii_color;
			  put_string(5,5,"КОЛИЧЕСТВО ОШИБОК:",text_tablo,1);
			  bgcolor = 0xF800;
			  put_string(65,112,"СБРОС",text_tablo,1);
              first_draw = 0; 
            }
            if (StartSend == 1) {                      
              SendCommand(readDTCByStatus,8);
              StartSend = 0;
            }
            else  {                                      // !!!!!!!!!!! ответ может быть произвольной длинны !!!!!!!!!!!!!!
              if (counter>12) {                          // парсим  запрос + ответ 
			    unsigned char cislo_oshibok = 0; 
			    unsigned char otvet_length = 0; 
                cislo_oshibok = buffer[12];              // смотрим число ошибок   
			    otvet_length = 13+(3*cislo_oshibok);     // длинна ответа
                if (counter > otvet_length) {            // ждем пока примется весь пакет
                  unsigned char i = 0;
			      unsigned int crc = 0;
			      for (i=8;i<otvet_length;i++) {               
                    crc = crc + buffer[i];
                  }
                  i =  crc & 0xFF;                       // берем 2 младших разряда 
			      if ( buffer[otvet_length] == i ) {     // проверяем контрольную сумму
			        bgcolor = svetlii_color;
                    sprintf(convert,"%d",cislo_oshibok);
				    put_string(150,5,convert,text_parametr,1);
                    if (cislo_oshibok != 0) {
				      for (i=0;i < cislo_oshibok;i++) {  // за раз можежем вывести на экран максимум 15 ошибок (3 столбика по 5 ошибок)
				        sprintf(convert,"Р%02x%02x",buffer[(13+(i*3))],buffer[(14+(i*3))]);
						if (i < 5) {
                          put_string(5,25+(i*15),convert,0xAFE0,1);
                        }
                        else if (i>=5 && i<10) {
                          put_string(65,25+((i-5)*15),convert,0xAFE0,1);
                        } 
                        else if (i>=10 && i<15) {
                          put_string(125,25+((i-10)*15),convert,0xAFE0,1);
                        }
				      }
                    }
                  }
                counter = 0;                             // сбрасываем counter иначе опять попадем под условие if (counter>12)				
                }		  
			  }
            }
            
            if (push && !press) {
			  if ((x > 61) && (x < 115) && (y > 108) && (y < 124)) {  // если попали на кнопку "сброс"
			    draw_rect(53,105,123,127,0x07E0);            // меняем цвет кнопки 
                bgcolor = 0x07E0;
			    put_string(65,112,"СБРОС",text_tablo,1);
                SendCommand(clearDiagnosticInformation,7);   // стираем ошибки
                delay_ms(200);                               // даём ЭБУ время на стирание ошибок (хз зачем, но мне показалось так лучше срабатывает)
			    first_draw = 1;                              // перерисовываем экран
			  }
			  else {
			    first_draw = 1;
                mode = Menu;
			  }
            }
			break;
			
		  case SpeedSample:                              // если режим замера скорости 0-100 км/ч
            if (first_draw == 1) {
              fill_screen(bgcolor);
              put_string(15,5,"РАЗГОН 0-100 КМ/Ч",text_tablo,1);
              put_string(65,25,"0.0",0xAFE0,2);
              put_string(50,57,"СКОРОСТЬ",text_tablo,1); 
			  
			  draw_rect(53,105,123,127,0xF800);
			  bgcolor = 0xF800;
			  put_string(65,112,"СБРОС",text_tablo,1);
			  
              first_draw = 0;
            }
            if (StartSend == 1) {                      
              SendCommand(readDataByLocalIdentifier_RLI_ASS,6);
              StartSend = 0;
            }
            else  { 
              if (counter>10) {                          //  если начали принимать ответ
                unsigned char i = 0;
                unsigned int crc = 0;
                unsigned char speed = 0;
			    unsigned char otvet_length = 0;          
                otvet_length = buffer[9];                // длина пакета данных (4-й байт в посылке)
                if (counter > (otvet_length+10)) {       // ждем пока примется вся посылка                
                  for (i=6;i<(otvet_length+10);i++) {                    
                    crc = crc + buffer[i];                 // считаем контрольную сумму
                  }
                  i =  crc & 0xFF;                         // берем 2 младших разряда 
                  if ( buffer[(otvet_length+10)] == i ) {  // проверяем контрольную сумму 
                  
                  speed = buffer[29];
                  if (speed == 0)  {                     // если машина остановлена
                    go = 1;                              // разрешаем замер скорости
                  }
                  if (speed > 0 && go == 1) {            // начали разгоняться    
                    if (zamer_finish) {
                      TCCR1B=0x00;
                    }
                    else {
                      TCCR1B=0x04;                       // запустили отсчет времени  
                      if (speed >= 100) {                // замер разгона до 100 км/ч
                        zamer_finish = 1;
                        TCCR1B=0x00;                          
                      }
                    }                    
                  }
				  sprintf(convert,"%.1f",((float) (dsec * 0.1)));  // выводим время
				  bgcolor = svetlii_color;
				  put_string(65,25,convert,0xAFE0,2);
                  sprintf(convert,"%d",speed);                     // показываем текущую скорость
				  if (strlen(convert) < 3)  {                
                      put_string(67,75,strcat(convert, " "),0xAFE0,2);
				  }
				  else {
				      put_string(67,75,convert,0xAFE0,2);
				  }
                  
				}
			  counter = 0;                               // сбрасываем counter иначе опять попадем под условие if (counter > (otvet_length+10))
			  }
			  }  
              
            }
			
            if (push && !press) {
			  if ((x > 61) && (x < 115) && (y > 108) && (y < 124)) {  // если попали на кнопку "сброс"
			    draw_rect(53,105,123,127,0x07E0);            // меняем цвет кнопки 
                bgcolor = 0x07E0;
			    put_string(65,112,"СБРОС",text_tablo,1); 
                
                TCCR1B=0x00;
                dsec = 0;                                    // сбрасываем замер времени 
                go = 0;
                zamer_finish = 0; 
                bgcolor = svetlii_color;
			    first_draw = 1;                              // перерисовываем экран
			  }
			  else {              
                TCCR1B=0x00;
                dsec = 0;                                   // сбрасываем замер времени 
                go = 0;
                zamer_finish = 0;
			    first_draw = 1;
                mode = Menu;
			  }
            }
            break;
			
          case Menu:
            if (first_draw == 1) {
              fill_screen(0xFFE0);
              bgcolor = svetlii_color;           
              draw_rect(8,19,83,61,0x0000);
              draw_rect(9,20,82,60,svetlii_color);
              put_string(103,36,"ОШИБКИ",text_tablo,1);          
              draw_rect(93,19,168,61,0x0000);
              draw_rect(94,20,167,60,svetlii_color); 
              put_string(6,36,"ПАРАМЕТРЫ",text_tablo,1);      
              draw_rect(8,71,83,113,0x0000); 
              draw_rect(9,72,82,112,svetlii_color);  
              put_string(107,80,"0-100",text_tablo,1);
              put_string(111,96,"КМ/Ч",text_tablo,1);      
              draw_rect(93,71,168,113,0x0000);                                                                                 
              draw_rect(94,72,167,112,svetlii_color);
              put_string(31,87,"АЦП",text_tablo,1);              
              first_draw = 0;
            }
            
            if (StartSend == 1) {                      
              SendCommand(testerPresent,6);
              StartSend = 0;
            }
            
            if (push && !press) {
              if ((x > 10) && (x < 80) && (y > 21) && (y < 59) ) {
                first_draw = 1;
                mode = ReadData;               
              } 
              if ((x > 84) && (x < 165) && (y > 21) && (y < 59) ) {  
                first_draw = 1;
                mode = ReadError; 
              } 
              if ((x > 10) && (x < 80) && (y > 73) && (y < 111) ) {
                first_draw = 1;
                mode = ReadADC;               
              } 
              if ((x > 84) && (x < 165) && (y > 73) && (y < 111) ) {  
                first_draw = 1;
                mode = SpeedSample; 
              }
            }            
            
		    break;
			
          default : break;			
		}

      }
}

// под калибровку
/*
  lcd_circle(30,30,6,0xFC00); 
  lcd_circle(30,102,6,0xFC00);
  lcd_circle(146,102,6,0xFC00);
  lcd_circle(146,30,6,0xFC00);
  put_string(23,8,"1",text_tablo,1);
  put_string(23,115,"2",text_tablo,1);
  put_string(139,115,"3",text_tablo,1);
  put_string(139,8,"4",text_tablo,1); 
            
  sprintf(convert,"%d ",x);
  put_string(74,53,convert,text_tablo,1);
  sprintf(convert,"%d ",y);           
  put_string(74,73,convert,text_tablo,1);
*/