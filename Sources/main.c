/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2017
***********************************************************************
	 	   			 		  			 		  		
 Team ID: < 24 >

 Project Name: < Distance Monitoring System >

 Team Members:

   - Team/Doc Leader: < Emilio Sison >      Signature:   ______Emilio Sison__________
   
   - Software Leader: < Daniel E. Ruano >      Signature: ______Daniel Ruano_________

   - Interface Leader:< Alex Helton >    Signature: ________Alex Helton______________

   - Peripheral Leader: < Alex/Daniel >    Signature: _____^^^^^^_________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to create a system that can be used to retrofit older vehicles
 with parking sensors and crash avoidance assistance technology.


***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1.  LCD color shifts depending on distance from sensor (SPI)

 2.  Sound for parking mode increses frequency when getting closer to object (PWM)

 3.  LCD shows distance readings

 4.  Button sampling to switch between parking mode and cruising mode

 5.  ATD long range sensor, and TIM short range sensor

***********************************************************************

  Date code started: < 12/1/17 >

  Update history (add an entry every time a significant change is made):

  Date: < 12/1/17 >  Name: < Daniel R >   Update: < Initial design >

  Date: < 12/2/17 >  Name: < Alex H >   Update: < Sensor interfacing >

  Date: < 12/3/17 >  Name: < Daniel R >   Update: < Clean up sensor readings a little >

***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
// LCD drivers (written previously)
void shiftout(char);
void lcdwait(void);
void send_byte(char);
void send_i(char);
void chgline(char);
void print_c(char);
void pmsglcd(char[]);
void colorlcd(unsigned char, unsigned char, unsigned char);
//high level functions
void display(void);
void set_status(char);
void set_period(int);
int get_long_dist(void); 

/* Variable declarations */

//mode variables
#define NUM_MODES 2
#define MODE_BUTTON_PORT PTAD_PTAD7
char modebtn_last = 0;
char modebtn_current = 0;
char modebtn = 0; //mode button
char mode = 0; //mode flag (short or long distance)

//long distance variables
#define MODE_LONG 0
#define MODE_LONG_PORT PTAD_PTAD0 
#define MODE_LONG_MIN_DIST 3    //minimum safe distance (TWEAK THIS)
#define MODE_LONG_MAX_DIST 23    //maximum sensor distance (FIX THIS)
#define MODE_LONG_MAX_DELTA 10   //max safe change between samples (TWEAK THIS)
unsigned char long_dist_prev = 0;
unsigned char long_dist = 0;

//short distance variables
#define MODE_SHORT 1
#define MODE_SHORT_FRONT_TRIGGER PTT_PTT7
#define MODE_SHORT_FRONT_ECHO PTT_PTT6
#define MODE_SHORT_BACK_TRIGGER PTT_PTT5
#define MODE_SHORT_BACK_ECHO PTT_PTT4
#define MODE_SHORT_MIN_DIST 24  //minimum safe distance
#define MODE_SHORT_WARN_DIST 48 //warning distance
#define MODE_SHORT_WARN_DELTA MODE_SHORT_WARN_DIST - MODE_SHORT_MIN_DIST
#define MODE_SHORT_MAX_DIST 160 
unsigned int short_front_dist = 0; //distance to front sensor
unsigned int short_back_dist = 0; //distance to back sensor
char front_rise_flag = 0; //indicates whether front short sensor is echoing
char back_rise_flag = 0;  //indicates if back short sensor is echoing
unsigned int short_front_start_time = 0;  //Store tcnt value on rise edge
unsigned int short_front_end_time = 0;  //Store tcnt value on fall edge
unsigned int short_back_start_time = 0; 
unsigned int short_back_end_time = 0;
int rti_second_counter = 0;

//danger alerts
#define ALERT_AUDIO_PORT PTT_PTT0
#define ALERT_STATUS_SAFE 0
#define ALERT_STATUS_WARNING 1
#define ALERT_STATUS_DANGER 2
char alert_status = 0; // 0 = safe, 1 = warning, 2 = danger
   	   			 		  			 		       

/* Special ASCII characters */
#define CR 0x0D		// ASCII return 
#define LF 0x0A		// ASCII new line 

/* LCD COMMUNICATION */
#define LCDCLK_PORT PTT_PTT3
#define LCD_RW_PORT PTT_PTT2
#define LCD_RS_PORT PTT_PTT1
#define LCD_RED_PORT PTM_PTM2
#define LCD_GREEN_PORT PTM_PTM1
#define LCD_BLUE_PORT PTM_PTM0

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1 0x80	// LCD line 1 cursor position
#define LINE2 0xC0	// LCD line 2 cursor position

	 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */

/* SPI Initial */
/* Initialize SPI for baud rate of 6 Mbs */
  SPIBR = 0x01; //set baud rate to 6Mbs
  SPICR1 = 0x50;
  SPICR2 = 0x00;

/* Buttons */

  DDRAD = 0x00;
  ATDDIEN = 0b10000000;  //enable digital inputs
  
/* 
 Initialize the ATD to sample a D.C. input voltage (range: 0 to 5V)
 on Channel 0 (connected to a 10K-ohm potentiometer). The ATD should
 be operated in a program-driven (i.e., non-interrupt driven), normal
 flag clear mode using nominal sample time/clock prescaler values,
 8-bit, unsigned, non-FIFO mode.
                         
 Note: Vrh (the ATD reference high voltage) is connected to 5 VDC and
       Vrl (the reference low voltage) is connected to GND on the 
       9S12C32 kit.  An input of 0v will produce output code $00,
       while an input of 5.00 volts will produce output code $FF
*/	 	   			 		  			 		  		
	ATDCTL2 = 0x80;
	ATDCTL3 = 0x10;
	ATDCTL4 = 0x85;
  
/* RTI at 2.048 ms */

  CRGINT = CRGINT | 0x80;
  RTICTL = RTICTL | 0x50;

/* TIM module enable */

  DDRT = 0b10101111;

  TSCR1 = 0x80;
  TSCR2 = 0x06;  //Enable overflow interrupt every .17 sec 24MHz / 64
  TIOS = 0x00;
  TCTL3 = TCTL3 | 0x33; //Trigger Interrupt when output goes high or low 
  TFLG1 = 0x50;

/* Initialize PWM for audio output */

  MODRR = 0x01; //set PT0 to PWM
  PWME = 0x01;  //enable PWM0
  PWMPOL = 0x01; //set active high
  PWMCTL = 0x00;  //left aligned
  PWMCAE = 0x00;  //8-bit
  PWMPER0 = 0xFF; //max period
  PWMDTY0 = 0x7F; //initially off
  PWMCLK = 0x01;  //clock SA
  PWMPRCLK = 0x07; //A scalar -> 128
  PWMSCLA = 0x02; //SA scalar -> 128
            
/* Initialize interrupts */

/* 
   Initialize the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up     
*/ 
	DDRM = 0b00110111; 	   			 		  			 		  		
	LCDCLK_PORT = 1;
	LCD_RW_PORT = 0;
	send_i(LCDON);
	send_i(TWOLINE);
	send_i(LCDCLR);
	lcdwait();
	
	pmsglcd("Started...");   
   	      
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  	DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;
	

long_dist = get_long_dist();
long_dist_prev = long_dist;

 for(;;) {
  
/* < start of your main loop > */ 

    //swap between modes
    if (modebtn) {
        modebtn = 0;
        mode = (++mode) % NUM_MODES;
        set_period(0);
        
        if (mode == MODE_SHORT) {
             TIE_C6I = 1; //enable TIM 6 interrupt
             TIE_C4I = 1; //enable TIM 4 interrupt
             TSCR2_TOI = 1; //enable TIM overflow interrupt
        } 
        else {
            TIE_C6I = 0;
            TIE_C4I = 0;
            TSCR2_TOI = 0;
        }
    }
    

    
    if (mode == MODE_LONG) {
       int dist = get_long_dist();
       
    
       if (dist >= 2) {
         long_dist = dist;
       }
    
       if (long_dist < MODE_LONG_MIN_DIST) {
           set_status(ALERT_STATUS_DANGER);
           set_period(50);
       } 
       else if (long_dist_prev - long_dist > MODE_LONG_MAX_DELTA) {
           set_status(ALERT_STATUS_DANGER);
           set_period(50);
       }
       else {
           set_status(ALERT_STATUS_SAFE);
           set_period(0);
       }
       
       long_dist_prev = long_dist;
    }
    else if (mode == MODE_SHORT) {
       if (short_front_dist < MODE_SHORT_MIN_DIST || short_back_dist < MODE_SHORT_MIN_DIST) {
           set_status(ALERT_STATUS_DANGER);
           set_period(50);
       } 
       else if (short_front_dist < MODE_SHORT_WARN_DIST || short_back_dist < MODE_SHORT_WARN_DIST) {
           set_status(ALERT_STATUS_WARNING);
       
           
           if (short_back_dist < MODE_SHORT_WARN_DIST) {
                set_period(100-(MODE_SHORT_WARN_DIST - short_back_dist)*50/MODE_SHORT_WARN_DELTA);
           }
           else if (short_front_dist < MODE_SHORT_WARN_DIST) {
                set_period(100-(MODE_SHORT_WARN_DIST - short_front_dist)*50/MODE_SHORT_WARN_DELTA);
           }
  
       }
       else {
           set_status(ALERT_STATUS_SAFE);
           set_period(0); 
       }
    }
    
    if(rti_second_counter == 250) {
  	   rti_second_counter = 0;
       display();
    }
  
   } /* loop forever */
   
}   /* do not leave main */

void set_status(char status) {
    alert_status = status;
    
    if (status == ALERT_STATUS_SAFE) {
        colorlcd(0, 1, 0);  
    } 
    else if (status == ALERT_STATUS_WARNING) {
        colorlcd(0, 1, 1);
      
    } 
    else if (status == ALERT_STATUS_DANGER) {
        colorlcd(0, 0, 1);
    }
}

void set_period(int period) {
  //50% duty cycle square wave
  PWMPER0 = period;
  PWMDTY0 = period/2;
}

int get_long_dist(void) {
  	ATDCTL5 = 0x10;
		while((ATDSTAT0 & 0x80) == 0) {}
    return ATDDR0H * MODE_LONG_MAX_DIST / 255;
}

/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80;
  	rti_second_counter++;
  	
  	//sample buttons
  	modebtn_current = MODE_BUTTON_PORT;
  	
  	if (modebtn_current && !modebtn_last) {
  	  modebtn = 1;
  	}
  	
  	//save current samples
  	modebtn_last = modebtn_current;
}

/*
***********************************************************************                       
  TIM interrupt service routine	 

	Read sensor values and then trigger them again
 ***********************************************************************
*/

interrupt 16 void TIM_OVR(void) {
    int i = 0;
    int front = 0;
    int back = 0;
    front = short_front_end_time - short_front_start_time;  //Calculating distance (need meth to convert to inches)
    front = (front / 111) * 2;  //Adjusting distance using meth
    back = short_back_end_time - short_back_start_time;
    back = (back / 111) * 2;
    front_rise_flag = 0;
    back_rise_flag = 0;
    
	//store value if it's valid
    if (front > 1) {
        short_front_dist = front;
    }
    if (back > 1) {
        short_back_dist = back;
    }
    
    MODE_SHORT_FRONT_TRIGGER = 1;
 	  MODE_SHORT_BACK_TRIGGER = 1;
    for(i = 0; i < 250; i ++) {  //Trigger Next Measurement 
    }
    MODE_SHORT_FRONT_TRIGGER = 0;
    MODE_SHORT_BACK_TRIGGER = 0;
    TFLG2_TOF = 1;
}

//Timestamp the front short-range sensor pulse
interrupt 14 void TIM_PT6(void) {
    if(front_rise_flag == 0) {
      front_rise_flag = 1;
      short_front_start_time = TC6;  //Getting rising edge
    } else {
      front_rise_flag = 0;
      short_front_end_time = TC6;  //Getting falling edge
    }
    TFLG1_C6F = 1;
}

//Timestamp the back short-range sensor pulse
interrupt 12 void TIM_PT4(void)
{
    if(back_rise_flag == 0) {
      back_rise_flag = 1;
      short_back_start_time = TC4;  //Getting rising edge
    } else {
      back_rise_flag = 0;
      short_back_end_time = TC4; //Getting falling edge
    }
    TFLG1_C4F = 1; 	    
}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
 


}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}

/*
***********************************************************************                              
 Distance display routine - display
***********************************************************************
*/

void display(void)
{
  send_i(LCDCLR);
  chgline(LINE1);
  

  if (mode == MODE_LONG) {
     pmsglcd("Dist = ");
     print_c((long_dist/100)%10 + 48);
     print_c((long_dist/10)%10 + 48);
     print_c((long_dist%10) + 48);
     pmsglcd(" ft");
  }
  else if (mode == MODE_SHORT) {
     if (short_front_dist < MODE_SHORT_MAX_DIST) {
        pmsglcd("Front = ");
        print_c((short_front_dist/100)%10 + 48);
        print_c((short_front_dist/10)%10 + 48);
        print_c((short_front_dist%10) + 48);
        pmsglcd(" in");
     }
     else {
        pmsglcd("Frnt Out of Range");
     }
     
     chgline(LINE2);
     
     if (short_back_dist < MODE_SHORT_MAX_DIST) {
       pmsglcd("Back = ");
       print_c((short_back_dist/100)%10 + 48);
       print_c((short_back_dist/10)%10 + 48);
       print_c((short_back_dist%10) + 48);
       pmsglcd(" in");
     } 
     else {
       pmsglcd("Back Out of Range");
     }
  }
    
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  It should shift MSB first.  
            MISO = PM[4]
            SCK  = PM[5]
***********************************************************************
*/
 
void shiftout(char x)
{
	int i;
  // test the SPTEF bit: wait if 0; else, continue
  while (SPISR_SPTEF == 0) {}
  // write data x to SPI data register
  SPIDR = x;
  // wait for 30 cycles for SPI data to shift out
	for (i = 0; i < 30; i++) {}
}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
	int i;
	for (i = 0; i < 7500; i++) {}
}

/*
*********************************************************************** 
  send_byte: writes character x to the LCD
***********************************************************************
*/

void send_byte(char x)
{
     // shift out character
	 shiftout(x);
     // pulse LCD clock line low->high->low
	 LCDCLK_PORT = 0;
	 LCDCLK_PORT = 1;
	 LCDCLK_PORT = 0;
     // wait 2 ms for LCD to process data
	 lcdwait();
}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD  
***********************************************************************
*/

void send_i(char x)
{
    // set the register select line low (instruction data)
		LCD_RS_PORT = 0;
    // send byte
		send_byte(x);
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
	send_i(CURMOV);
	send_byte(x);
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD            
***********************************************************************
*/
 
void print_c(char x)
{
	LCD_RS_PORT = 1;
	send_byte(x);
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[])
{
	int i;
	for (i = 0; str[i] != '\0'; i++) {
		print_c(str[i]);
	}
}

/*
***********************************************************************
  colorlcd: set the RGB color on LCD
***********************************************************************
*/

void colorlcd(unsigned char red, unsigned char green, unsigned char blue) {
  LCD_RED_PORT = red;
  LCD_GREEN_PORT = green;
  LCD_BLUE_PORT = blue; 
}