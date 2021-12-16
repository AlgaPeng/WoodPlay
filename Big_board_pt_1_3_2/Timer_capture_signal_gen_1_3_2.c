/**
 * This is a very small example that shows how to use
 * === OUTPUT COMPARE and INPUT CAPTURE ===
 * The system uses hardware to generate precisely timed
 * pulses, then uses input capture to compare the capture period
 * to the generation period for accuracy
 *
 * There is a capture time print-summary thread
 * There is a one second timer tick thread
 * 
 * On the SECABB:
 * -- Pin RB5 and RB9 are output compare outputs
 * -- Pin RB13 is input capture input -- connect this to one of the output compares
 *
 * Modified by Bruce Land 
 * Jan 2018
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_3_2.h"
// threading library
#include "pt_cornell_1_3_2.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
////////////////////////////////////

// === thread structures ============================================
// thread control structs

// note that UART input and output are threads
static struct pt pt_print, pt_time,pt_adc  ; //, pt_input, pt_output, pt_DMA_output ;
volatile int DAC_data_A,DAC_data_B, pwm, e_radians, sum, out_v;
volatile int past_e[4] = {0,0,0,0};
volatile int k_p = 400;
volatile double k_i = 0.032;
volatile int k_d = 25000;
volatile int desired_angle = 0;
volatile int start_demo = 0;
volatile int motor_disp=0;// output values
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// system 1 second interval tick
int sys_time_seconds ;

//The measured period of the wave
short capture1, last_capture1=0, capture_period=99 ;
//The actual period of the wave
int generate_period=40000;

// == Capture 1 ISR ====================================================
// check every cpature for consistency
void __ISR(_TIMER_2_VECTOR, ipl2) C1Handler(void)
{
    mT2ClearIntFlag();
    int junk;

    DAC_data_A=ReadADC10(0);   // read the result of channel 9 conversion from the idle buffer
    AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below
    // convert DAC_data_A to current_angle in radians
    e_radians = desired_angle - DAC_data_A;  
    if(sum > 1000000){
        sum = 1000000;
    }
//    if(past_e[2] > 0 && past_e[3] < 0 ){
//        sum = 0.95*sum;
//    }
//    if(past_e[2] < 0 && past_e[3] > 0 ){
//        sum = 1.05*sum;
//    }

    out_v = k_p * e_radians + k_i * sum + k_d * (e_radians-past_e[3]);
    if(out_v < 0){
       pwm = 0; 
    }
    else if(out_v > 40000){
        pwm = 40000;
    }
    else{
        pwm = out_v;
    }
    int i = 0;
    for(i = 1; i < 4; i++){
        past_e[i] = past_e[i-1];
    }
    past_e[0] = e_radians;
    sum += e_radians;
    
    motor_disp = motor_disp + ((pwm - motor_disp)>>4);
    DAC_data_B = motor_disp >> 4;
    SetDCOC3PWM(pwm);
  // reset spi mode to avoid conflict with expander
    SPI_Mode16();
    // DAC-A CS low to start transaction
       
    
         // DAC-B CS low to start transaction
    mPORTBClearBits(BIT_4); // start transaction
     // write to spi2
    WriteSPI2(DAC_config_chan_B | (DAC_data_B & 0xfff) );
    
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // MUST read to clear buffer for port expander elsewhere in code
    junk = ReadSPI2();
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
    
    mPORTBClearBits(BIT_4); // start transaction
     // write to spi2
    WriteSPI2(DAC_config_chan_A | ((DAC_data_A<<2) & 0xfff) );
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
        // MUST read to clear buffer for port expander elsewhere in code
    junk = ReadSPI2();
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
    
}

// === Period print Thread ======================================================
// prints the captured period of the generated wave
static PT_THREAD (protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);
      // string buffer
      static char buffer[128];
      
      tft_setCursor(0, 0);
      tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
      tft_writeString("Connect RB9 \n");
      tft_setCursor(0, 20);
      tft_writeString("OR RB5 \n");
      tft_setCursor(0, 40);
      tft_writeString("to RB13 thru 300ohm\n");
      while(1) {
            // print every 200 mSec
            PT_YIELD_TIME_msec(200) ;
            // erase
            tft_setCursor(0, 80);
            tft_fillRoundRect(0,80, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            // print the periods
             sprintf(buffer,"e_radians=%d ", e_radians );
             tft_writeString(buffer);
             
             tft_fillRoundRect(0,100, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
             tft_setCursor(0, 100);
             sprintf(buffer,"DAC_data_A=%d  ", DAC_data_A);
             tft_writeString(buffer);
             
              tft_fillRoundRect(0,120, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
              tft_setCursor(0, 120);
              sprintf(buffer,"out_v=%d  ", out_v);
             
            tft_writeString(buffer);
            
      } // END WHILE(1)
  PT_END(pt);
} // thread 4

// === One second Thread ======================================================
// update a 1 second tick counter
static PT_THREAD (protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);
      static char buffer[128];
      
      while(1) {
            // yield time 1 second
            PT_YIELD_TIME_msec(1000) ;
            sys_time_seconds++ ;
         //   tft_fillRoundRect(0,150, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
         //    tft_setCursor(0, 150);
         //    sprintf(buffer,"sys_time=%d  ", sys_time_seconds);
            tft_writeString(buffer);
            // NEVER exit while
      } // END WHILE(1)

  PT_END(pt);
} // 

////////////////////////////////////
// === print a line on TFT =====================================================
// print string buffer
char tft_str_buffer[60];
// SEE 
// http://people.ece.cornell.edu/land/courses/ece4760/PIC32/index_TFT_display.html
// for details
void tft_printLine(int line_number, int indent, char* print_buffer, short text_color, short back_color, short char_size){
    // print_buffer is the string to print
    int v_pos, h_pos;
    char_size = (char_size>0)? char_size : 1 ;
    //
    v_pos = line_number * 8 * char_size ;
    h_pos = indent * 6 * char_size ;
    // erase the pixels
    //tft_fillRoundRect(0, v_pos, 239, 8, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor2(text_color, back_color); 
    tft_setCursor(h_pos, v_pos);
    tft_setTextSize(char_size);
    tft_writeString(print_buffer);
}

// === outputs from python handler =============================================
// signals from the python handler thread to other threads
// These will be used with the prototreads PT_YIELD_UNTIL(pt, condition);
// to act as semaphores to the processing threads
char new_string = 0;
// identifiers and values of controls
// current string
char receive_string[64];

// === string input thread =====================================================
// process text from python
static PT_THREAD (protothread_python_string(struct pt *pt))
{
    PT_BEGIN(pt);
    static int data;
    static double data_i;

     static char buffer[128];
    // 
    while(1){
        // wait for a new string from Python
        PT_YIELD_UNTIL(pt, new_string==1);
        new_string = 0;
        int par = receive_string[0] - '0';
        tft_fillRoundRect(0,200, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 200);
        sprintf(buffer,"par=%d", par); 
        
        tft_writeString(buffer);
        
        if(par == 4){
            sscanf(receive_string+1, "%f", &data_i);
        }
        else{
            sscanf(receive_string+1, "%d", &data);
        }
    
        tft_setTextSize(2);
        tft_fillRoundRect(0,180, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 180);
        sprintf(buffer,"data=%d  ", data); 
        tft_writeString(buffer);
        // Set the desired beam angle
        if (par == 1){
            //Set the desired beam angle
            
            desired_angle = data * 2.778 + 250;
            tft_fillRoundRect(0,150, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 150);
            sprintf(buffer,"desired_angle=%d  ", desired_angle);
            tft_writeString(buffer);
        }
        // Set the PID proportional gain
        else if (par == 2){
            k_p = data;
            tft_fillRoundRect(0,220, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 220);
            sprintf(buffer,"k_p=%d  ", k_p);
            tft_writeString(buffer);
        }
        // Set the PID differential gain
        else if (par == 3){
            k_d =data;
            tft_fillRoundRect(0,240, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 240);
            sprintf(buffer,"k_d=%d  ", k_d);
            tft_writeString(buffer);
        }
        // Set the PID integral gain
        else if (par == 4){
            k_i = data_i;
            tft_fillRoundRect(0,260, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 260);
            sprintf(buffer,"k_i=%f  ", k_i);
            tft_writeString(buffer);
        }
        else if (par == 5){
            start_demo = 1;
            desired_angle = 250;
            PT_YIELD_TIME_msec(5000);
            desired_angle = 333.34;
            PT_YIELD_TIME_msec(5000);
            desired_angle = 166.66;
            PT_YIELD_TIME_msec(5000);
            desired_angle = 250;
        }
        //
        else {
            tft_printLine(1,0, receive_string, ILI9340_GREEN, ILI9340_BLACK, 2);
            printf("received>%s", receive_string);        
        }
    } // END WHILE(1)   
    PT_END(pt);  
} // thread python_string


// === Python serial thread ====================================================
// you should not need to change this thread UNLESS you add new control types
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    static char junk;
    //   
    //
    while(1){
        // There is no YIELD in this loop because there are
        // YIELDS in the spawned threads that determine the 
        // execution rate while WAITING for machine input
        // =============================================
        // NOTE!! -- to use serial spawned functions
        // you MUST edit config_1_3_2 to
        // (1) uncomment the line -- #define use_uart_serial
        // (2) SET the baud rate to match the PC terminal
        // =============================================
        
        // now wait for machine input from python
        // Terminate on the usual <enter key>
        PT_terminate_char = '\r' ; 
        PT_terminate_count = 0 ; 
        PT_terminate_time = 0 ;
        // note that there will NO visual feedback using the following function
        PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input) );
        
        // Parse the string from Python
        
        // string from python input line
        if (PT_term_buffer[0]=='$'){
            // signal parsing thread
            new_string = 1;
            // output to thread which parses the string
            // while striping off the '$'
            strcpy(receive_string, PT_term_buffer+1);
        }                                  
    } // END WHILE(1)   
    PT_END(pt);  
} // thread blink

// === Main  ======================================================

int main(void)
{
   // the ADC ///////////////////////////////////////
        // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
        // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
        // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
        // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF //

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
        //
	// Define setup parameters for OpenADC10
        // use peripherial bus clock | set sample time | set ADC clock divider
        // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
        // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// define setup parameters for OpenADC10
	// set AN11 and  as analog inputs
	#define PARAM4	ENABLE_AN11_ANA // pin 24 PB13

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL
    
    
	// use ground as neg ref for A | use AN11 for input A     
	// configure to sample AN11 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11 ); // configure to sample AN11 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
  ///////////////////////////////////////////////////////
   
  // === Config timer and output compares to make pulses ========
  // set up timer2 to generate the wave period
  // count is Zero-based, so subtract one for # of clock ticks
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period-1);
  
  // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
  
  
  // set up compare3 for double compare mode
  // first number is the time to clear, second is the time to set the pin
  // in this case, the end of the timer period and 50% of the timer period
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , 39999, 0); //
  // OC3 is PPS group 4, map to RPB9 (pin 18)
    PPSOutput(4, RPB9, OC3);
  
  
  
 // set up DAC on big board
  // timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // at 40 MHz PB clock
    // 40,000,000/Fs = 909 : since timer is zero-based, set to 908
  //  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 908);

    // SCK2 is pin 26
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);

    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);

    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // clk divider set to 4 for 10 MHz
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 4);
   // end DAC setup
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  tft_setCursor(0, 0);
  
  // === config the uart, DMA, vref, timer5 ISR ===========
  PT_setup();

  // === setup system wide interrupts  ====================
  INTEnableSystemMultiVectoredInt();
  
  pt_add(protothread_print, 1);
  pt_add(protothread_time, 1);
  pt_add(protothread_serial, 1);
  pt_add(protothread_python_string, 1);
  
  // === initalize the scheduler ====================
  PT_INIT(&pt_sched) ;

  // >>> CHOOSE the scheduler method: <<<
  // (1)
  // SCHED_ROUND_ROBIN just cycles thru all defined threads
  //pt_sched_method = SCHED_ROUND_ROBIN ;
  
  // (2)
  // SCHED_RATE executes some threads more often then others
  // -- rate=0 fastest, rate=1 half, rate=2 quarter, rate=3 eighth, rate=4 sixteenth,
  // -- rate=5 or greater DISABLE thread!
  // pt_sched_method = SCHED_RATE ;
  
  pt_sched_method = SCHED_ROUND_ROBIN ;
  
  // === scheduler thread =======================
  // scheduler never exits
  PT_SCHEDULE(protothread_sched(&pt_sched));
  // ============================================
} // main
