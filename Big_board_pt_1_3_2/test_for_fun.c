#include "config_1_3_2.h"
// threading library
#include "pt_cornell_1_3_2.h"
#include "port_expander_brl4.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// need for sine function
#include <math.h>
// The fixed point types
#include <stdfix.h>

// lock out timer interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR
#define start_spi2_critical_section INTEnable(INT_T2, 0);
#define end_spi2_critical_section INTEnable(INT_T2, 1);

////////////////////////////////////
// some precise, fixed, short delays
// to use for extending pulse durations on the keypad
// if behavior is erratic
#define NOP asm("nop");
// 20 cycles 
#define wait20 NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// 40 cycles
#define wait40 wait20;wait20;

////////////////////////////////////
// string buffer
char buffer[60];
// === Animation Thread =============================================
// update a 1 second tick counter

#define float2Accum(a) ((_Accum)(a))
#define Accum2float(a) ((float)(a))
#define int2Accum(a) ((_Accum)(a))
#define Accum2int(a) ((int)(a))
////////////////////////////////////
// Audio DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// audio sample frequency
#define Fs 44000.0
// need this constant for setting DDS frequency
#define two32 4294967296.0 // 2^32 
// sine lookup table for DDS
#define sine_table_size 256
volatile unsigned int tfs=two32/Fs;
volatile _Accum sine_table[sine_table_size];
// phase accumulator for DDS
volatile unsigned int DDS_phase = 0;
volatile unsigned int DDS_increment = 261.63*two32/Fs;
volatile _Accum Fout = 261.63;
// waveform amplitude
volatile _Accum max_amplitude=2000;
volatile unsigned int counter_A = 8801;
volatile unsigned int counter_chirp = 8801;
volatile unsigned int counter_silent = 8801;

// waveform amplitude envelope parameters
// rise/fall time envelope 44 kHz samples
volatile unsigned int attack_time=1000, decay_time=1000, sustain_time=6800 ; 
//  0<= current_amplitude < 2048
volatile _Accum current_amplitude ;
// amplitude change per sample during attack and decay
// no change during sustain
volatile _Accum attack_inc, decay_inc ;
// Flag to determine which piano key is pressed
volatile unsigned int p_press = 0;
// Flag to determine which keypad is pressed
int key = 100;
volatile int arcade = 0;
volatile int level = 0;
// === thread structures ============================================
// thread control structs
//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data_A, DAC_data_B ;// output values
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 4 ; // 10 MHz max speed for port expander!!

volatile int adc_0, adc_1, adc_5, adc_11;
//The measured period of the wave
short capture1, last_capture1=0, capture_period=99;
// system 1 second interval tick
int sys_time_100msec;

// 320 * 240  
//              x
//       64  128 192 256
//       |       |   
//       |   |     
//  y        |   |   |
//       |       |
//           |       |
//       |      |    |
// long notes not yet supported

static _Accum Accum_rand, Accum_rand_scaled ;
_Accum speed = float2Accum(10);                 
int begin_time, frame_begin_time, frame_time;                  
int duration; // mSec / frame
int frame_yield_time;

int times_1[15] =     {30, 35,  40, 45,  48, 51, 54,  59,  63, 66,  70, 72, 75, 80, 85};
int positions_1[15] = {64, 128, 64, 192, 256,64, 192, 192, 64, 256, 128,256,192,64, 192};

int times_2[22] =     {30, 35,  40, 45,  48, 51, 54, 59,  63, 66,  70, 72, 75, 80, 85, 87, 93, 100, 105, 108, 112, 116};
int positions_2[22] = {128, 64, 64, 256, 192,128,64, 128, 64, 256, 256,256,64, 128, 192, 64, 128,128, 256, 192, 128, 64};

int times_3[25] =     {30,  35,  40, 45,  48, 51, 54, 59, 63, 66,  70, 72, 75, 80, 85, 87, 93, 100, 105, 108, 112, 116, 124, 128, 132,136,141,146};
int positions_3[25] = {128, 128, 256,128, 256,256,192,64, 128,128, 192,64, 256,64,256,192,64, 128, 128, 192, 192, 64,  128, 64,  64, 128,256,192};

int note_num = 0;
int size_of_notes = 0;
int start = 0;
int end = 0;
int p_count = 0;

typedef struct{
    _Accum x;
    _Accum y;
    int hit;
    int num;
}
note_t;

note_t notes[10]; // maximum notes that we allow on the screen simultaneously

// == Capture 1 ISR ====================================================
// check every capture for consistency
void __ISR(_TIMER_2_VECTOR, ipl2) C1Handler(void)
{
    int junk;

    if(p_press != 0 && counter_A <= 8800){
        counter_A++;  
        DDS_phase += DDS_increment ;
    }
    mT2ClearIntFlag();

    if(p_press != 0){
       DAC_data_A = (int)(current_amplitude*sine_table[DDS_phase>>24]) + 2048 ; // for testing sine_table[DDS_phase>>24]
       // update amplitude envelope 
       if (counter_A < (attack_time + decay_time + sustain_time)){
           current_amplitude = (counter_A <= attack_time)? 
               current_amplitude + attack_inc : 
               (counter_A <= attack_time + sustain_time)? current_amplitude:
                   current_amplitude - decay_inc ;
       }
       else {
           current_amplitude = 0 ;
       }
       DAC_data_B = (int) current_amplitude ;   
    }
    else{
        DAC_data_A = 0;
        DAC_data_B = 0;
    }
    // AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below
    // reset spi mode to avoid conflict with expander
    SPI_Mode16();
    // DAC-A CS low to start transaction
    mPORTBClearBits(BIT_4); // start transaction 
    // write to spi2
    WriteSPI2(DAC_config_chan_A | (DAC_data_A & 0xfff) );
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // MUST read to clear buffer for port expander elsewhere in code
    junk = ReadSPI2(); 
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
    
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
}
static PT_THREAD (protothread_press(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
            // detect presses every 350 mSec
            PT_YIELD_TIME_msec(350) ;
            // read AN0
            adc_0 = ReadADC10(0); // 3-35
            // read AN1 
            adc_1 = ReadADC10(1); // 800-750 
            // read AN5 
            adc_5 = ReadADC10(2); //2-1000
            // read AN11 
            adc_11 = ReadADC10(3);// 410-(460-490)
            
            if(arcade){
                key = -1;
                tft_fillRoundRect(40, 230, 260, 30, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            }
            if(adc_0 > 10){
                p_press = 1;
                current_amplitude = 0;
                counter_A = 0;
                // C4
                if(key==1) Fout = 261.63;
                // G4
                if(key == 2) Fout = 392.00;  
                // D#4/Eb4 	
                if(key == 3) Fout = 311.13;  
                
                if(arcade){ 
                    Fout = 277.18;
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setCursor(50, 230);
                    tft_setTextSize(0.5);
                    if(Accum2int(notes[0].x) == 64){
                        if(Accum2int(notes[0].y) > 180){
                            // C5
                            Fout = 523.25;
                            tft_writeString("PERFECT!");
                            notes[0].hit = 1;
                            p_count++;
                        }
                    }else{
                        tft_writeString("MISSED :<");
                    }
                }
            }           
            if(adc_1 < 780){
                p_press = 2;
                current_amplitude = 0;
                counter_A = 0;
                // E4   
                if(key==1) Fout = 329.63;
                // B4
                if(key == 2) Fout = 493.88;  
                //  G#4/Ab4 	 	
                if(key == 3) Fout = 415.30;  
                
                if(arcade){ 
                    Fout = 277.18;
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setCursor(183, 230);
                    tft_setTextSize(0.5);
                    if(Accum2int(notes[0].x) == 192){
                        if(Accum2int(notes[0].y) > 180){
                            // E5
                            Fout = 659.25;
                            tft_writeString("PERFECT!");
                            notes[0].hit = 1;
                            p_count++;
                        }
                    }else{
                        tft_writeString("MISSED :<");
                    }
                }
            }
            if(adc_5 > 1010){
                p_press = 3;
                current_amplitude = 0;
                counter_A = 0;
                // F4
                if(key==1) Fout = 349.23;
                // C#4/Db4
                if(key == 2)Fout = 277.18;  
                // A#4/Bb4 	 	 	
                if(key == 3) Fout = 466.16;  
                
                if(arcade){ 
                    Fout = 277.18;
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setCursor(245, 230);
                    tft_setTextSize(0.5);
                    if(Accum2int(notes[0].x) == 256){
                        if(Accum2int(notes[0].y) > 180){
                            // F5
                            Fout = 698.46;
                            tft_writeString("PERFECT!");
                            notes[0].hit = 1;
                            p_count++;
                        }
                    }else{
                        tft_writeString("MISSED :<");
                    }
                }
            }
            if(adc_11 > 450){
                p_press = 4;
                current_amplitude = 0;
                counter_A = 0;
                // D4
                if(key == 1) Fout =  293.66;
                // A4
                if(key == 2) Fout = 440.00;  
                // F#4/Gb4 	
                if(key == 3) Fout = 369.99;  
                
                if(arcade){ 
                    Fout = 277.18;
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setCursor(115, 230);
                    tft_setTextSize(0.5);
                    if(Accum2int(notes[0].x) == 128){
                        if(Accum2int(notes[0].y) > 180){
                            // D5
                            Fout = 587.33;
                            tft_writeString("PERFECT!");
                            notes[0].hit = 1;
                            p_count++;
                        }
                    }else{
                        tft_writeString("MISSED :<");
                    }
                }
            }
            DDS_increment = Fout*two32/Fs;
      } // END WHILE(1)
  PT_END(pt);
}

static PT_THREAD (protothread_w1(struct pt *pt))
{
    PT_BEGIN(pt);
    // PT_YIELD_TIME_msec(1000);
    // unsigned int start_time = ReadTimer2(); // used to accurately measure time
    // keep updating the y position of the note
    while(arcade && start){
        // mark the begin time of this frame
        frame_begin_time = PT_GET_TIME();
        
        if (level == 3){
            if (sys_time_100msec == times_1[note_num]){
                // create the corresponding note
                note_t note;
                note.x = positions_1[note_num];
                note.y = 0;
                note.hit = 0;
                note.num = note_num;
                note_num ++;

                // put the note in the array of the current notes on the screen
                notes[size_of_notes] = note;
                size_of_notes ++;
            }
        }
        if (level == 4){
            if (sys_time_100msec == times_2[note_num]){
                // create the corresponding note
                note_t note;
                note.x = positions_2[note_num];
                note.y = 0;
                note.hit = 0;
                note.num = note_num;
                note_num ++;

                // put the note in the array of the current notes on the screen
                notes[size_of_notes] = note;
                size_of_notes ++;
            }
        }
        if (level == 5){
            if (sys_time_100msec == times_3[note_num]){
                // create the corresponding note
                note_t note;
                note.x = positions_3[note_num];
                note.y = 0;
                note.hit = 0;
                note.num = note_num;
                note_num ++;

                // put the note in the array of the current notes on the screen
                notes[size_of_notes] = note;
                size_of_notes ++;
            }
        }
        
        tft_drawLine(48, 0, 48, 220,ILI9340_YELLOW);
        tft_drawLine(80, 0, 80, 220,ILI9340_YELLOW);
        tft_drawLine(112, 0, 112, 220,ILI9340_BLUE);
        tft_drawLine(144, 0, 144, 220,ILI9340_BLUE);
        tft_drawLine(176, 0, 176, 220,ILI9340_CYAN);
        tft_drawLine(208, 0, 208, 220,ILI9340_CYAN);
        tft_drawLine(240, 0, 240, 220,ILI9340_MAGENTA);
        tft_drawLine(272, 0, 272, 220,ILI9340_MAGENTA);
        tft_drawLine(0, 220, 320, 220,ILI9340_RED);
        
        // moving every note on the screen downwards
        int i = 0;
        while (i < size_of_notes){
            // cover up the old note by drawing a black circle
            tft_drawCircle(Accum2int(notes[i].x), Accum2int(notes[i].y), 4, ILI9340_BLACK);
            tft_fillCircle(Accum2int(notes[i].x), Accum2int(notes[i].y), 4, ILI9340_BLACK); //x, y, radius, color

            // Update the position of the note
            notes[i].y += speed;
            
            // if the note is out of the screen, remove it by moving everything after it up
            if (Accum2int(notes[i].y) >= 220){
                if((level==3 && notes[i].num==14) ||(level==4 && notes[i].num==21) || (level==5 && notes[i].num==24)) end=1;
                if(notes[i].hit==0){
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setCursor(Accum2int(notes[i].x)-10, 230);
                    tft_setTextSize(0.5);
                    tft_writeString("MISSED :<");  
                }
                int j = i;
                while (j < size_of_notes){
                    notes[j] = notes[j+1];
                    j ++;
                }
                size_of_notes --;
            }
            // draw the note out
            
            tft_drawCircle(Accum2int(notes[i].x), Accum2int(notes[i].y), 4, ILI9340_GREEN);
            tft_fillCircle(Accum2int(notes[i].x), Accum2int(notes[i].y), 4, ILI9340_GREEN);//x, y, radius, color
        
            i ++;
        }
        // 30 fps => frame time of 33 mSec
        // cycles = ReadTimer2()- start_time;
        duration = PT_GET_TIME() - frame_begin_time;
        frame_yield_time = 30 - duration;
        PT_YIELD_TIME_msec(frame_yield_time);
        frame_time = PT_GET_TIME()- frame_begin_time;
        //PT_YIELD_TIME_msec(100);
    }
    PT_END(pt);
} // animation thread

void clean_up(){
    if(key==2 || key==3){
        // key 1
        tft_fillRoundRect(30, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(69, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(108, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(146, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color    
    }
    if(key==1 || key ==3){
        // key 2
        tft_fillRoundRect(189, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(229, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(269, 175, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(49, 5, 25, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
    }
    if(key==1 || key==2){
        // key 3
        tft_fillRoundRect(89, 5, 25, 23, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(169, 5, 25, 23, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_fillRoundRect(209, 5, 25, 23, 1, ILI9340_BLACK);// x,y,w,h,radius,color 
        tft_fillRoundRect(249, 5, 25, 23, 1, ILI9340_BLACK);// x,y,w,h,radius,color   
    }
}

static PT_THREAD (protothread_w2(struct pt *pt))
{
    PT_BEGIN(pt);
    // PT_YIELD_TIME_msec(1000);
    // unsigned int start_time = ReadTimer2(); // used to accurately measure time
    // keep updating the y position of the note
    while(level==1){
        // mark the begin time of this frame
        frame_begin_time = PT_GET_TIME();
       
        tft_drawRect(20, 25, 280, 200, ILI9340_WHITE);
        tft_drawLine(60, 25, 60, 225,ILI9340_WHITE);
        tft_drawLine(100, 25, 100, 225,ILI9340_WHITE);
        tft_drawLine(140, 25, 140, 225,ILI9340_WHITE);
        tft_drawLine(180, 25, 180, 225,ILI9340_WHITE);
        tft_drawLine(220, 25, 220, 225,ILI9340_WHITE);
        tft_drawLine(260, 25, 260, 225,ILI9340_WHITE);
       
        tft_drawRect(50, 25, 20, 120, ILI9340_WHITE);
        tft_fillRect(50, 25, 20, 120, ILI9340_WHITE);
        tft_drawRect(90, 25, 20, 120, ILI9340_WHITE);
        tft_fillRect(90, 25, 20, 120, ILI9340_WHITE);
        tft_drawRect(170, 25, 20, 120, ILI9340_WHITE);
        tft_fillRect(170, 25, 20, 120, ILI9340_WHITE);
        tft_drawRect(210, 25, 20, 120, ILI9340_WHITE);
        tft_fillRect(210, 25, 20, 120, ILI9340_WHITE);
        tft_drawRect(250, 25, 20, 120, ILI9340_WHITE);
        tft_fillRect(250, 25, 20, 120, ILI9340_WHITE);
       
        tft_fillRoundRect(0, 0, 20, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 0);
        tft_setTextColor(ILI9340_WHITE);  
        tft_setTextSize(2);
        sprintf(buffer,"%d", key);
        tft_writeString(buffer);
       
        if (key == 1){
            clean_up();
            tft_setCursor(30, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("1A");
           
            tft_setCursor(69, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("1B");
           
            tft_setCursor(108, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("1C");
           
            tft_setCursor(146, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("1D");
        }
       
        if (key == 2){
            clean_up();
            tft_setCursor(189, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("2A");
           
            tft_setCursor(229, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("2B");
           
            tft_setCursor(269, 175);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("2C");
           
            tft_setCursor(49, 5);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("2D");
        }
   
        if (key == 3){
            clean_up();
            tft_setCursor(89, 5);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("3A");

            tft_setCursor(169, 5);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("3B");
           
            tft_setCursor(209, 5);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("3C");

            tft_setCursor(249, 5);
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(2);
            tft_writeString("3D");
        }
       
        // 30 fps => frame time of 33 mSec
        // cycles = ReadTimer2()- start_time;
        duration = PT_GET_TIME() - frame_begin_time;
        frame_yield_time = 30 - duration;
        PT_YIELD_TIME_msec(frame_yield_time);
        frame_time = PT_GET_TIME()- frame_begin_time;
        //PT_YIELD_TIME_msec(100);
    }
    PT_END(pt);
} // animation thread



static PT_THREAD (protothread_menu(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        // print every 150 mSec
        PT_YIELD_TIME_msec(150);
        // Piano Mode key 10 = *
        if(key == 10){
            tft_fillScreen(ILI9340_BLACK);
            arcade = 0;
            level = 1;
            key = -1;
            tft_setTextColor(ILI9340_WHITE);  
            tft_setTextSize(1.7);
            tft_setCursor(65, 230);
            tft_writeString("Return to Menu: Press 0");
        }
        // Arcade Mode key 11 = #
        if(key == 11){
            tft_fillScreen(ILI9340_BLACK);
            arcade = 1;
            level = 2;
            key = -1;
        }
        if(level==0){
            tft_fillCircle(85,80, 20, ILI9340_GREEN);//x, y, radius, color 
            tft_setTextSize(2);
            tft_setCursor(60,115);
            tft_setTextColor(ILI9340_WHITE);  
            tft_writeString("Piano");
            tft_setCursor(60,140); 
            tft_writeString("Press *");
            tft_fillCircle(230 ,80, 20, ILI9340_CYAN);//x, y, radius, color
            tft_setCursor(200,115);
            tft_setTextColor(ILI9340_WHITE);  
            tft_writeString("Arcade");
            tft_setCursor(200,140); 
            tft_writeString("Press #");
            
            tft_setCursor(45, 10);
            tft_setTextColor(ILI9340_WHITE);  
            tft_writeString("Choose A Game Mode ;>");
            tft_setCursor(65, 200);
            tft_writeString("By Alga & Xiangyi");
        }
        else if(level==2){
            tft_setTextColor(ILI9340_WHITE); 
            tft_setTextSize(2);
            tft_fillCircle(50, 75, 10, ILI9340_BLUE);//x, y, radius, color 
            tft_setCursor(70,70);
            tft_writeString("Level 1: Press 1");
            tft_fillCircle(50, 115, 10, ILI9340_YELLOW);//x, y, radius, color 
            tft_setCursor(70,110); 
            tft_writeString("Level 2: Press 2");
            tft_fillCircle(50, 155, 10, ILI9340_RED);//x, y, radius, color 
            tft_setCursor(70,150); 
            tft_writeString("Level 3: Press 3");
            tft_setTextColor(ILI9340_WHITE);  
            tft_setCursor(65, 210);
            tft_writeString("Return to Menu: Press 0");
        }
        
        if(arcade){
            if(key==1){
                level = 3;
                tft_fillScreen(ILI9340_BLACK);
                sys_time_100msec = 0;
                note_num = 0;
                size_of_notes = 0;
                start = 1;
            }
            if(key==2){
                level = 4;
                tft_fillScreen(ILI9340_BLACK);
                sys_time_100msec = 0;
                note_num = 0;
                size_of_notes = 0;
                start = 1;
            }
            if(key==3){
                level = 5;
                tft_fillScreen(ILI9340_BLACK);
                sys_time_100msec = 0;
                note_num = 0;
                size_of_notes = 0;
                start = 1;
            }
        }
        if(end && start){
            key = -1;
            level = 6;
            tft_fillScreen(ILI9340_BLACK);
            tft_setTextColor(ILI9340_WHITE); 
            tft_setTextSize(3);
            tft_setCursor(60, 30);
            if(p_count/note_num > 0.4){
                tft_writeString("WELL PLAYED :>");
            }
            else{
                tft_writeString("GAME OVER :<");;
            }
            tft_setTextSize(2);
            tft_fillCircle(50, 95, 10, ILI9340_GREEN);//x, y, radius, color 
            tft_setCursor(70, 90);
            sprintf(buffer,"Perfect Presses: %d", p_count);
            tft_writeString(buffer);
            tft_fillCircle(50, 155, 10, ILI9340_RED);//x, y, radius, color 
            tft_setCursor(70,150); 
            sprintf(buffer,"Missed Presses: %d", note_num-p_count);
            tft_writeString(buffer);
            tft_setCursor(30, 210);
            tft_writeString("Return to Menu: Press 0");
            start = 0;
        }
    }
    PT_END(pt);
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
      while(1) {
            // print every 1000 mSec
            PT_YIELD_TIME_msec(1000) ;
//            tft_fillRoundRect(0, 0, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            tft_setCursor(0, 0);
//            // print the periods
//            sprintf(buffer,"AN0=%d ", adc_0 );
//            tft_writeString(buffer);
//             
//            tft_fillRoundRect(0, 20, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            tft_setCursor(0, 20); 
//            sprintf(buffer,"AN1=%d ", adc_1 );
//            tft_writeString(buffer);
//             
//            tft_fillRoundRect(0, 40, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            tft_setCursor(0, 40);
//            sprintf(buffer,"AN5=%d ", adc_5 );             
//            tft_writeString(buffer);
//                        
//            tft_fillRoundRect(0, 60, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            tft_setCursor(0, 60);
//            sprintf(buffer,"AN11=%d ", adc_11 );             
//            tft_writeString(buffer);
//            
//            tft_fillRoundRect(64, 215, 20, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            tft_setTextColor(ILI9340_WHITE); 
//            tft_setCursor(64, 215);
//            sprintf(buffer,"y=%d ", Accum2int(notes[0].y) );
//            tft_writeString(buffer);
            
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
            // yield time 0.1 second
            PT_YIELD_TIME_msec(100) ;
            sys_time_100msec++ ;
      } // END WHILE(1)

  PT_END(pt);
} 

void printLine2(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 20 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 16, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(2);
    tft_writeString(print_buffer);
}


// === Keypad Thread =============================================
// Port Expander connections:
// y0 -- row 1 -- thru 300 ohm resistor -- avoid short when two buttons pushed
// y1 -- row 2 -- thru 300 ohm resistor
// y2 -- row 3 -- thru 300 ohm resistor
// y3 -- row 4 -- thru 300 ohm resistor
// y4 -- col 1 -- internal pullup resistor -- avoid open circuit input when no button pushed
// y5 -- col 2 -- internal pullup resistor
// y6 -- col 3 -- internal pullup resistor
// y7 -- shift key connection -- internal pullup resistor

static PT_THREAD (protothread_key(struct pt *pt))
{
    PT_BEGIN(pt);
    static int keypad, i, pattern;
    // order is 0 thru 9 then * ==10 and # ==11
    // no press = -1
    // table is decoded to natural digit order (except for * and #)
    // with shift key codes for each key
    // keys 0-9 return the digit number
    // keys 10 and 11 are * adn # respectively
    // Keys 12 to 21 are the shifted digits
    // keys 22 and 23 are shifted * and # respectively
    static int keytable[24]=
    //        0     1      2    3     4     5     6      7    8     9    10-*  11-#
            {0xd7, 0xbe, 0xde, 0xee, 0xbd, 0xdd, 0xed, 0xbb, 0xdb, 0xeb, 0xb7, 0xe7,
    //        s0     s1    s2  s3    s4    s5    s6     s7   s8    s9    s10-* s11-#
             0x57, 0x3e, 0x5e, 0x6e, 0x3d, 0x5d, 0x6d, 0x3b, 0x5b, 0x6b, 0x37, 0x67};
    // bit pattern for each row of the keypad scan -- active LOW
    // bit zero low is first entry
    static char out_table[4] = {0b1110, 0b1101, 0b1011, 0b0111};
    
    // init the port expander
    start_spi2_critical_section;
    initPE();
    // PortY on Expander ports as digital outputs
    mPortYSetPinsOut(BIT_0 | BIT_1 | BIT_2 | BIT_3);    //Set port as output
    // PortY as inputs
    // note that bit 7 will be shift key input, 
    // separate from keypad
    mPortYSetPinsIn(BIT_4 | BIT_5 | BIT_6 | BIT_7);    //Set port as input
    mPortYEnablePullUp(BIT_4 | BIT_5 | BIT_6 | BIT_7);
    
    end_spi2_critical_section ;
    
    // the read-pattern if no button is pulled down by an output
    #define no_button (0x70)

      while(1) {
        // yield time
        PT_YIELD_TIME_msec(200);
    
        for (i=0; i<4; i++) {
            start_spi2_critical_section;
            // scan each rwo active-low
            writePE(GPIOY, out_table[i]);
            //reading the port also reads the outputs
            keypad  = readPE(GPIOY);
            end_spi2_critical_section;
            // was there a keypress?
            if((keypad & no_button) != no_button) { break;}
        }
        
        // search for keycode
        if (keypad > 0){ // then button is pushed
            for (i=0; i<24; i++){
                if (keytable[i]==keypad) break;
            }
            // if invalid, two button push, set to -1
            if (i==24) i=-1;
        }
        else i = -1; // no button pushed
        
        // draw key number
//        if (i>-1 && i<10) sprintf(buffer,"   %x %d", keypad, i);
//        if (i==10 ) sprintf(buffer,"   %x *", keypad);
//        if (i==11 ) sprintf(buffer,"   %x #", keypad);
//        if (i>11 && i<22 ) sprintf(buffer, "   %x shift-%d", keypad, i-12);
//        if (i==22 ) sprintf(buffer,"   %x shift-*", keypad);
//        if (i==23 ) sprintf(buffer,"   %x shift-#", keypad);
//        if (i>-1 && i<12) printLine2(10, buffer, ILI9340_GREEN, ILI9340_BLACK);
//        else if (i>-1) printLine2(10, buffer, ILI9340_RED, ILI9340_BLACK);
        if( -1 < i && i < 12){
            key =i;   
        }
        if(key==0){
            level=0;
            arcade=0;
            start = 0;
            end = 0;
            tft_fillScreen(ILI9340_BLACK);
            key=-1;
        }
        // !!!! NEVER exit while !!!!
      } // END WHILE(1)
  PT_END(pt);
} // keypad thread

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
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | enable scan mode | do 4 samples | use single buf | alternate mode on
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF
	// Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// define setup parameters for OpenADC10
	// set AN4, AN5, AN10, AN11 and as analog inputs
    // AN0=PA0; AN1=PA1; AN5=PB3; AN11=PB13
	#define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA |ENABLE_AN5_ANA | ENABLE_AN11_ANA 

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    
    
	// use ground as neg ref for A   
	// configure to sample AN11 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above
    
    EnableADC10(); // Enable the ADC
    ///////////////////////////////////////////////////////

    // === Config timer ========
    // timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // at 40 MHz PB clock 
    // 40,000,000/Fs = 909 : since timer is zero-based, set to 908
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 908);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    
    // build the sine lookup table
    // scaled to produce values between 0 and 4096
    int i;
    for (i = 0; i < sine_table_size; i++){
          sine_table[i] = (_Accum)(sin((float)i*6.283/(float)sine_table_size));
    }
    // build the amplitude envelope parameters
    // bow parameters range check
	if (attack_time < 1) attack_time = 1;
	if (decay_time < 1) decay_time = 1;
	if (sustain_time < 1) sustain_time = 1;
	// set up increments for calculating bow envelope
	attack_inc = max_amplitude/(_Accum)attack_time ;
	decay_inc = max_amplitude/(_Accum)decay_time ;

    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(1); // Use tft_setRotation(1) for 320x240
    tft_setCursor(0, 0);

    // === config the uart, DMA, vref, timer5 ISR ===========
    PT_setup();
    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
    pt_add(protothread_press, 1);
    pt_add(protothread_key, 1);
    pt_add(protothread_w1, 1);
    pt_add(protothread_menu, 1);
  //  pt_add(protothread_print, 1);
    pt_add(protothread_time, 1);
    pt_add(protothread_w2, 1);

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
