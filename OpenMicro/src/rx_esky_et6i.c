#include "rx_esky_et6i.h"

#include "xn297.h"

typedef uint8_t u8;


#ifdef USE_ET6I_TX

extern u8 currentchannel;
extern int rxdata[PAYLOADSIZE];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];
extern float rx[4];



#ifdef ET_PID_CH

extern int next_pid_term(void);
extern int next_pid_axis(void);

extern float pidkp[PIDNUMBER];
extern float pidki[PIDNUMBER];
extern float pidkd[PIDNUMBER];

extern uint8 is_combined_tuning;

extern int current_pid_axis;
extern int current_pid_term;
extern int pid_gestures_used;


float last_p;
float last_d;
float last_i;
#ifdef COMBINE_PITCH_ROLL_PID_TUNING
float last_p2;
float last_d2;
float last_i2;
#endif /* COMBINE_PITCH_ROLL_PID_TUNING */
float rx6_zero = 0.0f;
float rx5_zero = 0.0f;
#endif



u8      first_data_packet    = 1;
u8      throttle_block       = 1;
u8      channel_code         = 0;
uint8_t rxaddress[4]         = {0x00, 0x00, 0x00, 0x00};
uint8_t chans[RX_CHAN_COUNT] = {50, 50, 50, 50, 50, 50}; // channels for data, selected on runtime, binding channel 50.

float       rx_tmp[2];
uint16_t    et_rx[7];
uint8_t     end_bytes[RX_CHAN_COUNT];



void set_channels() {
    uint32_t channel_ord = channel_code-10;
    u8 channel1 = 10 + (u8) ((37 + channel_ord*5) % 74);
    u8 channel2 = 10 + (u8) ((channel_ord*5) % 74);
    chans[0] = channel1;
    chans[1] = channel1;
    chans[2] = channel1;
    chans[3] = channel2;
    chans[4] = channel2;
    chans[5] = channel2;
    
    end_bytes[0] = 6;
    end_bytes[1] = channel1*2;
    end_bytes[2] = channel1*2;
    end_bytes[3] = 6;
    end_bytes[4] = channel2*2;
    end_bytes[5] = channel2*2;
}



/** Check if bind packet is valid
@return 1 when packet is valid, 0 when invalid
*/
u8 decode_bind_packet()
{
    if(0x18 != rxdata[4] && 0x29 != rxdata[5])
        return 0;

    // get TX address
    rxaddress[0]=rxdata[2];
    rxaddress[1]=rxdata[1];
    rxaddress[2]=rxdata[0];
    rxaddress[3]=0xBB;

    channel_code=rxdata[3];
        
    xn_writereg( SETUP_AW , AW_4B ); // address size (4 bytes)
    xn_writeregmulti(RX_ADDR_P0, rxaddress, 4);

    set_channels();

    return 1;
}





float esky_range(float f){
    if(f < -1.0f)
        return -1.0f;
    
    if(f > 1.0f)
        return 1.0f;
    
    return f;
}



#ifdef ET_PID_CH
void temp_save_pid()
{
    last_p = pidkp[current_pid_axis];
    last_d = pidkd[current_pid_axis];
    last_i = pidki[current_pid_axis];
    
    #ifdef COMBINE_PITCH_ROLL_PID_TUNING
    last_p2 = pidkp[current_pid_axis+1];
    last_d2 = pidkd[current_pid_axis+1];
    last_i2 = pidki[current_pid_axis+1];
    #endif /* COMBINE_PITCH_ROLL_PID_TUNING */
    
    rx6_zero = rx_tmp[ET_RX6];
    rx5_zero = rx_tmp[ET_RX5];
}

float pid_check(float f){
    
    if(f<0)
        return 0.0f;
    
    return f;
}

#endif /* ET_PID_CH */



/**
@return 0 when failed to decode packet
*/
u8 decode_syma_x5c()
{
    if(end_bytes[currentchannel]!=rxdata[12])
        return 0;

    // get millisecond values from dat packet 
    et_rx[ROLL]     = rxdata[0]<<8 | rxdata[1];
    et_rx[PITCH]    = rxdata[2]<<8 | rxdata[3];
    et_rx[YAW]      = rxdata[6]<<8 | rxdata[7]; 
    et_rx[THROTTLE] = rxdata[4]<<8 | rxdata[5];
    et_rx[ET_CH5]   = rxdata[8]<<8 | rxdata[9];
    et_rx[ET_CH6]   = rxdata[10]<<8 | rxdata[11];
    
    // scale stick input to -1 ... 1
    if(et_rx[ROLL] < ET_CP)
        rx[ROLL] = (float)((ET_CP-et_rx[ROLL]))/(float)(ET_CP-ET_ROLLMIN);
    else
        rx[ROLL] = (float)(-(et_rx[ROLL]-ET_CP))/(float)(ET_ROLLMAX-ET_CP);
    
    if(et_rx[PITCH] < ET_CP)
        rx[PITCH] = (float)((ET_CP-et_rx[PITCH]))/(float)(ET_CP-ET_PITCHMIN);
    else
        rx[PITCH] = (float)(-(et_rx[PITCH]-ET_CP))/(float)(ET_PITCHMAX-ET_CP);
    
    if(et_rx[YAW] < ET_CP)
        rx[YAW] = (float)((ET_CP-et_rx[YAW]))/(float)(ET_CP-ET_YAWMIN);
    else
        rx[YAW] = (float)(-(et_rx[YAW]-ET_CP))/(float)(ET_YAWMAX-ET_CP);
    
    rx[THROTTLE] = 1.0f-((float)(et_rx[THROTTLE]-ET_THRTLMIN)/(float)(ET_THRTLMAX-ET_THRTLMIN));

    for(u8 i=0; i<3; ++i){
        rx[i] = esky_range( rx[i] );
    }
    
    // channel 5 + gyro SW
    et_rx[ET_GYROSW]=(et_rx[ET_CH5]<ET_CP)?1:0;
    float f_ch5=0;
    if(et_rx[ET_GYROSW])
    {
        if(et_rx[ET_CH5] < ET_CH5MID1)
        {
            f_ch5 =     (float)(et_rx[ET_CH5]-ET_CH5MIN1)/(float)(2*(ET_CH5MID1-ET_CH5MIN1));
        }
        else
        {
            f_ch5 =0.5f+(float)(et_rx[ET_CH5]-ET_CH5MID1)/(float)(2*(ET_CH5MAX1-ET_CH5MID1));
        }
    } else {
        if(et_rx[ET_CH5] < ET_CH5MID2)
        {
            f_ch5 =     (float)(et_rx[ET_CH5]-ET_CH5MIN2)/(float)(2*(ET_CH5MID2-ET_CH5MIN2));
        }
        else
        {
            f_ch5 =0.5f+(float)(et_rx[ET_CH5]-ET_CH5MID2)/(float)(2*(ET_CH5MAX3-ET_CH5MID2));
        }
    }
    rx_tmp[ET_RX5] = esky_range(-1.0f+f_ch5*2.0f);
    
    if(!aux[MODE_CHANNEL])
    {
        aux[CH_EXPERT] = et_rx[ET_GYROSW];
        aux[PID_ACTIVE_CH] = 0;
    }
    else 
    {
        aux[CH_EXPERT] = 0;
        aux[PID_ACTIVE_CH] = et_rx[ET_GYROSW];
    }
    
    // channel 6
    rx_tmp[ET_RX6] = esky_range(1.0f-(((float)(et_rx[ET_CH6]-ET_CH6MIN)/(float)(ET_CH6MAX-ET_CH6MIN))*2.0f));
    
    // arming protection while powering on transmitter
    if(first_data_packet)
    {
        if(0.1f<rx[THROTTLE])
        {
            // throttle over 10% in first received data packet, block throttle
            throttle_block = 1;
        }
        else
        {
            throttle_block = 0;
        }
        first_data_packet = 0;
        
        #ifdef ET_PID_CH
        if(et_rx[ET_GYROSW]){
            temp_save_pid();
        }
        #endif /* ET_PID_CH */
    }
    
    if(!throttle_block)
    {
        // block throttle until it has been below 10%
        rx[THROTTLE]    = rx[THROTTLE];
    } else {
        if(0.1f>rx[THROTTLE])
            throttle_block = 0;
        else
            rx[THROTTLE] = 0.0f;
    }
    
    #ifdef CH5_AS_SWITCH
        aux[MULTI_CHAN_5] =(rx_tmp[ET_RX5] < 0.0f )? 0 : 1;
    #endif /* CH5_AS_SWITCH */
    
    #ifdef CH6_AS_SWITCH
        aux[MULTI_CHAN_6] =(et_rx[ET_CH6] < ET_CP )? 0 : 1;
    #endif /* CH6_AS_SWITCH */
    
    
    
    #ifdef ET_PID_CH
    if(aux[MODE_CHANNEL])
    {
        if(0==current_pid_term){
            next_pid_term();
        }
        
        #ifdef COMBINE_PITCH_ROLL_PID_TUNING
        if(is_combined_tuning && PITCH==current_pid_axis){
            next_pid_axis();
            next_pid_axis();
        }
        #endif /* COMBINE_PITCH_ROLL_PID_TUNING */
        
        
        if(aux[PID_ACTIVE_CH])
        { 
            // POS 1
            
            if(!lastaux[PID_ACTIVE_CH]) temp_save_pid();
                
            pid_gestures_used = 1;
                
            rx_tmp[ET_RX5]-=rx5_zero;
            rx_tmp[ET_RX6]-=rx6_zero;
            
            // scale knobs a little bit
            rx_tmp[ET_RX5] *= PID_CH_SCALE;
            rx_tmp[ET_RX6] *= PID_CH_SCALE;
                
            pidkp[current_pid_axis] = pid_check(last_p + rx_tmp[ET_RX6]); // left knob
            
            if(2==current_pid_term)
            {
                pidkd[current_pid_axis] = pid_check(last_d + rx_tmp[ET_RX5]); // right knob
            }
            else
            {
                pidki[current_pid_axis] = pid_check(last_i + rx_tmp[ET_RX5]); // right knob
            }

            #ifdef COMBINE_PITCH_ROLL_PID_TUNING
            
            if(ROLL==current_pid_axis && is_combined_tuning)
            {
                pidkp[PITCH]= pid_check(last_p2 + rx_tmp[ET_RX6]);
                        
                if(2==current_pid_term)
                {
                    pidkd[PITCH] = pid_check(last_d2 + rx_tmp[ET_RX5]); // right knob
                }
                else
                {
                    pidki[PITCH] = pid_check(last_i2 + rx_tmp[ET_RX5]); // right knob
                }
            }
            #endif /* COMBINE_PITCH_ROLL_PID_TUNING */
            
        }
    }
    #endif /* ET_PID_CH */

    // update change flags
    for ( u8 i = 0 ; i < AUXNUMBER - 2 ; i++)
    {
        auxchange[i] = 0;
        if ( lastaux[i] != aux[i] ) auxchange[i] = 1;
        lastaux[i] = aux[i];
    }

    return 1;
}


#endif
