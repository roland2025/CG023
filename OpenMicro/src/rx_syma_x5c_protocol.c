/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"
#include "drv_time.h"
#include <stdio.h>
#include "config.h"
#include "defines.h"

#include "rx_bayang.h"
#include "util.h"


#ifdef RX_SYMA_X5C_PROTOCOL

// compatibility with older version hardware.h
#if ( defined RADIO_XN297 || defined RADIO_XN297L)
    #error PROTOCOL needs NRF24 compatible radio
#endif /*  RX_SYMA_X5C_PROTOCOL */

// Bit vector from bit position
#define BV(bit) (1 << bit)

typedef uint8_t u8;

extern float rx[4];
// the last 2 are always on and off respectively
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];

#ifdef USE_STOCK_TX
	// Use Syma X5SC stock transmitter
	int rxaddress[5] =  {0xab,0xac,0xad,0xae,0xaf};
	#define TX_BITRATE BITRATE_250K
	#define PAYLOADSIZE 10
	#define RX_CHAN_COUNT 4
	uint8_t chans[RX_CHAN_COUNT] = {0x09, 0x30, 0x40, 0x20}; // channels for binding
	//uint8_t data_chans[12] = {40, 43,52,55, 56, 59, 68,71,72,75,84,87}; // channels, where data is received
#else
	// Syma X5C with Devo transmitter
	static int rxaddress[5] =  {0x6D,0x6A,0x73,0x73,0x73};
	#define TX_BITRATE BITRATE_2M
	#define PAYLOADSIZE 16
uint8_t chans[15] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24, 
													 0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};
	#define RX_CHAN_COUNT 16
#endif



int bkfound = 0;


void rx_init()
{
	if (xn_readreg(0x07) & 0x80)
	{
			xn_activate(0x53);
	}

	xn_writerxaddress( rxaddress);

	uint8_t rf_setup=0;
	uint8_t bitrate=TX_BITRATE;

	rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);

	xn_writereg( CONFIG, BV(PWR_DOWN) );

	xn_writereg( EN_AA , 0 );	// aa disabled
	
	#ifdef USE_STOCK_TX
	xn_writereg( EN_RXADDR , 0x3F );
	#else
	xn_writereg( EN_RXADDR , 1 ); // pipe 0 only
	#endif /*  USE_STOCK_TX */

	xn_writereg( RF_SETUP , rf_setup);  // 
	xn_writereg( RX_PW_P0 , PAYLOADSIZE ); // payload size
	xn_writereg( RX_PW_P1 , PAYLOADSIZE ); // payload size
	xn_writereg( RX_PW_P2 , PAYLOADSIZE ); // payload size
	xn_writereg( RX_PW_P3 , PAYLOADSIZE ); // payload size
	xn_writereg( RX_PW_P4 , PAYLOADSIZE ); // payload size
	xn_writereg( RX_PW_P5 , PAYLOADSIZE ); // payload size
	xn_writereg( SETUP_RETR , 0 ); // no retransmissions ( redundant?)
	xn_writereg( SETUP_AW , AW_5B ); // address size (5 bytes)
	xn_command( FLUSH_RX);
	xn_writereg( RF_CH , 0x08 );  // bind  channel

	xn_activate(0x53); // BK24xx bank switch

	if ( xn_readreg(0x07) & 0x80 ) 
	{
		bkfound = 1;
		
		#ifdef USE_STOCK_TX
		if(0x63==xn_readreg(0x08))
		{
			// BK2425 chip detected (chip ID is the same as BK2423)
			// https://www.interoberlin.de/downloads/syma-x8c/bk2425.pdf
			//  For register 0 to 8 at bank 1, the byte order is inversed that the MSB byte is R/W before LSB byte. 
			xn_writeregmulti(0x00, (uint8_t *) "\x40\x4B\x01\xE2", 4);
			xn_writeregmulti(0x01, (uint8_t *) "\xC0\x4B\x00\x00", 4);
			xn_writeregmulti(0x02, (uint8_t *) "\xD0\xFC\x8C\x02", 4);
			xn_writeregmulti(0x03, (uint8_t *) "\x99\x00\x39\x21", 4);
			xn_writeregmulti(0x04, (uint8_t *) "\xF9\x96\x82\x1B", 4); // ????
			xn_writeregmulti(0x05, (uint8_t *) "\x24\x06\x7F\xA6", 4); // disable RSSI
			xn_writeregmulti(0x06, (uint8_t *) "\x00\x00\x00\x00", 4); // reserved
			xn_writeregmulti(0x07, (uint8_t *) "\x00\x00\x00\x00", 4); // reserved
			// Byte order LSB to MSB
			xn_writeregmulti(0x09, (uint8_t *) "\x00\x00\x00\x00", 4); // reserved
			xn_writeregmulti(0x0A, (uint8_t *) "\x00\x00\x00\x00", 4); // reserved
			xn_writeregmulti(0x0B, (uint8_t *) "\x00\x00\x00\x00", 4); // reserved
			xn_writeregmulti(0x0C, (uint8_t *) "\x00\x12\x73\x00", 4); // compatible mode: dynamic; PLL settling: 120us
			xn_writeregmulti(0x0D, (uint8_t *) "\x36\xB4\x80\x00", 4);
			xn_writeregmulti(0x0E, (uint8_t *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);	
		}
		#endif
	} 
    
	xn_activate(0x53); // switch bank back
	
	uint8_t config=0;
	config=xn_readreg(CONFIG);
	config|=BV(PWR_UP) | BV(CRCO) | BV(EN_CRC) | BV(MASK_MAX_RT); // power up, enable crc
	
	xn_command( FLUSH_RX);
	
	xn_writereg( CONFIG, config );

#ifdef RADIO_CHECK
	int temp = xn_readreg( 0x0f); // rx address pipe 5	
	// should be 0xc6
	extern void failloop( int);
	if ( temp != 0xc6) failloop(3);
#endif	 /* RADIO_CHECK */
}



static int checkpacket()
{
	spi_cson();
	int status = spi_sendzerorecvbyte();
	spi_csoff();
	if( (status & B00001110) != B00001110 )
	{
		// rx fifo not empty		
		return 2;	
	}
	
  return 0;
}


int rxdata[PAYLOADSIZE];

int rxmode = RXMODE_BIND;


float syma_scale( int input)
{
  float result = 0;
  if ( input >= 0x80 )
		{
			result = (input - 0x80)  * 0.007874f ;
		}
		else  result = - input * 0.007874f ; 
return result;
}


#ifdef USE_STOCK_TX
	
//#define TRIM_C 0.0032258064516129f
#define TRIM_C 0.01f

float syma_trim_scale(u8 input){
	float res;
	
	if(input<0x20)
		res=(float) input * -TRIM_C;
	else
		res=(float) (input&0x1F) * TRIM_C;
	
	return res;
}

float syma_trim_pitch(u8 input){
	float res;
//	
//	5F - 95 
// - scale
//	40 - 64 
//	40 & 60 = 0
//	60  - 96
// + scale
//	7F  - 127
//	
	if(input<0x60)
		res=(float) (input&0x1F) * -TRIM_C;
	else
		res=(float) (input&0x1F) * TRIM_C;
	
	return res;
}


void set_channels() {
  static const u8 start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
  static const u8 start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
  static const u8 start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};

  u8 laddress = rxaddress[4] & 0x1f;
  u8 i;

  u8 num_rf_channels = 4;

	if(laddress==0x01){
      chans[0] = 0x54;
      chans[1] = 0x57;
      chans[2] = 0x28;
      chans[3] = 0x44;
	} else if (laddress < 0x10) {
    if (laddress == 6) laddress = 7;
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_1[i] + laddress;
    }
  } else if (laddress < 0x18) {
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_2[i] + (laddress & 0x07);
    }
    if (laddress == 0x16) {
      chans[0] += 1;
      chans[1] += 1;
    }
  } else if (laddress < 0x1e) {
    for(i=0; i < num_rf_channels; i++) {
      chans[i] = start_chans_3[i] + (laddress & 0x07);
    }
  } else if (laddress == 0x1e) {
      chans[0] = 0x38;
      chans[1] = 0x18;
      chans[2] = 0x41;
      chans[3] = 0x21;
  } else {
      chans[0] = 0x39;
      chans[1] = 0x19;
      chans[2] = 0x41;
      chans[3] = 0x21;
  }
}



u8 calc_checksum()
{
	u8 sum=rxdata[0];
	for(u8 i=1; i<9; ++i)
	{
		sum^=rxdata[i]; 
	}
	
	sum+=0x55;
	return sum;
}

/** Check if bind packet is valid
@return 1 when packet is valid, 0 when invalid
*/
u8 decode_bind_packet()
{
	if(0xB1 != rxdata[8])
		return 0;
	
	if(0xAA != rxdata[5] && 0xAA != rxdata[6])
		return 0;
	
	if(calc_checksum()!=rxdata[9])
		return 0;
	
	// get TX address
	rxaddress[4]=rxdata[0];
	rxaddress[3]=rxdata[1];
	rxaddress[2]=rxdata[2];
	rxaddress[1]=rxdata[3];
	rxaddress[0]=rxdata[4];
	
	xn_writerxaddress( rxaddress);
	
	set_channels();
	
	return 1;
}



/**
@return 0 when failed to decode packet
*/
u8 decode_syma_x5c()
{
	
	if(calc_checksum()!=rxdata[9])
		return 0;
			
	rx[THROTTLE] 	= (float) rxdata[0] * 0.00393f;
	rx[YAW] 			= syma_scale( rxdata[2] ) + syma_trim_scale( rxdata[6] & 0x3F );
	rx[PITCH] 	  =-syma_scale( rxdata[1] ) - syma_trim_pitch( rxdata[5] & 0x7F );
	rx[ROLL] 			= syma_scale( rxdata[3] ) + syma_trim_scale( rxdata[7] & 0x3F );
	
	aux[CH_VID]	 	=( rxdata[4] & 0x80 )? 1 : 0;//video
	aux[CH_PIC] 	=( rxdata[4] & 0x40 )? 1 : 0 ;//pic
	aux[CH_FLIP] 	=( rxdata[6] & 0x40 )? 1 : 0 ;//flip
	aux[CH_EXPERT]=( rxdata[5] & 0x80 )? 1 : 0 ;// L / H speed mode
	aux[CH_HEADFREE]=( rxdata[7] & 0x80 )? 0 : 1 ;//acro
	
	//aux[???] = ( rxdata[6] & 0x80 )? 1 : 0 ; //flip switch, while holding down 3 sec
	
	// update change flags
	for ( int i = 0 ; i < AUXNUMBER - 2 ; i++)
	{
		auxchange[i] = 0;
		if ( lastaux[i] != aux[i] ) auxchange[i] = 1;
		lastaux[i] = aux[i];
	}
	
	
	return 1;
}

#else


/** Check if bind packet is valid
@return 1 when packet is valid, 0 when invalid
*/
u8 decode_bind_packet()
{
	return (rxdata[7] == 0xae && rxdata[8] == 0xa9);
}

int decode_syma_x5c()
{
		uint8_t sum;
    if ( decode_bind_packet() )
    {
       sum = 0;
       for ( int i = 0 ; i < 15 ; i++) 
        sum+= rxdata[i];
        
       if ( sum == rxdata[15] )
       {
        // checksum passes
        
        // throttle
        rx[3] = (float) rxdata[0] * 0.00393f;
           
        rx[2] = syma_scale( rxdata[1] );
        rx[1] = syma_scale( rxdata[2] );
        rx[0] = syma_scale( rxdata[3] );
        
           aux[CH_VID] = ( rxdata[14] & 0x10 )? 1 : 0;//video
        aux[CH_PIC] = ( rxdata[14] & 0x08 )? 1 : 0 ;//pic
        aux[CH_FLIP] = ( rxdata[14] & 0x01 )? 1 : 0 ;//flip
        aux[CH_EXPERT] = ( rxdata[14] & 0x04 )? 1 : 0 ;//expert
           
        // update change flags
        for ( int i = 0 ; i < AUXNUMBER - 2 ; i++)
		{
			auxchange[i] = 0;
			if ( lastaux[i] != aux[i] ) auxchange[i] = 1;
			lastaux[i] = aux[i];
		}
           
        return 1;  
       }
       else return 0;
       
    }
    // data 7 or 8 different
    return 0;
}

#endif /*  USE_STOCK_TX */


//
static unsigned long failsafetime;

int failsafe = 0;



#ifdef RXDEBUG	
struct rxdebug rxdebug;

int packetrx;
unsigned long lastrxtime;
unsigned long secondtimer;
unsigned int rx_chan_count[RX_CHAN_COUNT];
#warning "RX debug enabled"
#endif /* RXDEBUG */


#include <stdlib.h>

int currentchannel = 0;
unsigned long lastrx;
					
void checkrx( void)
{
		if ( checkpacket() ) 
		{ 
			xn_readpayload( rxdata , PAYLOADSIZE);
			if ( RXMODE_BIND == rxmode )
			{	// rx startup , bind mode	
				if (decode_bind_packet()) 
				{// bind packet received			
					rxmode = RXMODE_NORMAL;				

				  xn_writereg( RF_CH, chans[0] ); // Set channel frequency	
                    
				
					#ifdef SERIAL_INFO	
					printf( " BIND \n");
					#endif /* SERIAL_INFO */
				}
			}
			else
			{	// normal mode	
				#ifdef RXDEBUG	
				rxdebug.packettime = gettime() - lastrxtime;
				lastrxtime = gettime();
				#endif /* RXDEBUG */
				
				int pass = decode_syma_x5c();
			 
				if ( pass )
				{ 	
					failsafetime = gettime(); 
					failsafe = 0;
                    #ifdef RXDEBUG	
                    packetrx++;
                    if ( currentchannel < sizeof ( rx_chan_count )) rx_chan_count[currentchannel]++;
										#endif
					lastrx = failsafetime;
                    currentchannel++;
                    currentchannel%=sizeof(chans);
                    xn_writereg( RF_CH, chans[currentchannel] ); // Set channel frequency

				}	
				else
				{
				#ifdef RXDEBUG	
				rxdebug.failcount++;
				#endif /* RXDEBUG */
				}
			
			}// end normal rx mode
				
		}// end packet received
        		
		unsigned long time = gettime();

if ( time - lastrx > 8000* (sizeof(chans) - 2) )
{
    currentchannel++;
    currentchannel%=sizeof(chans);
    xn_writereg( RF_CH, chans[currentchannel] ); // Set channel frequency

    lastrx = time;
}
		
		if( time - failsafetime > FAILSAFETIME )
		{//  failsafe
		  failsafe = 1;
			rx[0] = 0;
			rx[1] = 0;
			rx[2] = 0;
			rx[3] = 0;
		}
#ifdef RXDEBUG	
		// packets per second counter
			if ( time - secondtimer  > 1000000)
			{
				rxdebug.packetpersecond = packetrx;
				packetrx = 0;
				secondtimer = gettime();
			}
#endif /* RXDEBUG */

}
	


// end cg023 proto
#endif






