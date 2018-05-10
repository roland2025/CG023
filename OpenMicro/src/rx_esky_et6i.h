#include "config.h"
#include "stdint.h"

#ifndef RX_ESKY_ET6I_H
#define RX_ESKY_ET6I_H

typedef uint8_t u8;

#ifdef USE_ET6I_TX

// Use CH5 & CH6 to modify PID values
#define ET_PID_CH
#define PID_ACTIVE_CH CH_HEADFREE
#define PID_CH_SCALE 0.3f
#define MODE_CHANNEL CH_AUX2

#ifndef ET_PID_CH

// uncomment to use CH5 knob as aux switch MULTI_CHAN_5
//#define CH5_AS_SWITCH

// uncomment to use HOV.PIT knob as aux switch MULTI_CHAN_6
//#define CH6_AS_SWITCH

#endif /* ET_PID_CH */

/* stick input scaling values */

// ROLL
#define ET_ROLLMIN  1126
#define ET_ROLLMAX  1846
// PITCH
#define ET_PITCHMIN 1163
#define ET_PITCHMAX 1837
// YAW
#define ET_YAWMIN   1137
#define ET_YAWMAX   1867
// THROTTLE
#define ET_THRTLMIN 1170
#define ET_THRTLMAX 1843
// CH5
#define ET_CH5MIN1  992
#define ET_CH5MID1  1242
#define ET_CH5MAX1  1461

#define ET_CH5MIN2  1503
#define ET_CH5MID2  1753
#define ET_CH5MAX3  2012

// CH6
#define ET_CH6MIN   991
#define ET_CH6MAX   2013



/* Do not modify defines after this line */
#define ET_CP       1500 // channel value middlepoint (50%)

#define ET_CH5 4
#define ET_CH6 5
#define ET_GYROSW 6

#define ET_RX5 0
#define ET_RX6 1



#define TX_BITRATE BITRATE_1M
#define PAYLOADSIZE 13
#define RX_CHAN_COUNT 6 // 3 repeats on 2 data channels


extern uint8_t chans[RX_CHAN_COUNT];
extern uint8_t rxaddress[4];


u8 decode_bind_packet(void);


u8 decode_syma_x5c(void);

#endif /* USE_ET6I_TX */

#endif /* RX_ESKY_ET6I_H */
