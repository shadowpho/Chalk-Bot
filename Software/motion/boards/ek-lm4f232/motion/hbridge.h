// hbridge.h - Definition file for interface with the ChalkBot hbridge modules.
// Alex Suchko for ChalkBot
// November 10,2011

// Definitions

#define HBR_LEFT          (1)         // hash code to refer to left hbridge
#define HBR_RIGHT         (2)         // hash code to refer to right hbridge

// Function Prototypes

void hbr_init(void);                  // setup pins and PWM hardware to talk to hbridge

// EOF