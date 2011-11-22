// encoder.h - Definition file for Interface for 2ch of Quadrature Encoder Interface (QEI) to the encoders.
// Alex Suchko for ChalkBot
// November 20,2011

// Definitions

#define ENC_1           (1)           // hash definitions for accessing enc 1 through APIs
#define ENC_2           (2)           // hash definitions for accessing enc 2 through APIs

// Function Prototypes

void enc_init(void);                  // setup pins and qei hardware
void enc_2_int_init(void);            // setup and enable interrupts for qei
void enc_dir_reverse(unsigned char encoder, unsigned char reverse);
                                      // specifies option to reverse encoder positive direction
signed long enc_pos_get(unsigned char encoder);
                                      // gets current encoder position in pulses
void enc_pos_set(unsigned char encoder, signed long position);
                                      // sets current encoder position in pulses
signed long enc_vel_get(unsigned char encoder);
                                      // gets current encoder velocity (see important note in "encoder.c")

// EOF