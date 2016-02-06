
/*************************************************************
 *
 * Project
 *   ___    _   _	____    _____   ____     ___
 *  / _ \  | | | | |  _ \  |  _  | |  _ \   / _ \
 * | |_| | | | | | | |_| | | | | | | |_| | | |_| |
 * | ._. | | |_| | | ._ /  | |_| | | ._ /  | ._. |
 * |_| |_|  \___/  |_| \_\ |_____| |_| \_\ |_| |_|
 *
 *	Mark I
 *
 * Author: 		Thomas Richner (mail@trichner.ch)
 * Last change: 18. April 2013
 *
 */

/*
 * PIN-Layout
 *
 * Red: 1.0-1.7
 * Blu: 2.0 - 2.4,3.5-3.7
 * Gre: 4.0 - 4.7
 *
 * SRIN:  2.5,
 * SRCLK: 3.3,
 * SROE:  3.0,
 *
 * SDA 3.1, 12
 * SCL 3.2, 13
 */

/* HEX Values
 * 0 0000
 * 1 0001
 * 2 0010
 * 3 0011
 * 4 0100
 * 5 0101
 * 6 0110
 * 7 0111
 * 8 1000
 * 9 1001
 * A 1010
 * B 1011
 * C 1100
 * D 1101
 * E 1110
 * F 1111
*/

//#include <msp430x22x2.h>
#include <msp430.h>
#include <stdbool.h>


//Debug Options

//#define I2CEN

//==== Clock cycles
#define CLKCYCLES 1000

//==== Color low-side drivers
#define POBLU04 P2OUT // Blue Port, Part 0-4
#define PDBLU04 P2DIR

#define POBLU57 P3OUT // Blue Port, Part 5-7
#define PDBLU57 P3DIR

#define PORED P1OUT // Red Port
#define PDRED P1DIR

#define POGRE P4OUT // Green Port
#define PDGRE P4DIR

//==== Shift Register
#define SRDCTL P3DIR     // Shift register control port
#define SRDDTA P2DIR     // Shift register control port
#define SRCTL P3OUT     // Shift register control port
#define SRDTA P2OUT     // Shift register data port
#define SRIN BIT5       // Shift register serial input
#define SRSTRB BIT4       // Shift register strobe output
#define SRCLK BIT3      // Shift register clock
#define SROE BIT0     // Shift register output enable

/* FRAMEBUFFERSIZE sets the Size of the Ringbuffer, the bigger, the more
 * space for buffering. It has to be a Divider of 256 because in that case
 * modulo can be made with just using a MASK.
 * The FrameBufferMask is the corresponding Modulo Mask
 */
#define FRAMEBUFFERSIZE 2//64 	//space for at least two secs at 30 fps //TODO
#define FBM 0x01//0x3F 			// 0b0011 1111, Framebuffer mask
#define PACKETLENGTH 24

#define MAGIC 0x17//Magic-Packet

/* Represents a Frame with all colors
 * mred: red matrix
 * mgre: green matrix
 * mblu: blue matrix
 */
typedef struct{
  unsigned char mred[8];
  unsigned char mgre[8];
  unsigned char mblu[8];
}Frame;


static const Frame NEO = 	{{0xFF,0x81,0x81,0x99, 0x99,0x81,0x81,0xFF},
				 {0x00,0x7E,0x42,0x5A, 0x5A,0x42,0x7E,0x00},
				 {0x00,0x00,0x3C,0x3C, 0x3C,0x3C,0x00,0x00}};

static const Frame OFF = 	{{0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00},
				 {0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00},
				 {0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00}};

static const Frame ON = 	{{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
				 {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
				 {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}};

static const Frame TROLL = 	{{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
				 {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
				 {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}};

/* Receiving Stuff */

//==== Framebuffer
/* ringbuffer containing maximal FRAMEBUFFERSIZE Frames */
Frame fbuf[FRAMEBUFFERSIZE];
unsigned int fbufw;	/* Position of writing */
unsigned int fbufr;	/* Position of reading */


void init_matrix();	/* Initializes the Matrix*/
void init_i2c();		/* Initializes i2c slave*/
void init_shftreg();	/* Initializes the high-side shift-register driver*/


inline void nextFrame();		/* returns the address of the next Frame */
inline void receiveFrame(char RX);	/* receives a frame packet */

unsigned int cpos;		/* Column position of the Matrix */


//===============    MAIN Function
int main( void )
{
  WDTCTL = WDTPW + WDTHOLD;		// Stop watchdog timer to prevent time out reset

  //----------------DCO Boost, might not be necessary
  BCSCTL1 = CALBC1_16MHZ;//0x87;
  DCOCTL  = CALDCO_16MHZ;//0xE0;

  //---- enable external clock
  BCSCTL1 |= XTS; 				// Use LFXT in High Freq mode
  BCSCTL3 |= LFXT1S_2 + XCAP_0; // 3-16MHz clock, external capacitors


  //---- initialise registers & modules
  //init_i2c();		// set up i2c slave
  init_shftreg();	// initialise high-side, shift-register, matrix driver
  init_matrix();	// initialise ports for low-side matrix drivers

  _EINT();         	// enable global interrupt

  char TXData = 0,RXData;

  //---- Mainloop, trap uC
  while(1);

  return 0;
}

const Frame* STANDBY = &ON;
Frame* active; 			/* address of active Frame */

void nextFrame(){
	if(fbufr==fbufw) 		//no new frames? No need for mod
	  return; 					//take the same again

	active = &(fbuf[fbufr]); //take the new one
	fbufr = ((fbufr+1)&FBM); //Modulo operation for ringbuffer
}

void init_matrix(){
  fbuf[0] = NEO;			//set default first frame
  active = &(fbuf[0]);		//set an active
  cpos = 0;					//init cpos at 0

  PDRED |= 0xFF;				//Red LEDs
  PDGRE |= 0xFF;				//Green LEDs
  PDBLU04 |= 0x1F;				//Blue LEDs
  PDBLU57 |= 0xE0;				//Blue LEDs

  //set all ports off
  //setRed(0x00);
  //setGre(0x00);
  //setBlu(0x00);
	PORED = 0x00;									//all RED off
	POGRE = 0x00;									//all Green off
	POBLU04 &= 0xE0; POBLU57 &= 0x1F;	//all blue off

  //----------------init TimerA
  TACTL = TASSEL_1 + TACLR + ID_3; // ACLK, clear TAR, divide 8  // TASSEL_2 + TACLR + ID_3; //SMCLK

  CCTL0 = CCIE;         	// CCR0/Multiplexer interrupt enabled
  //CCTL1 = CCIE;            // CCR1/Effects interrupt enabled
  //CCTL2 = CCIE;          // CCR2/Clock Interrupt, unused
  CCR0	= CLKCYCLES;
  //CCR1	= 1092; 				//30 fps
  //CCR2	= 0x7FFF;
  //unused, may be used for PWM
  //CCR2 = 0x7FFF; //1sec, Uhrenquarz

  TACTL |= MC_2;          	// Start Timer_A in continuous mode
}

void init_i2c(){
	  P3SEL |= 0x06;                            // Assign I2C pins to USCI_B0
	  UCB0CTL1 |= UCSWRST | UCSSEL_3;           // Enable SW reset + SMCLK
	  UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
	  UCB0I2COA = 0x48;                         // Own Address is 048h
	  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
	  //UCB0I2CIE |= UCSTTIE;                   // Enable STT interrupt
	  //IE2 |= UCB0TXIE;                        // Enable TX interrupt
	  //TXData = 0xff;                          // Used to hold TX data
}

void init_shftreg(){
	SRDCTL |= SRCLK + SROE + SRSTRB;	// set shift reg port directions
	SRDCTL |= SRSTRB; // set strobe high
	SRDDTA |= SRIN;
}

char packetpos = 42;


inline void receiveFrame(char RX){
	if(UCB0STAT&UCSTTIFG){		// start condition?
		 if(((fbufw)+1)!=(fbufr)){ // ready to receive? buffer not full? //TODO what happens if 1st magic gets lost?
			fbufw = ((1+fbufw)&FBM);
		 	packetpos = -1;			//start new record
		 }
		 UCB0STAT &=~UCSTTIFG; 	// reset start flag
	}

	switch(packetpos){
	case -1:
		//CMD Byte
		packetpos++;
		break;
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		// RED
		fbuf[fbufw].mred[packetpos]   = RX;
		packetpos++;
		break;
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		// GRE
		fbuf[fbufw].mgre[packetpos-8] = RX;
		packetpos++;
		break;
	case 16:
	case 17:
	case 18:
	case 19:
	case 20:
	case 21:
	case 22:
	case 23:
		//BLU
		fbuf[fbufw].mblu[packetpos-16]= RX;
		packetpos++;
		break;
	default:	//packetpos out of range?
		// Do nothing for now, packet was too long
		break;
	}
}

//===== Interrupts

//----------------TimerA0 Interrupt, Multiplexer
/* highly time critical matrix multiplexer
 * programmed to use with the micrel shift-register high-side driver
 * and ULN2803 low-side driver
 */

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A0 (void)
{
	CCR0 += CLKCYCLES;
	//-------nextColumn
	unsigned char col = cpos&0x07;		// column
	unsigned char red = active->mred[col];
	unsigned char gre = active->mgre[col];
	unsigned char blu = active->mblu[col];
	unsigned char blu04 =(blu&0x1F);
	unsigned char blu57 =(blu&0xE0);

	blu04 |= (POBLU04 & 0xE0);
	blu57 |= (POBLU04 & 0x1F);


	//==== TIME CRITICAL START
  	//----- disable Column
	SRCTL |= SROE;                // Pull SR Output enable LOW (disable shift reg output)

	SRCTL |= SRCLK; 				// rising edge, shift one bit

	//------- load colors

	//==== Setting Colors inline functions
	PORED = red;					//change RED
	POGRE = gre;					//change Green
	POBLU04 &= 0xE0;
	POBLU04 |= blu04;
	POBLU57 &= 0x1f;
	POBLU57 |= blu57;				//change Blue



	//------ enable Column
	SRCTL &= ~SROE;               // pull SR Output enable High (enable)

	//==== TIME CRITICAL STOP

	cpos++;

	SRDTA &= ~SRIN;       // no SR data in
	//-------nextColumn
	if(!(cpos&0x07)){        // mod 8 ?
		SRDTA |= SRIN;       // shift one bit into the SR
		if(!(cpos&0x3F)){ // change frame every 8x8 multiplexes %64
			nextFrame();
		}
	}

	//------- shift one
	SRCTL &= ~SRCLK;
}

//==== I2C Interrupts
// USCI_B0 Data ISR
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  UCB0TXBUF = TXData;                       // TX data
  //__bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
}

// USCI_B0 State ISR
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
  UCB0STAT &= ~UCSTTIFG;                    // Clear start condition int flag
  TXData++;                                 // Increment data
}
