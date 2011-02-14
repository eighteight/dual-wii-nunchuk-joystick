/*
 * Nunchuk - USB Converter
 * by
 * dSanchez
 * v1.3
 * Hacked from kero905's Wii extension controller to USB Hid project v 0.01a
 * In turn based on Objective Development's V-USB
 * License: GNU General Public License Version 2 (GPL)
 */
#include <math.h>
#include <avr/io.h>		    // include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv/usbdrv.h"

// Get declaration for f(int i, char c, float x)
#include "includes/global.h"		// include our global settings
#include "twi/twi.h"		// include i2c support

#define TRUE 1
#define FALSE 0


/* ------------------------------------------------------------------------- */
/* ----------------------------- Controller stuff -------------------------- */
/* ------------------------------------------------------------------------- */

unsigned char nunchuck_buf[7];
unsigned char nunchuck_cmd_buf[3];
unsigned char i2c_return;
unsigned char status[12];

#define switchpin PORTD7

unsigned char offset_x = 115;
unsigned char offset_y = 122;
int control1 = FALSE,control2=FALSE;
// Low-pass filter variables
float xlow[3]={0,0,0},ylow[3]={0,0,0},zlow[3]={0,0,0};
float xraw[3]={0,0,0},yraw[3]={0,0,0},zraw[3]={0,0,0};
float x2low[3]={0,0,0},y2low[3]={0,0,0},z2low[3]={0,0,0};
float x2raw[3]={0,0,0},y2raw[3]={0,0,0},z2raw[3]={0,0,0};

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

//  Controls are currently assigned as follows:

// NUNCHUK   CONTROL       USB USAGE
//	  1       PITCH        RY POINTER
// 	  1       ROLL         Z  POINTER
//    1       X-JOY        X  POINTER
//    1       Y-JOY        Y  POINTER
//    1       C-BUT			BUTTON 2
//    1       Z-BUT			BUTTON 1
//	  2       PITCH        THROTTLE
// 	  2       ROLL         DIAL
//    2       X-JOY        RX POINTER
//    2       Y-JOY        RZ POINTER
//    2       C-BUT			BUTTON 4
//    2       Z-BUT			BUTTON 3


PROGMEM char usbHidReportDescriptor[89] = { // USB report descriptor, size must match usbconfig.h
		0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
		0x09, 0x05,                    // USAGE (Joystick)
		0xa1, 0x01,                    // COLLECTION (Application)

		0x05, 0x01,							// USAGE_PAGE (Generic Desktop)
		0x09, 0x01,                    //   USAGE (Pointer)
		0xa1, 0x00,                    //   COLLECTION (Physical)
		0x09, 0x30,                    //     USAGE (X)
		0x09, 0x31,                    //     USAGE (Y)
		0x09, 0x32,                    //     USAGE (Z)
		0x09, 0x33,					   //     USAGE (Rx)
		0x09, 0x34,					   //     USAGE (Ry)
		0x09, 0x35,						//	  USAGE (Rz)

		0x09, 0x37,						// USAGE (dial)
		0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		0x26, 0xff, 0x00,           //     LOGICAL_MAXIMUM (255)
		0x75, 0x08,                    //     REPORT_SIZE (8)
		0x95, 0x07,                    //     REPORT_COUNT (7)
		0x81, 0x02,                    //     INPUT (Data,Var,Abs)
		0xc0,                          //     END_COLLECTION

		0x05, 0x02,					//UsagePage(Simulation Controls),
		0x09, 0xbb,						// USAGE (THROTTLE)
		0x15, 0x00,                    //     LOGICAL_MINIMUM (-127)
		0x26, 0xff, 0x00,           //     LOGICAL_MAXIMUM (127)
		0x75, 0x08,                    //     REPORT_SIZE (8)
		0x95, 0x01,                    //     REPORT_COUNT (1)
		0x81, 0x02,                    //     INPUT (Data,Var,Abs)

		0x09, 0x39, 						// Usage (Hat switch),
		0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		0x25, 0x03,                    //     Logical Maximum (3),
		0x35, 0x00, 					// 		Physical Minimum (0)
		0x46, 0x0e, 01,					//		Physical Maximum (270),
		0x65, 0x14,						//		Unit (English Rotation: Angular Position), ; Degrees
		0x75, 0x04,							// REPORT_SIZE(4)
		0x95, 0x01,						//	Report Count (1),
		0x81, 0x02,						//  Input (Data, Variable, Absolute, Null State),

		0x05, 0x09,                    //     USAGE_PAGE (Button)
		0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
		0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
		0x19, 0x00,                    //     USAGE_MINIMUM (No buttons)
		0x29, 0x04,                    //     USAGE_MAXIMUM (Button 4)
		0x75, 0x01,                    //     REPORT_SIZE (1)
		0x95, 0x04,                    //     REPORT_COUNT (4)
		0x81, 0x02,                    //     INPUT (Data,Var,Abs)

		0xc0                          // END_COLLECTION

		// -> TOTAL = 89
};


static uchar    reportBuffer[9];    // buffer for axes bytes, last step in the pipeline
static uchar report2[1];			// buffer for buttons
static uchar    idleRate;   // repeat rate for keyboards, never used for mice
static uchar lx_arr = 0;		// joystick axes
static uchar ly_arr = 1;
static uchar lx_arr2 = 4;
static uchar ly_arr2 = 5;


usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;

	// The following requests are never used. But since they are required by
	// the specification, we implement them in this example.

	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    // class request type
		//        DBG1(0x50, &rq->bRequest, 1);   // debug output: print our request
		if(rq->bRequest == USBRQ_HID_GET_REPORT){  // wValue: ReportType (highbyte), ReportID (lowbyte)
			// we only have one report type, so don't look at wValue
			reportBuffer[0]= 0;
			reportBuffer[1]= 0;
			reportBuffer[2]= 0;
			reportBuffer[3]= 0;
			reportBuffer[4]= 0;
			reportBuffer[5]= 0;
			reportBuffer[6]= 0;
			reportBuffer[7]= 0;
			reportBuffer[8]= 0;

			usbMsgPtr = (void *)&reportBuffer;
			return sizeof(reportBuffer);
		}else if(rq->bRequest == USBRQ_HID_GET_IDLE){
			usbMsgPtr = &idleRate;
			return 1;
		}else if(rq->bRequest == USBRQ_HID_SET_IDLE){
			idleRate = rq->wValue.bytes[1];
		}
	}else{
		// no vendor specific requests implemented
	}
	return 0;   // default for not implemented requests: return no data back to host
}

// Delay function
void delayms(uint16_t millis) {
	while ( millis ) {
		_delay_ms(1);
		millis--;
	}
}

// Pin-toggling function
void switch_nunchuck(unsigned int st)
{
	switch(st)
	{
	case 0: PORTD &=~ _BV(switchpin);
	break;
	case 1: PORTD |= _BV(switchpin);
	break;
	default:break;
	}

}

// Low-pass filter function
void filter_point(float xac,float yac,float zac,float xac2,float yac2,float zac2)
{
	xraw[0]=xac;
	yraw[0]=yac;
	zraw[0]=zac;
	x2raw[0]=xac2;
	y2raw[0]=yac2;
	z2raw[0]=zac2;

	// Sampling rate ~= 80ms (92Hz)
	// Cheby2 low-pass filter: 2nd order, 10dB cutoff, .5 half the sampling rate (20ms=50Hz)

	// Coefficients
	float blow[3]={
			0.370223437617634,   0.246815625078423,   0.370223437617634};
	float alow[3]={
			1.000000000000000, -0.286868287209145,   0.274130787522837};


	//  Low-pass acceleration
	xlow[0] = (blow[0]*xraw[0]+blow[1]*xraw[1]+blow[2]*xraw[2]-alow[1]*xlow[1]-alow[2]*xlow[2]);
	ylow[0] = (blow[0]*yraw[0]+blow[1]*yraw[1]+blow[2]*yraw[2]-alow[1]*ylow[1]-alow[2]*ylow[2]);
	zlow[0] = (blow[0]*zraw[0]+blow[1]*zraw[1]+blow[2]*zraw[2]-alow[1]*zlow[1]-alow[2]*zlow[2]);

	xraw[2]=xraw[1];
	xraw[1]=xraw[0];
	yraw[2]=yraw[1];
	yraw[1]=yraw[0];
	zraw[2]=zraw[1];
	zraw[1]=zraw[0];

	xlow[2]=xlow[1];
	xlow[1]=xlow[0];
	ylow[2]=ylow[1];
	ylow[1]=ylow[0];
	zlow[2]=zlow[1];
	zlow[1]=zlow[0];

	x2low[0] = (blow[0]*x2raw[0]+blow[1]*x2raw[1]+blow[2]*x2raw[2]-alow[1]*x2low[1]-alow[2]*x2low[2]);
	y2low[0] = (blow[0]*y2raw[0]+blow[1]*y2raw[1]+blow[2]*y2raw[2]-alow[1]*y2low[1]-alow[2]*y2low[2]);
	z2low[0] = (blow[0]*z2raw[0]+blow[1]*z2raw[1]+blow[2]*z2raw[2]-alow[1]*z2low[1]-alow[2]*z2low[2]);

	x2raw[2]=x2raw[1];
	x2raw[1]=x2raw[0];
	y2raw[2]=y2raw[1];
	y2raw[1]=y2raw[0];
	z2raw[2]=z2raw[1];
	z2raw[1]=z2raw[0];

	x2low[2]=x2low[1];
	x2low[1]=x2low[0];
	y2low[2]=y2low[1];
	y2low[1]=y2low[0];
	z2low[2]=z2low[1];
	z2low[1]=z2low[0];

}

// Angle calculation based on status variables
void get_angles() {

	float pitch0=0,roll0=0,pitch1=0,roll1=0;
	float x1 = (float)((status[2] << 2) + (((status[5] & (3 << 2 ) >> 2)) >> 2));
	float y1 = (status[3] << 2) + (((status[5] & (3 << 4 ) >> 4)) >> 2);
	float z1 = (status[4] << 2) + (((status[5] & (3 << 6 ) >> 6)) >> 2);
	float x2 = (float)((status[8] << 2) + (((status[11] & (3 << 2 ) >> 2)) >> 2));
	float y2 = (status[9] << 2) + (((status[11] & (3 << 4 ) >> 4)) >> 2);
	float z2 = (status[10] << 2) + (((status[11] & (3 << 6 ) >> 6)) >> 2);


// Scale & offset values to approx. 0->210

//x2 = (x2-300)/2; 
//y2 = (y2-300)/2;
//z2 = (z2-300)/2;

x1 = x1-510;
y1 = y1-510;
z1 = z1-510;

x2 = x2-510;
y2 = y2-510;
z2 = z2-510;

/*
	if(x1<77)  x1= 77;
	if(x1>184) x1=184;
	if(y1<76)  y1= 76;
	if(y1>186) y1=186;
	if(z1<79)  z1= 79;
	if(z1>193) z1=193;

	if(x2<77)  x2= 77;
	if(x2>184) x2=184;
	if(y2<76)  y2= 76;
	if(y2>186) y2=186;
	if(z2<79)  z2= 79;
	if(z2>193) z2=193;

	x1 = x1*1.869-243.9;
	if(x1<-100) x1=-100;
	if(x1>100) x1 = 100;
	y1 = y1*1.818-238.2;
	if(y1<-100) y1=-100;
	if(y1>100) y1 = 100;
	z1 = z1*1.754-238.6;
	if(z1<-100) z1=-100;
	if(z1>100) z1 = 100;

	x2 = x2*1.869-243.9;
	if(x2<-100) x2=-100;
	if(x2>100) x2 = 100;
	y2 = y2*1.818-238.2;
	if(y2<-100) y2=-100;
	if(y2>100) y2 = 100;
	z2 = z2*1.754-238.6;
	if(z2<-100) z2=-100;
	if(z2>100) z2 = 100;
*/

	filter_point(x1,y1,z1,x2,y2,z2);

	x1=xlow[0];
	y1=ylow[0];
	z1=zlow[0];
	x2=x2low[0];
	y2=y2low[0];
	z2=z2low[0];

	// Clamp values to -210->210
	if(x1<-210) x1=-200;
	if(x1>210)  x1=210;
	if(y1<-210) y1=-210;
	if(y1>210)  y1=210;
	if(z1<-210) z1=-210;
	if(z1>210)  z1=210;

	if(x2<-210) x2=-200;
	if(x2>210)  x2=210;
	if(y2<-210) y2=-210;
	if(y2>210)  y2=210;
	if(z2<-210) z2=-210;
	if(z2>210)  z2=210;

	pitch0  = atan2( y1,sqrt(x1*x1+z1*z1));
	roll0   = atan2(-x1,sqrt(y1*y1+z1*z1));
	pitch1  = atan2( y2,sqrt(x2*x2+z2*z2));
	roll1   = atan2(-x2,sqrt(y2*y2+z2*z2));


// Scale to 0->180

	pitch0=(pitch0*57.29)+90;
	pitch0 = (pitch0*(-1) + 180);
	roll0 = (roll0*57.3)+90;
	roll0 = (roll0*(-1) + 180);
	pitch1=(pitch1*57.29)+90;
	pitch1=(pitch1*(-1) + 180);
	roll1 = (roll1*57.3) +90;
	roll1 = roll1*(-1) + 180;

// We need to move that to 0->255 for minimum calibration.
pitch1 *= 1.4;
roll1  *= 1.4;
pitch0 *= 1.4;
roll0  *= 1.4;


	reportBuffer[3] = ((int)roll0)*control1;
	reportBuffer[2] = ((int)pitch0)*control1;
	reportBuffer[6] = ((int)roll1)*control2;
	reportBuffer[7] = ((int)pitch1)*control2;

}

static void nunchuck_init()
{
	nunchuck_cmd_buf[0] = 0x40;
	nunchuck_cmd_buf[1] = 0x00;
	i2c_return = twi_writeTo(0x52, nunchuck_cmd_buf, 2, 1);
}

static void nunchuck_send_request()
{
	nunchuck_cmd_buf[0] = 0x00;// sends one byte
	i2c_return = twi_writeTo(0x52, nunchuck_cmd_buf, 1, 1);

}

static char nunchuk_decode_byte (char x)
{
	x = (x ^ 0x17) + 0x17;
	return x;
}

static int nunchuck_get_data()
{
	unsigned int cnt = 0,statuscnt=0;
	control1=FALSE;
	control2=FALSE;

	switch_nunchuck(0);
	i2c_return = twi_readFrom(0x52, nunchuck_buf, 6); // request data from nunchuck

	if(i2c_return>=6) control1=TRUE;
	for (cnt = 0; cnt < 6 ; cnt++) {
		status[statuscnt] = nunchuk_decode_byte (nunchuck_buf[cnt]);
		statuscnt++;
	}

	delayms(4);  //wait some time before sending request.. raise time if controller doesn't respond
	nunchuck_send_request();

	switch_nunchuck(1);
	i2c_return = twi_readFrom(0x52, nunchuck_buf, 6); // request data from nunchuck

	if(i2c_return>=6) control2=TRUE;
	for (cnt = 0; cnt < 6 ; cnt++) {
		status[statuscnt] = nunchuk_decode_byte (nunchuck_buf[cnt]);
		statuscnt++;
	}
	delayms(4);  //wait some time before sending request.. raise time if controller doesn't respond
	nunchuck_send_request();
	return 1;   // success
}

// returns z button state: 1=pressed, 0=notpressed
int nunchuck_zbutton(unsigned int nc)
{
	switch (nc)
	{
	case 0:
		return !(status[5] & 1 );
		break;

	case 1:
		return !(status[11] &1 );
		break;
	default: return 0;
	break;
	}

}

// returns c button state: 1=pressed, 0=notpressed
int nunchuck_cbutton(unsigned int nc)
{
	switch (nc)
	{
	case 0:
		return !((status[5] & 2 )>>1);
		break;

	case 1:
		return !((status[11] &2 )>>1);
		break;
	default: return 0;
	break;
	}
}

// Decode nunchuk bytes
void make_sense_nunchuck() {

float temp;

	reportBuffer[lx_arr]= 0x80;
	reportBuffer[ly_arr]= 0x80;
	reportBuffer[lx_arr2]= 0x80;
	reportBuffer[ly_arr2]= 0x80;
	report2[0] = 0;

	// joysticks
	temp=(status[0]-32) * 1.35;
	if(temp<0) temp=0; if(temp>255) temp=255;
	reportBuffer[lx_arr] = (int)temp*control1;
	temp=((255-status[1])-32)*1.35;
	if(temp<0) temp=0; if(temp>255) temp=255;
	reportBuffer[ly_arr] = (int)temp*control1;
	temp=(status[6]-32) * 1.35;
	if(temp<0) temp=0; if(temp>255) temp=255;
	reportBuffer[lx_arr2] = (int)temp*control2;
	temp=((255-status[7])-32)*1.35;
	if(temp<0) temp=0; if(temp>255) temp=255;
	reportBuffer[ly_arr2] = (int)temp*control2;


	// buttons
	if (nunchuck_zbutton(0)) {
		report2[0] = report2[0] | (control1 << 4);
	}
	if (nunchuck_cbutton(0)) {
		report2[0] = report2[0] | (control1 << 5);
	}

	if (nunchuck_zbutton(1)) {
		report2[0] = report2[0] | (control2 << 6);
	}
	if (nunchuck_cbutton(1)) {
		report2[0] = report2[0] | (control2 << 7);
	}
	report2[0] |= 4;

	get_angles();




}
int main(void) {
	unsigned int i = 0;

	usbDeviceDisconnect();  // enforce re-enumeration, do this while interrupts are disabled!
	i = 500;
	while(i--){             // fake USB disconnect for > 500 ms
		wdt_reset();
		_delay_ms(1);
	}
	usbInit();
	usbDeviceConnect();

	sei();  // Set interrupts

	twi_init();  // Initialize TWI/I2C

	DDRD |= _BV(switchpin); // Switch to second nunchuk - only one works if this is not here :(

	// Start up nunchuks
	switch_nunchuck(0);
	nunchuck_init();
	switch_nunchuck(1);
	nunchuck_init();

	for(;;){                // main event loop
		wdt_reset();

		while (!usbInterruptIsReady()) usbPoll(); // need to wait for int
		nunchuck_get_data();
		make_sense_nunchuck();
		usbSetInterrupt((void *)&reportBuffer + 0, 8);//first 8
		while (!usbInterruptIsReady()) usbPoll();//need to wait
		usbSetInterrupt((void *)&report2 , 1); //next 8 bytes

	}
	return 0;

}


