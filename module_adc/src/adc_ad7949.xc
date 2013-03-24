/**
 * \file adc_ad7949.xc
 *
 * Copyright 2012, Synapticon GmbH. All rights reserved.
 * Author: Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/

#include <xs1.h>
#include <stdint.h>
#include <print.h>
#include <xclib.h>
#include <xscope.h>
//#include "zip.h"
#include "refclk.h"
//int trigger=0;
static unsigned adc_data_a[5];
static unsigned adc_data_b[5];
//static unsigned ia_calibr,ib_calibr;

extern out port p_ifm_ext_d2;
extern out port p_ifm_ext_d3;

unsigned short iIndexADC = 0;

/* typedef enum { */
/*   ACQ_MOTOR_SPI, */
/*   ACQ_MOTOR_WAIT, */
/*   CONV_MOTOR, */
/*   ACQ_OTHER_SPI, */
/*   ACQ_OTHER_WAIT, */
/*   CONV_OTHER */
/* } adc_state_t; */

/* zipper based on Henk's module_zip */
static unsigned zipLookup[256] = {
    0x00000000, 0x00000001, 0x00000010, 0x00000011,
    0x00000100, 0x00000101, 0x00000110, 0x00000111,
    0x00001000, 0x00001001, 0x00001010, 0x00001011,
    0x00001100, 0x00001101, 0x00001110, 0x00001111,
    0x00010000, 0x00010001, 0x00010010, 0x00010011,
    0x00010100, 0x00010101, 0x00010110, 0x00010111,
    0x00011000, 0x00011001, 0x00011010, 0x00011011,
    0x00011100, 0x00011101, 0x00011110, 0x00011111,
    0x00100000, 0x00100001, 0x00100010, 0x00100011,
    0x00100100, 0x00100101, 0x00100110, 0x00100111,
    0x00101000, 0x00101001, 0x00101010, 0x00101011,
    0x00101100, 0x00101101, 0x00101110, 0x00101111,
    0x00110000, 0x00110001, 0x00110010, 0x00110011,
    0x00110100, 0x00110101, 0x00110110, 0x00110111,
    0x00111000, 0x00111001, 0x00111010, 0x00111011,
    0x00111100, 0x00111101, 0x00111110, 0x00111111,
    0x01000000, 0x01000001, 0x01000010, 0x01000011,
    0x01000100, 0x01000101, 0x01000110, 0x01000111,
    0x01001000, 0x01001001, 0x01001010, 0x01001011,
    0x01001100, 0x01001101, 0x01001110, 0x01001111,
    0x01010000, 0x01010001, 0x01010010, 0x01010011,
    0x01010100, 0x01010101, 0x01010110, 0x01010111,
    0x01011000, 0x01011001, 0x01011010, 0x01011011,
    0x01011100, 0x01011101, 0x01011110, 0x01011111,
    0x01100000, 0x01100001, 0x01100010, 0x01100011,
    0x01100100, 0x01100101, 0x01100110, 0x01100111,
    0x01101000, 0x01101001, 0x01101010, 0x01101011,
    0x01101100, 0x01101101, 0x01101110, 0x01101111,
    0x01110000, 0x01110001, 0x01110010, 0x01110011,
    0x01110100, 0x01110101, 0x01110110, 0x01110111,
    0x01111000, 0x01111001, 0x01111010, 0x01111011,
    0x01111100, 0x01111101, 0x01111110, 0x01111111,
    0x10000000, 0x10000001, 0x10000010, 0x10000011,
    0x10000100, 0x10000101, 0x10000110, 0x10000111,
    0x10001000, 0x10001001, 0x10001010, 0x10001011,
    0x10001100, 0x10001101, 0x10001110, 0x10001111,
    0x10010000, 0x10010001, 0x10010010, 0x10010011,
    0x10010100, 0x10010101, 0x10010110, 0x10010111,
    0x10011000, 0x10011001, 0x10011010, 0x10011011,
    0x10011100, 0x10011101, 0x10011110, 0x10011111,
    0x10100000, 0x10100001, 0x10100010, 0x10100011,
    0x10100100, 0x10100101, 0x10100110, 0x10100111,
    0x10101000, 0x10101001, 0x10101010, 0x10101011,
    0x10101100, 0x10101101, 0x10101110, 0x10101111,
    0x10110000, 0x10110001, 0x10110010, 0x10110011,
    0x10110100, 0x10110101, 0x10110110, 0x10110111,
    0x10111000, 0x10111001, 0x10111010, 0x10111011,
    0x10111100, 0x10111101, 0x10111110, 0x10111111,
    0x11000000, 0x11000001, 0x11000010, 0x11000011,
    0x11000100, 0x11000101, 0x11000110, 0x11000111,
    0x11001000, 0x11001001, 0x11001010, 0x11001011,
    0x11001100, 0x11001101, 0x11001110, 0x11001111,
    0x11010000, 0x11010001, 0x11010010, 0x11010011,
    0x11010100, 0x11010101, 0x11010110, 0x11010111,
    0x11011000, 0x11011001, 0x11011010, 0x11011011,
    0x11011100, 0x11011101, 0x11011110, 0x11011111,
    0x11100000, 0x11100001, 0x11100010, 0x11100011,
    0x11100100, 0x11100101, 0x11100110, 0x11100111,
    0x11101000, 0x11101001, 0x11101010, 0x11101011,
    0x11101100, 0x11101101, 0x11101110, 0x11101111,
    0x11110000, 0x11110001, 0x11110010, 0x11110011,
    0x11110100, 0x11110101, 0x11110110, 0x11110111,
    0x11111000, 0x11111001, 0x11111010, 0x11111011,
    0x11111100, 0x11111101, 0x11111110, 0x11111111,
};

void outputWordsZipped(clock clk,
		       in buffered port:32 p_data_a,
		       in buffered port:32 p_data_b,
		       buffered out port:32 p, int d, int c, int b, int a) {

	#pragma unsafe arrays
	  int bits[4];

	//  if(trigger > 0){
	//  printhex(d); printstr(" ");  printhex(c);printstr(" "); printhex(b); printstr(" "); printhexln(a);
	//  }

	  for (int i = 0; i < 4; i++) {
		bits[i] = zipLookup[a & 0xff];
		bits[i] = bits[i] << 1 | zipLookup[b & 0xff];
		bits[i] = bits[i] << 1 | zipLookup[c & 0xff];
		bits[i] = bits[i] << 1 | zipLookup[d & 0xff];
		a >>= 8;
		b >>= 8;
		c >>= 8;
		d >>= 8;
	  }

	 /*
	  if(trigger > 0 ){
		  //if(trigger == 1)
		  {
		  printhex(bits[0]); printstr(" ");  printhex(bits[1]);printstr(" "); printhex(bits[2]); printstr(" "); printhexln(bits[3]);
		  }
	  trigger--;
	  }
	*/
	  stop_clock(clk);
	  clearbuf(p_data_a);
	  clearbuf(p_data_b);
	  //clearbuf(p);
	  p <: bits[0];
	  start_clock(clk);

	  p <: bits[1];
	  p <: bits[2];
	  p <: bits[3];

	  sync(p);
	  stop_clock(clk);
}

static void configure_adc_ports(clock clk,
				buffered out port:32 p_sclk_conv_mosib_mosia,
				in buffered port:32 p_data_a,
				in buffered port:32 p_data_b)
{
	  /* SCLK period >= 22ns (45.45 MHz)
		 clk needs to be configured twice as fast as the required SCLK frequency */
	  configure_clock_rate_at_most(clk, 250, 5); // 83.3  --  < (2*45.45)

	  /* when idle, keep clk and mosi low, conv high */
	  configure_out_port(p_sclk_conv_mosib_mosia, clk, 0b0100);

	  configure_in_port(p_data_a, clk);
	  //set_port_shift_count(p_data_a, 2);
	  //set_port_sample_delay(p_data_a);

	  configure_in_port(p_data_b, clk);
	  //set_port_shift_count(p_data_b, 2);
	  //set_port_sample_delay(p_data_b);

	  start_clock(clk);
}




static inline unsigned convert(unsigned raw)
{
  unsigned data;
  /* raw == 0b xxxx aabb ccdd eeff ...
     we read every data bit twice because of port clock setting */
  raw = bitrev(raw);
  data  = raw & 0x06000000;
  data >>= 2;
  data |= raw & 0x00600000;
  data >>= 2;
  data |= raw & 0x00060000;
  data >>= 2;
  data |= raw & 0x00006000;
  data >>= 2;
  data |= raw & 0x00000600;
  data >>= 2;
  data |= raw & 0x00000060;
  data >>= 2;
  data |= raw & 0x00000006;
  data >>= 1;
  return data;
}




#pragma unsafe arrays
static void adc_ad7949_singleshot(   buffered out port:32 p_sclk_conv_mosib_mosia,
		   	   	   	   	   	   	   	   	   	   in buffered port:32 p_data_a,
		   	   	   	   	   	   	   	   	   	   in buffered port:32 p_data_b,
		   	   	   	   	   	   	   	   	   	    	   	   	   	   clock clk )
{
		/*                                    0bRB    <REF >  <INx ><INCC>  00 */
		const unsigned adc_config_imot    =   0b000000110000110000001111111100;   		/* Motor current (ADC Channel 0), unipolar, referenced to GND */

		const unsigned adc_config_other[] = { 0b000000110000110000001111001100,   		/* Temperature */
											  0b000000110000110011001111111100, 		/* ADC Channel 2, unipolar, referenced to GND */
											  0b000000110000111100111111111100,   		/* ADC Channel 4, unipolar, referenced to GND  */
											  0b000000110000111100111111111100,   		/* ADC Channel 5, unipolar, referenced to GND  */

											//0b000000110000111100000000111100,   		/* ADC Channel 4, unipolar, differential pair */
											//0b000000110000111111111111111100,
		}; 		/* ADC Channel 7, unipolar, referenced to GND */
		const unsigned clk_signal = 0x2aaaaaa8;
		//const unsigned clk_signal[2]  = { 0xaaaaaaa8, 0x02aaaaaa };
		//const unsigned adc_trigcnv[2] = { 0x00000000, 0xf8000000 };

		const unsigned delay_conv = (15*USEC_FAST) / 5; // 2.2 us /* FIXME: why does this result in a delay of 3.2us?! */
		const unsigned delay_acq =   (9*USEC_FAST) / 5; // 1.8 us
		timer t;
		unsigned ts;

		//  int cmd;
		//  unsigned char ct;
		unsigned data_raw_a, data_raw_b;

		/* Reading/Writing after conversion (RAC)
		  Read previous conversion result
		  Write CFG for next conversion */
		#define SPI_IDLE   configure_out_port(p_sclk_conv_mosib_mosia, clk, 0b0100)
		#define SPI_SELECT configure_out_port(p_sclk_conv_mosib_mosia, clk, 0b0000)

		//   #define WAIT		case t when timerafter(ts) :> ts:

	  	SPI_IDLE;
		t :> ts;
		ts += delay_conv;
		t when timerafter(ts) :> ts;  // WAIT;

		//case ACQ_OTHER_SPI:
		outputWordsZipped(clk, p_data_a, p_data_b, p_sclk_conv_mosib_mosia, adc_config_imot, adc_config_imot, 0, clk_signal);

		SPI_SELECT;

		p_data_a :> data_raw_a;
		adc_data_a[4] = convert(data_raw_a);
		p_data_b :> data_raw_b;
		adc_data_b[4] = convert(data_raw_b);

		ts += delay_acq;
		t when timerafter(ts) :> ts;

		SPI_IDLE;
		ts += delay_conv;
		//WAIT;
		t when timerafter(ts) :> ts;


		//case ACQ_MOTOR_SPI:
		outputWordsZipped(clk, p_data_a, p_data_b, p_sclk_conv_mosib_mosia, adc_config_other[iIndexADC], adc_config_other[iIndexADC], 0, clk_signal);

		ts += delay_acq;
		SPI_SELECT;

		p_data_a :> data_raw_a;
		adc_data_a[iIndexADC] = convert(data_raw_a);
		p_data_b :> data_raw_b;
		adc_data_b[iIndexADC] = convert(data_raw_b);

		iIndexADC++;
		iIndexADC &= 0x3;

}



void adc_ad7949_triggered( chanend c_adc,
			   	   	   	   clock clk,
			   	   	   	   buffered out port:32 p_sclk_conv_mosib_mosia,
			   	   	   	   in buffered port:32 p_data_a,
			   	   	   	   in buffered port:32 p_data_b )
{
	  timer tx;
	  unsigned ts;
	  int cmd;
	  int xCount=100;
	  unsigned char iFlag=0;

	  //set_thread_fast_mode_on();

	  configure_adc_ports(clk, p_sclk_conv_mosib_mosia, p_data_a, p_data_b);

	  //ii=4;
	  //while(ii-- > 0)adc_ad7949_singleshot( p_sclk_conv_mosib_mosia, p_data_a, p_data_b, clk );

	 // ia_calibr=0;
	 // ib_calibr=0;
	 // ii = 32;
	 /* while(ii > 0)
	  {
		  adc_ad7949_singleshot( p_sclk_conv_mosib_mosia, p_data_a, p_data_b, clk );
		  ia_calibr += adc_data_a[4];
		  ib_calibr += adc_data_b[4];
		  ii--;
	  }
	  ia_calibr /= 32;
	  ib_calibr /= 32;
	  printstr("ia_calibration ");   printint(ia_calibr); printstr("\n");
	  printstr("ib_calibration ");   printint(ib_calibr); printstr("\n");
	  */
	  tx :> ts;


	while (1)
	{
		tx when timerafter(ts + 250) :> ts;   	// 250 => 1µsec 125= 0,5µsec

//		iFlag ^= 0x01;    p_ifm_ext_d2 <: iFlag;

	    xCount--;
		if(xCount==0){
		p_ifm_ext_d3 <: 1;
		adc_ad7949_singleshot( p_sclk_conv_mosib_mosia, p_data_a, p_data_b, clk );
		xCount = 55;
		tx :> ts;
		}

#pragma ordered
	    select {
		case c_adc :> cmd:
		    if(cmd == 0) {
		    				c_adc <: adc_data_a[4]; 		//  - ia_calibr;
		    				c_adc <: adc_data_b[4]; 		//  - ib_calibr;
		    				c_adc <: adc_data_a[0]; 		//
		    				c_adc <: adc_data_a[1]; 		//
		    				c_adc <: adc_data_a[2]; 		//
		    				c_adc <: adc_data_a[3]; 		//
		    				c_adc <: adc_data_b[0]; 		//
		    				c_adc <: adc_data_b[1]; 		//
		    				c_adc <: adc_data_b[2]; 		//
		    				c_adc <: adc_data_b[3];
		    				xCount = 25;
		 	 	 	 	  }
		break;
		default:  break;
	    }// end of select


		p_ifm_ext_d3 <: 0;
	}// end while 1
}






  /* serialization -- LSB first */

  /* adc_config: !! reverse bit-order compared to AD7949 datasheet !!
     Bit    Name  Description
     32:14  0     unused
     13     RB    Read back CFG reg       (  1 => no)
     12:11  SEQ   Sequencer               ( 00 => disable)
     10:8   REF   Reference selection     (100 => internal)
     7      BW    LPF bandwidth           (  1 => full)
     6:4    INx   Input channel selection (000 => CH0)
     3:1    INCC  Input channel config    (111 => unipolar, referenced to GND)
     0      CFG   Config update           (  1 => overwrite cfg reg)
     -1     --PADDING-- (delay 1 clk cycle)
     => adc_config = 0b100100100011110
  */

