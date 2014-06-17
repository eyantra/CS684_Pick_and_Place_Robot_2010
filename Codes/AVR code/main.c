/**@mainpage 
		Platform: Fire Bird V ATMEGA 2560
		Pick 'n' Place Robot
		@author Group 12: Bharat Jain(09305029) and Dinesh Rajpoot(09305043)
		
		M.Tech IIT Bombay 2009-2011
		
		Last Modification: 2010-11-07
		
		This program works as back end of pick 'n' place robot and moves the robot based on cammands from Matlab.
		
		
		The objective of this project is to pick and stack objects of similar color at base. The snapshots are taken from camera mounted on the Firebird and based on some interesting properties objects are picked. 
		
		The picked object is then placed to base where objects of same kind are placed. The snapshots/images are taken from overhead camera which is interfaced with MATLAB. Fire Bird then sends the snapshot to PC 
		
		where Image Processing is done and it will pick the object of minimum distance with the help of Arm assembly. Robot then places the objects at base and makes stack of similar objects at base.
		
		
		
		Note: 
 
       Make sure that in the configuration options following settings are 
 	   done for proper operation of the code

 	   Microcontroller: atmega2560
 	   Frequency: 11059200
 	   Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)

**************************************************************************************************/

/********************************************************************************

   Copyright (c) 2010, ERTS Lab, IIT Bombay erts@cse.iitb.ac.in 
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include "firebird.h"
unsigned int a,data1,data2,flag=0;
/**
* This is main function
**/
void main(void)
{
//Intializing devices like LCD, PWM, Servo motors etc.
init_devices();
position_encoder_interrupt_init();
//a =send(data3);
  while(1)

	{   
        
		if(data  > 5)
			{
				data2 =data;// Received angle, waiting for direction of movement
				while(data != 2 && data != 3);
				if(data == 2 ) // Move left
					{

	 					angle_rotate_left(data2);
	 					UDR0 = 50;
						//send ack 50 once moved
						data=0;
					}
				if(data ==3) // Move Right
					{

	 					angle_rotate_right(data2);
						UDR0 = 50;
						//send ack 50 once moved
						data=0;
	 					
					}
			
			}
			    if(data == 1)
				    {
 					linear_distance_mm(20); // Move forward 20 shaft count
 					velocity(150,150);
					//send ack 50 once moved
					UDR0 = 50;
					data=0;
					
				    }
	 
		        if(data == 4 )
			        {
				
				     pick();
				    _delay_ms(500);
				    UDR0 = 51;
				    //send ack 51 once picked object
				    data=0;
				
			        }
		       // SEND ACK 254  after picking object
				if(data == 5)
					{
					place();
					_delay_ms(500);
					UDR0 = 52;
					//send ack 52 once placed object
					data =0;
					}
				if(data == 0)
					{
				stop();
					}
	
					
	}
}





