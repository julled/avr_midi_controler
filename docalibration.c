

    /*  
     *  
     *   Copyright (c) 2001, Carlos E. Vidales. All rights reserved.  
     *   
     *   This sample program was written and put in the public domain   
     *    by Carlos E. Vidales.  The program is provided "as is"   
     *    without warranty of any kind, either expressed or implied.  
     *   If you choose to use the program within your own products  
     *    you do so at your own risk, and assume the responsibility  
     *    for servicing, repairing or correcting the program should  
     *    it prove defective in any manner.  
     *   You may copy and distribute the program's source code in any   
     *    medium, provided that you also include in each copy an  
     *    appropriate copyright notice and disclaimer of warranty.  
     *   You may also modify this program and distribute copies of  
     *    it provided that you include prominent notices stating   
     *    that you changed the file(s) and the date of any change,  
     *    and that you do not charge any royalties or licenses for   
     *    its use.  
     *      
     *  
     *   File Name:  sample.c  
     *  
     *  
     */   

	     /*  
     *  This file has been changed by Julian D. in September 2011
     *     for the use in the Midi-Controller-Project. 
	 *  The program is provided "as is"  without warranty of any kind,
	 *    either expressed or implied.  
     *  If you choose to use the program within your own products  
     *    you do so at your own risk, and assume the responsibility  
     *    for servicing, repairing or correcting the program should  
     *    it prove defective in any manner.   
	 *  I do not charge any royalties or licenses for  its use.   
     *   
     */   
       
       
    #define _SAMPLE_C_   
       
    /****************************************************/   
    /*                                                  */   
    /* Included files                                   */   
    /*                                                  */   
    /****************************************************/   
       
    #include "calibrate.h"   
//	#include "config.h"   
	#include <stdio.h>
	#include <util/delay.h> 
	#include "tlc5940.h"

       
    /****************************************************/   
    /*                                                  */   
    /* Local Definitions                                */   
    /*                                                  */   
    /****************************************************/   
       
    #define MAX_SAMPLES         3
	#define validcoord 80
	#define TOUCHCONFIG_ADC_SAMPLES 7

	#define valid_ADC_val_xmin 30//30
	#define valid_ADC_val_ymin 80//80
	#define valid_ADC_val_xmax 950//750
	#define valid_ADC_val_ymax 950//900

	#define TOUCH_DDR DDRA
	#define TOUCH_PORT PORTA
	#define TOUCH_X1 PA0
	#define TOUCH_X2 PA2
	#define TOUCH_Y1 PA4    //  PA4 
	#define TOUCH_Y2 PA6
       
       
    /****************************************************/   
    /*                                                  */   
    /* Local Variables                                  */   
    /*                                                  */   
    /****************************************************/   
       
            /* NOTE: Even though the validity of the calibration/translation method  */   
            /*        proposed has been verified with empirical data from several    */   
            /*        actual touch screen enabled displays, for the convenience of   */   
            /*        this exercise, the raw and expected data used and presented    */   
            /*        below are artificial.  When used with actual data the          */   
            /*        functions presented yield results that may be off by a larger  */   
            /*        but still small percentage (~1-3%) due to electrical noise and */   
            /*        human error (i.e., the hand touching a screen target and       */   
            /*        missing by a small amount.)                                    */   
       
       
            /* The array of input points.  The first three are used for calibration. */   
            /* These set of points assume that the touchscreen has vertical and      */   
            /*  horizontal resolutions of 1024 pixels (10-bit digitizer.)            */    
               
    /*static const char * _acPos[] = {   
      "(upper left position)",   
      "(middle right position)",   
      "(lower middle position)"   
    }; */
       
    POINT screenSample[MAX_SAMPLES] =   { 
	                                            { 0, 0 },   
                                                { 0, 0 },   
                                                { 0, 0 }   
												/*
                                                {  73, 154 },   
                                                { 891, 516 },   
                                                { 512, 939 },   
                                                { 265, 414 },   
                                                { 606, 171 },   
                                                { 768, 700 },   
                                                { 111, 956 },   
                                                { 448, 580 }  */ 
                                        } ;   
       
       
       
            /* The array of expected "right answers."  The values selected assume a  */   
            /*  vertical and horizontal display resolution of 240 pixels.            */ 
       
    POINT displaySample[MAX_SAMPLES] =  {   
												{ 1, 1 },   
                                                { 1, 127 },   
                                                { 127, 127 } 
	  /*
		  										{ 1, 1 },   
                                                { 64, 127 },   
                                                { 127, 64 } 
		*/										
												/*	
												{ 20, 20 },   
                                                { 64, 108 },   
                                                { 108, 64 } 
												*/
												/*
                                                {  20,  30 },   
                                                { 210, 120 },   
                                                { 120, 210 }  ,   
                                                {  70,  90 },   
                                                { 150,  40 },   
                                                { 180, 160 },   
                                                {  30, 210 },   
                                                { 110, 130 }  */ 
                                        } ;   
       
       
            /* An array of perfect input screen points used to obtain a first pass   */   
            /*  calibration matrix good enough to collect calibration samples.       */   
       
    POINT perfectScreenSample[3] =  {   
                                                { 0, 0 },   
                                                { 0, 0 },   
                                                { 0, 0 }   
                                    } ;   
       
       
       
            /* An array of perfect display points used to obtain a first pass        */   
            /*  calibration matrix good enough to collect calibration samples.       */   
       
    POINT perfectDisplaySample[3] = {   
												{ 1, 1 },   
                                                { 1, 127 },   
                                                { 127, 127 } 
 /*
												{ 1, 1 },   //{ 20, 20 }, 
                                                { 64, 127 },   //{ 64, 108 },   
                                                { 127, 64 }  */
												
                                    } ;   
       
    MATRIX  matrix ;   

	/*
	{ -62422, 944, 4819764, -1475, -34574,4189998,-206796}  GUTE MATRIX!!! }
	von 
	 POINT displaySample[MAX_SAMPLES] =  {   
		  										{ 5, 5 },   
                                                { 64, 123 },   
                                                { 123, 64 } 
												



       
    static int xPhys, yPhys;   
    static int PressDn;  */
	
//	uint8_t validsamples;	 
       
    /****************************************************/   
    /*                                                  */   
    /* Forward Declaration of local functions           */   
    /*                                                  */   
    /****************************************************/   
       
    int mycalibrate(void) ;   
    void  greeting( void ) ;   
    static void _WaitForPressedState(int Pressed);   
    static void _GetPhysValues(int32_t LogX, int32_t LogY, int32_t * pPhysX, int32_t * pPhysY);
    static void _DispStringCentered(const char * pString);   
       
       
    //user define   
       
      
     
    /**********************************************************************  
     *  
     *     Function: main()  
     *  
     *  Description: Entry point into console version of sample  
     *                program that exercises the calibration   
     *                functions.  
     *   
     *  Argument(s): argCount - the number of arguments provided  
     *               argValue - pointer to the list of char strings   
     *                           representing the command line arguments.  
     *   
     *       Return: void  
     *  
     */   
    int calibrate(void)   
    {   
       
        int retValue = OK ;   
       
           
        //POINT   display ;   
           
           
        int  i ;   
       
            /* Look at the matrix values when we use a perfect sample set. */   
            /* The result is a unity matrix.                               */   
           
        //greeting() ;   
       
       
            /* The following call calculates the translation matrix that   */   
            /*  results when the three consecutive points in the sample    */   
            /*  set are used.  Such points are assumed to be properly      */   
            /*  spaced within the screen surface.                          */   
            /* Note that we call the function twice as we would normally   */   
            /*  do within a calibration routine.  The first time we call   */   
            /*  it using a perfect set of display and screen arguments.    */   
            /* Such a call is made to obtain a calibration matrix that is  */   
            /*  just good enough to collect samples to do the real         */   
            /*  calibration.                                               */   
            /*                                                             */   
            /*                                                             */   
            /*                                                             */   
            /*                 NOTE!    NOTE!    NOTE!                     */   
            /*                                                             */   
            /*  setCalibrationMatrix() and getDisplayPoint() will do fine  */   
            /*  for you as they are, provided that your digitizer          */   
            /*  resolution does not exceed 10 bits (1024 values).  Higher  */   
            /*  resolutions may cause the integer operations to overflow   */   
            /*  and return incorrect values.  If you wish to use these     */   
            /*  functions with digitizer resolutions of 12 bits (4096      */   
            /*  values) you will either have to a) use 64-bit signed       */   
            /*  integer variables and math, or b) judiciously modify the   */   
            /*  operations to scale results by a factor of 2 or even 4.    */   
            /*                                                             */   
       
      /* 
        for (i = 0; i  3; i++) {   
            _GetPhysValues(perfectDisplaySample[i].x, perfectDisplaySample[i].y, &perfectScreenSample[i].x, &perfectScreenSample[i].y, _acPos[i]);   
        }   
        //GUI_Clear();   
          
         Lcd_printf(10,40, 0xff0000,0xFFFFFF,0,"Screen.X1=%6ld,Screen.Y1=%6ld\n",perfectScreenSample[0].x,perfectScreenSample[0].y);  
         Lcd_printf(10,60, 0xff0000,0xFFFFFF,0,"Screen.X2=%6ld,Screen.Y2=%6ld\n",perfectScreenSample[1].x,perfectScreenSample[1].y);  
         Lcd_printf(10,80, 0xff0000,0xFFFFFF,0,"Screen.X3=%6ld,Screen.Y3=%6ld\n",perfectScreenSample[2].x,perfectScreenSample[2].y);  
          */   
		  /*
         _WaitForPressedState(1);  
         _WaitForPressedState(0);  

           
        setCalibrationMatrix( &perfectDisplaySample[0],    
                              &perfectScreenSample[0],    
                              &matrix ) ;   
           
 
        _WaitForPressedState(1);  
        _WaitForPressedState(0);  
         */ 
                /*  
         Lcd_printf(10,20, 0xff0000,0xFFFFFF,0,"\n\nLook at the unity matrix:\n\n");  
           
         Lcd_printf(10,40, 0xff0000,0xFFFFFF,0,"matrix.An=%6ld,matrix.Bn=%6ld\n",matrix.An,matrix.Bn);  
                     
         Lcd_printf(10,60, 0xff0000,0xFFFFFF,0,"matrix.Cn=%6ld,matrix.Dn=%6ld\n",matrix.Cn,matrix.Dn);  
         Lcd_printf(10,80, 0xff0000,0xFFFFFF,0,"matrix.En=%6ld,matrix.Fn=%6ld\n",matrix.En,matrix.Fn);  
         Lcd_printf(10,140, 0xff0000,0xFFFFFF,0,"matrix.Divider = % 6ld\n",matrix.Divider ) ;  
                 
      _WaitForPressedState(1);  
      _WaitForPressedState(0);  
        */   
       
            /* Now is when we need to do the work to collect a real set of */   
            /*  calibration data.                                          */   
       
            /* Draw three targets on your display. Drawing one at time is  */   
            /*  probably a simpler implementation. These targets should be */   
            /*  widely separated but also avoid the areas too near the     */   
            /*  edges where digitizer output tends to become non-linear.   */   
            /*  The recommended set of points is (in display resolution    */   
            /*   percentages):                                             */   
            /*                                                             */   
            /*                  ( 15, 15)                                  */   
            /*                  ( 50, 85)                                  */   
            /*                  ( 85, 50)                                  */   
            /*                                                             */   
            /* Each time save the display and screen set (returned by the  */   
            /*  digitizer when the user touches each calibration target    */   
            /*  into the corresponding array).                             */    
            /* Since you normalized your calibration matrix above, you     */   
            /*  should be able to use touch screen data as it would be     */   
            /*  provided by the digitizer driver.  When the matrix equals  */   
            /*  unity, getDisplayPoint() returns the same raw input data   */   
            /*  as output.                                                 */  
			 
        	setOutput(TOUCH_DDR, TOUCH_Y1);
			setOutput(TOUCH_DDR, TOUCH_Y2);
			setOutput(TOUCH_DDR, TOUCH_X1);	
			setOutput(TOUCH_DDR, TOUCH_X2);	

			setLow(TOUCH_PORT, TOUCH_X1);
			setLow(TOUCH_PORT, TOUCH_X2);
			setLow(TOUCH_PORT, TOUCH_Y1);
			setLow(TOUCH_PORT, TOUCH_Y2);
 			_delay_ms(1000);


			_WaitForPressedState(0);
            _WaitForPressedState(1);  
     	
            for (i = 0; i < 3; i++) 
			{   
            	_GetPhysValues( displaySample[i].x	 , displaySample[i].y , &screenSample[i].x , &screenSample[i].y);   
				_WaitForPressedState(0);
        	}   
           
       
       
            /* Call the function once more to obtain the calibration       */   
            /*  factors you will use until you calibrate again.            */   
            setCalibrationMatrix( &displaySample[0], &screenSample[0], &matrix ) ;   
       
            /* Let's see the matrix values for no particular reason.       */   
               
            /*  
        printf("\n\nThis is the actual calibration matrix that we will use\n"  
               "for all points (until we calibrate again):\n\n"  
               "matrix.An = % 8d  matrix.Bn = % 8d  matrix.Cn = % 8d\n"  
               "matrix.Dn = % 8d  matrix.En = % 8d  matrix.Fn = % 8d\n"  
               "matrix.Divider = % 8d\n",  
               matrix.An,matrix.Bn,matrix.Cn,  
               matrix.Dn,matrix.En,matrix.Fn,  
               matrix.Divider ) ;  
      
    */   
       
            /* Now, lets use the complete set of screen samples to verify  */   
            /*  that the calculated calibration matrix does its job as     */   
            /*  expected.                                                  */   
       // OSPrintf("\n\nShow the results of our work:\n\n"   
        //       "  Screen Sample    Translated Sample    Display Sample\n\n" ) ;   
       
       
            /* In a real application, your digitizer driver interrupt      */   
            /*  would probably do the following:                           */   
            /*      1) collect raw digitizer data,                         */   
            /*      2) filter the raw data which would probably contain    */   
            /*          position jitter,                                   */   
            /*      3) filter out repeated values (a touch screen          */   
            /*          controller normally continues causing interrupts   */   
            /*          and collecting data as long as the user is         */   
            /*          pressing on the screen), and                       */   
            /*      4) call the function getDisplayPoint() to obtain       */   
            /*          the display coordinates that the user meant to     */   
            /*          input as he touched the screen.                    */   
            /*                                                             */   
            /* This code sample, of course, only uses sample data.  So we  */   
            /*  simply run through all the available sample points.        */   
       
    /*  
        for( n = 0 ; n  MAX_SAMPLES ; ++n )  
        {  
            getDisplayPoint( &display, &screenSample[n], &matrix ) ;  
            printf("  % 6d,%-6d      % 6d,%-6d       % 6d,%-6d\n",   
                    screenSample[n].x,  screenSample[n].y,  
                    display.x,          display.y,  
                    displaySample[n].x, displaySample[n].y ) ;  
        }  
      
    */   
           
        return( retValue ) ;   
       
    } // end of main()    
       
       


       
    void _WaitForPressedState(int pressed) 
	{   
		uint16_t x_ADC_values[TOUCHCONFIG_ADC_SAMPLES];
		uint16_t y_ADC_values[TOUCHCONFIG_ADC_SAMPLES];

		uint8_t validsamples;	

		if(pressed)
		{
			do
			{
			_delay_ms(100);
			validsamples = read_Touch_ADC_values ( &x_ADC_values[0] , &y_ADC_values[0]);
			
			}while ( !validsamples  );
			//}while (!( x_ADC_values[0] > valid_ADC_val_xmin && y_ADC_values[0] > valid_ADC_val_ymin &&  x_ADC_values[0] < valid_ADC_val_xmax && y_ADC_values[0] < valid_ADC_val_ymax));
		}
		else
		{
			do
			{
				_delay_ms(100);
			validsamples = read_Touch_ADC_values ( &x_ADC_values[0] , &y_ADC_values[0]);
		
			}while ( validsamples  );			
	
		}
    }   
       
    static void _GetPhysValues(int32_t LogX, int32_t LogY, int32_t * pPhysX, int32_t * pPhysY) 
	{
		static uint8_t displaycount = 0;
	
		uint16_t x_ADC_values[TOUCHCONFIG_ADC_SAMPLES];
		uint16_t y_ADC_values[TOUCHCONFIG_ADC_SAMPLES];

		uint16_t x_read_Filtered;
		uint16_t y_read_Filtered;

		uint8_t validsamples;	

		displaycount++;

		_WaitForPressedState(1);

		_delay_ms(300);
	   	
		do
		{
		
			validsamples = read_Touch_ADC_values ( &x_ADC_values[0] , &y_ADC_values[0]);
		
			switch (displaycount)
			{
				case 0:
					TLC5940_SetGS(1, 1 , 4095);
					break;
				case 1:
					TLC5940_SetGS(3, 6 , 4095);
					break;
				case 2:
					TLC5940_SetGS(8, 3 , 4095);
					break;
			}
			
		}while( !validsamples );

		x_read_Filtered = MedianFilter(&x_ADC_values[0]);  //(x_ADC_values);
		y_read_Filtered = MedianFilter(&y_ADC_values[0]);

		//x_coord_read = x_coord_temp / validvalues;
		//y_coord_read = y_coord_temp / validvalues;
	
		*pPhysX = x_read_Filtered;
		*pPhysY = y_read_Filtered;

	}

	   
