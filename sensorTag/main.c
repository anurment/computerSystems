/*
 * === main.c ===
 * Author: Aleksi Nurmento
 * Embedded device project for a university course.
 * Device: TI Simplelink SensorTag
 * 
 * Only the main.c file is uploaded to the repository since this is the only file that is completely written by the student in the project.
 *
 *
 *
 * This is the main function. Main function contains the functions for the UI.
 * With this code the sensortag can be used to:
 * 1. Measure with the built in accelerometer if the user is using an elevator or taking the stairs to move between floors in a building.
 * 2. Measure pressure and temperature
 * 3. Send and receive messages
 *
 */
 
/*
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>
#include <ti/drivers/i2c/I2CCC26XX.h>


/* Libraries */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

/* Board Header files */
#include "Board.h"

/* Header files */
#include "wireless/comm_lib.h"
#include "sensors/bmp280.h"
#include "sensors/mpu9250.h"

/* Task Stacks */
#define STACKSIZE 2048
#define STACKSIZE2 4096
Char ledTaskStack[STACKSIZE];
Char commTaskStack[STACKSIZE];
Char taskStack[STACKSIZE2];
Char displayTaskStack[STACKSIZE];

/////////////////////////////
// State machine constants //
/////////////////////////////

enum state {IDLAUS = 1, MPU = 2, MPUMITTAUS = 3, MSG1 = 4, MSG2 = 5, MSG_READ = 6,};
enum state myState = IDLAUS;

enum mittaus {PAIKALLAAN, HISSI, PORTAAT};
enum mittaus tulos = PAIKALLAAN;

enum nappi {BTN0 = 1, BTN1 = 2};
enum nappi btnState = BTN0;

///////////////////////
// Message constants //
///////////////////////

char viesti1[16] = "Hyvää joulua!";
char viesti2[16] = "God jul!";
char viesti3[16] = "JEE JEE JEE";

/////////////////////
// Other Constants //
/////////////////////


float keskiarvo = 0.0;

char lampo[20];
char paine[20];
double pres,temp;

char payload[16];   // message buffer




//////////////////////////
// MPU GLOBAL VARIABLES //
//////////////////////////

static PIN_Handle hMpuPin;
static PIN_State MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU9250 uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};


/* Display */
Display_Handle hDisplay;

/* Pin Button1 configured as power button */
static PIN_Handle hPowerButton;
static PIN_State sPowerButton;
PIN_Config cPowerButton[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};
PIN_Config cPowerWake[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE
};

/* Pin Button0 configured as input */
static PIN_Handle hButton0;
static PIN_State sButton0;
PIN_Config cButton0[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/* Leds */
static PIN_Handle hLed;
static PIN_State sLed;
PIN_Config cLed[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
    PIN_TERMINATE
};


// Calculates the standard deviation from accelerometer readings

float calculateSD(float lista[]);

float calculateSD(float lista[]) {
    
    float sum = 0.0, mean, standardDeviation = 0.0;
    int i;
    float sd;
    for(i=0; i<70; ++i)
    {
        sum += lista[i];
    }

    mean = sum/70;

    for(i=0; i<70; ++i)
        standardDeviation += pow(lista[i] - mean, 2);

    
    sd = sqrt(standardDeviation/70);
    return (float)sd;
}
    




// Counting the mean from the accelerometer readings

float keskiarvon_laskenta(float lista[]){
	float summa = 0.0;
	float keskiarvo = 0.0;
	int i = 0;
	for(i = 0; i < 70; i++){
		summa += lista[i];
	}
	keskiarvo = (float)summa/70;
	return (float)keskiarvo;
}


/* Handle for power button */

/* From the lower button you can:
    In IDLAUS state shut down and turn on the device.
    In MPU state turn on and shutdown the Kunto++ measuring
    In MSG1 and MSG2 states send messages
    In MSG_READ state remove the current message from the screen
    
*/
Void powerButtonFxn(PIN_Handle handle, PIN_Id pinId) {

    btnState = BTN1;
    
    switch (myState) {
        case IDLAUS:
            if(btnState == BTN1){
                Display_clear(hDisplay);
                Display_close(hDisplay);
                Task_sleep(100000 / Clock_tickPeriod);

	            PIN_close(hPowerButton);

                PINCC26XX_setWakeup(cPowerWake);
	            Power_shutdown(NULL,0);
                }
        case MPU:
            if(btnState == BTN1){
                myState = MPUMITTAUS;
                System_printf("mpu->mpu_mittaus\n");
                System_flush();
                break;
            }
        case MPUMITTAUS:
            if(btnState == BTN1){
                myState = MPU;
                System_printf("mpu_mittaus->mpu\n");
                System_flush();
                break;
            }
        case MSG1:
            if(btnState == BTN1){
                Send6LoWPAN(0xFFFF, viesti1, strlen(viesti1));
                StartReceive6LoWPAN;
                System_printf(viesti1);
                System_flush();
                break;
            }
        case MSG2:
            if(btnState == BTN1){
                Send6LoWPAN(0xFFFF, viesti2, strlen(viesti2));
                StartReceive6LoWPAN;
                System_printf(viesti2);
                System_flush();
                break;
            }
        case MSG_READ:
            if(btnState == BTN1){
                PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
                memset(payload,0,16);
                System_printf("Viesti poistettu!");
                System_flush();
                break;
            }
            
    }
    btnState = BTN0;

}

/* HANDLER FOR BUTTON0 PRESS */

/*Button0 is used to navigate in the menu: Main screen-> */
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    btnState = BTN1;
    
    switch (myState) {
        case IDLAUS:
            if(btnState == BTN1){
                System_printf("idl->mpu\n");
                System_flush();
                myState = MPU;
                break;
            }
        case MPU:
            if(btnState == BTN1){
                System_printf("mpu->msg1\n");
                System_flush();
                myState = MSG1;
                break;
            }
        case MSG1:
            if(btnState == BTN1){
                System_printf("msg1->msg2\n");
                System_flush();
                myState = MSG2;
                break;
            }
        case MSG2:
            if(btnState == BTN1){
                System_printf("msg2->msg_r\n");
                System_flush();
                myState = MSG_READ;
                break;
            }
        case MSG_READ:
            if(btnState == BTN1){
                System_printf("msg_r->idl\n");
                System_flush();
                myState = IDLAUS;
                break;
            }
        
    
    }
    btnState = BTN0;
}



/* Communication Task */


Void commTask(UArg arg0, UArg arg1) {
    uint16_t senderAddr;
    // Radio to receive mode
	int32_t result = StartReceive6LoWPAN();
	if(result != TRUE) {
		System_abort("Wireless receive mode failed");
	}

    while (1) {
        
        if(GetRXFlag() && myState == MSG_READ) {
            memset(payload,0,16);
            Receive6LoWPAN(&senderAddr, payload, 16);
            System_printf(payload);
            System_flush();
            
            
        }
        
        
    }
    
}



//////////////////////////////////////////////////////////////////////////////////////////
// ledTask for flashing the leds if the user uses stairs during the Kunto++ measurement //
//////////////////////////////////////////////////////////////////////////////////////////

Void ledTask(UArg arg0, UArg arg1) {
    while(1){
        if(tulos == PORTAAT){
            PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
            Task_sleep(2* 100000 / Clock_tickPeriod);
            PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
            Task_sleep(2* 100000 / Clock_tickPeriod);
            PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
            Task_sleep(2* 100000 / Clock_tickPeriod);
            PIN_setOutputValue( hLed, Board_LED0, !PIN_getOutputValue( Board_LED0 ) );
            
        }
        
        Task_sleep(10000 / Clock_tickPeriod);
    }
    
}

//////////////////////////////////////////////////////////////////////////////
// displayTask is defining what is been drawn on the lcd in various states ///
//////////////////////////////////////////////////////////////////////////////


Void displayTask(UArg arg0, UArg arg1) {
    Display_Params displayParams;
	displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
    Display_Params_init(&displayParams);
    int laskuri = 0;

    

    hDisplay = Display_open(Display_Type_LCD, &displayParams);
    if (hDisplay == NULL) {
        System_abort("Error initializing Display\n");
    }
    tContext *pContext = DisplayExt_getGrlibContext(hDisplay);
    
    GrCircleFill(pContext, 48, 70, 6 );
    GrFlush(pContext);
    Display_print0(hDisplay, 5, 1, "Supermittari");
    Display_print0(hDisplay, 6, 5, "3000");
    Task_sleep(5 * 100000 / Clock_tickPeriod);
    Display_clear(hDisplay);
    while(1){
        laskuri = 0;

        Display_clear(hDisplay);
        while(myState == IDLAUS){       //Aloitusvalikon piirto




                Display_print0(hDisplay, 1, 1, "     MENU  >>>");
                if (GetRXFlag()){
                    Display_print0(hDisplay, 3, 2, "Uusi viesti");
                }

                Display_print0(hDisplay, 7, 1, paine);
                Display_print0(hDisplay, 8, 1, lampo);
                GrCircleDraw(pContext, 86, 86, 6);                  // Pixel graphics
                GrFlush(pContext);
                GrLineDraw(pContext,86,86,86,78);
                GrFlush(pContext);
            //    Task_sleep(100000 / Clock_tickPeriod);
        }
        Display_clear(hDisplay);
            
        while(myState == MPU){                  // Kunto++ menu 

            Display_print0(hDisplay, 1, 1, "           >>>"); 
            Display_print0(hDisplay, 4, 1, " KUNTO ++     ");                                                                            
            Display_print0(hDisplay, 10, 1, "         START");
          //  Task_sleep(100000 / Clock_tickPeriod);
    
        }
        
        Display_clear(hDisplay);
        
        while(myState == MPUMITTAUS){                           //Kunto++ measuring on going
            Display_print0(hDisplay, 4, 1, " KUNTO ++  ");
            Display_print0(hDisplay, 10, 1, "         STOP");
                
            if(tulos == PORTAAT){
                    
                Display_clearLines(hDisplay, 5, 7);						//result: stairs
                Display_print0(hDisplay, 6, 1, "PORTAAT");
                Display_print0(hDisplay, 7, 1, "GO GO GO");
                Display_print0(hDisplay, 10, 1, "         STOP");
                laskuri = laskuri + 1;
                if(laskuri < 2){
                    Send6LoWPAN(IEEE80154_SERVER_ADDR, viesti3, strlen(viesti3));
                    StartReceive6LoWPAN;
                    System_printf("Liikkumisen iloa!");
                    System_flush();
                }


            }
            else if(tulos == HISSI){									//result: elevator
                    Display_clearLines(hDisplay,7,7);
                    Display_print0(hDisplay, 6, 1, "HISSI");
                    Display_print0(hDisplay, 10, 1, "         STOP");

            }
            else{														//result: stationary
                
                Display_clearLines(hDisplay,7,7);
                Display_print0(hDisplay, 6, 1, "PAIKALLAAN");
            }

        }
        

        Display_clear(hDisplay);       
        
        while(myState == MSG1){                                         //Msg1
                Display_print0(hDisplay, 1, 1, "           >>>"); 
                Display_print0(hDisplay, 4, 1, "Laheta viesti:");
                Display_print0(hDisplay, 6, 1, "Hyvaa Joulua!");
                Display_print0(hDisplay, 10, 1, "       LAHETA");
             
        }
        Display_clear(hDisplay);   
        
        while(myState == MSG2){                                         //msg2
                Display_print0(hDisplay, 1, 1, "           >>>"); 
                Display_print0(hDisplay, 4, 1, "Laheta viesti:");  
                Display_print0(hDisplay, 6, 1, "God jul!");
                Display_print0(hDisplay, 10, 1, "       LAHETA");
             
        }
        
        Display_clear(hDisplay);
        
        while(myState == MSG_READ){                                     //read msg

                Display_print0(hDisplay, 1, 1, "           >>>"); 
                Display_print0(hDisplay, 4, 1, "Vastaanotettu");
                Display_print0(hDisplay, 5, 1, "viesti:");
                Display_print0(hDisplay, 6, 1, payload);
                Display_print0(hDisplay, 10, 1, "       POISTA");
                
        }
    
    
    Task_sleep(100000 / Clock_tickPeriod);

    }
}

// SENSOR TASK
// task for sensor measurements

Void sensorFxn(UArg arg0, UArg arg1) {

    // *******************************
    //
    // USE TWO DIFFERENT I2C INTERFACES
    //
    // *******************************
	I2C_Handle i2c; // INTERFACE FOR OTHER SENSORS
	I2C_Params i2cParams;
	I2C_Handle i2cMPU; // INTERFACE FOR MPU9250 SENSOR
	I2C_Params i2cMPUParams;

	float ax, ay, az, gx, gy, gz;
	float keskiarvo;
	float keskiarvoz;

	char str[80];
	char str2[80];
	float lista[100];
	float lista2[100];
	int i = 0;
	float sd;

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
    

    // *******************************
    //
    // MPU OPEN I2C
    //
    // *******************************
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // *******************************
    //
    // MPU POWER ON
    //
    // *******************************
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // WAIT 100MS FOR THE SENSOR TO POWER UP
	Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // *******************************
    //
    // MPU9250 SETUP AND CALIBRATION
    //
    // *******************************
	System_printf("MPU9250: Setup and calibration...\n");
	System_flush();

	mpu9250_setup(&i2cMPU);

	System_printf("MPU9250: Setup and calibration OK\n");
	System_flush();

    // *******************************
    //
    // MPU CLOSE I2C
    //
    // *******************************
    I2C_close(i2cMPU);

    // *******************************
    //
    // OTHER SENSOR OPEN I2C
    //
    // *******************************
    i2c = I2C_open(Board_I2C, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }

    // BMP280 SETUP
    bmp280_setup(&i2c);

    // *******************************
    //
    // OTHER SENSOR CLOSE I2C
    //
    // *******************************
    I2C_close(i2c); 
    System_flush();
	while (1) {

	    
		switch (myState) {

            case MPUMITTAUS:
                i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
                if (i2cMPU == NULL) {
                    System_abort("Error Initializing I2CMPU\n");
                }
                
                for(i = 0; i < 100; i++){
                    mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
                    Task_sleep(30000 / Clock_tickPeriod);

                    lista[i] = az;
                    lista2[i] = gz;
                    
                }
                
                    
                keskiarvo = keskiarvon_laskenta(lista);
                keskiarvoz = keskiarvon_laskenta(lista2);
                sd = calculateSD(lista);
           
                sprintf(str,"%f\n" "%f\n" , keskiarvo, sd);
               // sprintf(str2,"%f\n" , keskiarvoz);
            //    System_printf(str2);
              //  System_flush();
                
                
                System_printf(str);
                System_flush();
                if((keskiarvo > -0.995 || keskiarvo < -1.005) && sd < 0.2 ){

                    tulos = HISSI;
                    System_printf("Hissi\n");
                    System_flush();
                }
                    //I2C_close(i2cMPU);
                
                else if(sd > 0.2){
                    tulos = PORTAAT;
                    System_printf("Portaat\n");
                    System_flush();
                }
                else{
                    tulos = PAIKALLAAN;
                    System_printf("paikallaan\n");
                    System_flush();
                }
                

                I2C_close(i2cMPU);
                
                break;
		        
                
            case IDLAUS:                                                //Air pressure and temperature readings for the main menu
                i2c = I2C_open(Board_I2C, &i2cParams);
                if (i2c == NULL) {
                    System_abort("Error Initializing I2C\n");
                }
                
                bmp280_get_data(&i2c, &pres, &temp);
                pres = round(pres);
                temp = round(temp);
                sprintf(paine,"Paine: %.0f hPa\n",pres);
                sprintf(lampo,"Lampo: %.0f C\n",temp);
                System_printf(paine);
                System_flush();
                System_printf(lampo);
                System_flush();
                I2C_close(i2c);
                Task_sleep(500000 / Clock_tickPeriod);
                break;
            

	    }

	    // *******************************
	    //
	    // MPU CLOSE I2C
	    //
	    // *******************************
	   // I2C_close(i2cMPU);

	    // WAIT 100MS
    	Task_sleep(100000 / Clock_tickPeriod);
	}

	// MPU9250 POWER OFF 
	//     Because of loop forever, code never goes here
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}
//
//
//



/* Main loop */

Int main(void) {

    // Task variables
	Task_Handle hLedTask;
	Task_Params ledTaskParams;
	Task_Handle hCommTask;
	Task_Params commTaskParams;
	Task_Handle task;
	Task_Params taskParams;
	Task_Handle hdisplayTask;
	Task_Params displayTaskParams;

    // Initialize board
    Board_initGeneral();
    Board_initI2C();

	/* Power Button */
	hPowerButton = PIN_open(&sPowerButton, cPowerButton);
	if(!hPowerButton) {
		System_abort("Error initializing power button shut pins\n");
	}
	if (PIN_registerIntCb(hPowerButton, &powerButtonFxn) != 0) {
		System_abort("Error registering power button callback function");
	}

    /* BUTTON0 */
    hLed = PIN_open(&sLed, cLed);
    if(!hLed) {
        System_abort("Error initializing LED pins\n");
    }
    hButton0 = PIN_open(&sButton0, cButton0);
    if(!hButton0) {
        System_abort("Error initializing button pins\n");
        
    }
    
    if (PIN_registerIntCb(hButton0, &buttonFxn) != 0) {
        System_abort("Error registering button callback function");
   }


    /* Init Main Task */                                                         // HUOM! labtask poistettu
    Task_Params_init(&ledTaskParams);
    ledTaskParams.stackSize = STACKSIZE;
    ledTaskParams.stack = &ledTaskStack;
    ledTaskParams.priority=2;

    hLedTask = Task_create(ledTask, &ledTaskParams, NULL);
    if (hLedTask == NULL) {
    	System_abort("Task create failed!");
    }

//  display Task
    Task_Params_init(&displayTaskParams);
    displayTaskParams.stackSize = STACKSIZE;
    displayTaskParams.stack = &displayTaskStack;
    displayTaskParams.priority=3;
    hdisplayTask = Task_create(displayTask, &displayTaskParams, NULL);
    if (hdisplayTask == NULL) {
    	System_abort("Task create failed!");
    }
   
    
 // sensorFxn task
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
    	System_abort("Pin open failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE2;
    taskParams.stack = &taskStack;
    taskParams.priority=2;
    
    task = Task_create((Task_FuncPtr)sensorFxn, &taskParams, NULL);
    if (task == NULL) {
    	System_abort("Task create failed!");
    }

    /* Init Communication Task */
    Init6LoWPAN();

    Task_Params_init(&commTaskParams);
    commTaskParams.stackSize = STACKSIZE;
    commTaskParams.stack = &commTaskStack;
    commTaskParams.priority=1;
    
    hCommTask = Task_create(commTask, &commTaskParams, NULL);
    if (hCommTask == NULL) {
    	System_abort("Task create failed!");
    }
    


    // Send hello to console
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

