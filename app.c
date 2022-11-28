/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Mc32DriverLcd.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
S_ADCResults AdcResult;
int count = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        //Etat Initialisation
        case APP_STATE_INIT:
        {
            lcd_init();                         //Init LCD
            
            printf_lcd("Tp0 Led+AD 2022-23");       
            lcd_gotoxy(1,2);                    //Ecrire sur la deuxieme ligne
            printf_lcd("Santiago Valiante");
           
            lcd_bl_on();                        //Allumer le LCD
       
            BSP_InitADC10();                    //Initialisation ADC
            
            AllLED_ON_OFF(1);                   //Allumer toutes les LEDs (1=on)
            
            DRV_TMR0_Start ();                  //starte timer 0 (100ms)
         
            appData.state = APP_STATE_WAIT;
            
            break;
        }
        
        //Etat Wait
        case APP_STATE_WAIT:
        {
            // rien à faire ici !
            break;
        }        
        

        case APP_STATE_SERVICE_TASKS:
        {
            AllLED_ON_OFF(0);                   //Eteindre toutes les LEDs (0=off)
            
            AdcResult = BSP_ReadAllADC();       //Lecture des ADC
            
            lcd_gotoxy(1,3);                    //Ecrire sur la troisieme ligne
            
            printf_lcd("Ch0 %04d Ch1 %04d", AdcResult.Chan0, AdcResult.Chan1);
             
            Chenillard(count);                  //Appel de la fonction Chenillard
            
            //Gestion du compteur du chenillard
            if (count > 7)                      
            {
                count = 0;  //Remettre le compteur à 0
            }
            else
            {
                count++;    //Incrémenter le compteur 
            }
            
            appData.state = APP_STATE_WAIT;    //Aller dans l'etat Wait  
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

//Fonction pour mettre à jour les états
void APP_UpdateState(APP_STATES newState)
{
    appData.state = newState; 
}
 
//Fonction pour allumer et eteindre toutes les leds
void AllLED_ON_OFF (int on_off)
{
    //on_off = 1 (Leds allumées))
    if (on_off == 1)
    {
        BSP_LEDOn(BSP_LED_0);
        BSP_LEDOn(BSP_LED_1); 
        BSP_LEDOn(BSP_LED_2);
        BSP_LEDOn(BSP_LED_3);
        BSP_LEDOn(BSP_LED_4);
        BSP_LEDOn(BSP_LED_5); 
        BSP_LEDOn(BSP_LED_6);
        BSP_LEDOn(BSP_LED_7);          
    }
    //on_off = 0 (Leds éteintes))
    else
    {
        BSP_LEDOff(BSP_LED_0);
        BSP_LEDOff(BSP_LED_1); 
        BSP_LEDOff(BSP_LED_2);
        BSP_LEDOff(BSP_LED_3);
        BSP_LEDOff(BSP_LED_4);
        BSP_LEDOff(BSP_LED_5); 
        BSP_LEDOff(BSP_LED_6);
        BSP_LEDOff(BSP_LED_7);         
    }     
}

//Fonction chenillard
void Chenillard (int count)
{
    //Gestion des leds suivant l'etat du compteur
    switch (count)
    {
        case 0:
            BSP_LEDOff(BSP_LED_7);
            BSP_LEDOn(BSP_LED_0);            
            break;
            
        case 1:            
            BSP_LEDOff(BSP_LED_0);
            BSP_LEDOn(BSP_LED_1);
            break;
            
        case 2:
            BSP_LEDOff(BSP_LED_1);
            BSP_LEDOn(BSP_LED_2);
            break;
            
        case 3:
            BSP_LEDOff(BSP_LED_2);
            BSP_LEDOn(BSP_LED_3);
            break;
            
        case 4:
            BSP_LEDOff(BSP_LED_3);
            BSP_LEDOn(BSP_LED_4);
            break;
            
        case 5:
            BSP_LEDOff(BSP_LED_4);
            BSP_LEDOn(BSP_LED_5);
            break;
            
        case 6:
            BSP_LEDOff(BSP_LED_5);
            BSP_LEDOn(BSP_LED_6);
            break;
            
        case 7:
            BSP_LEDOff(BSP_LED_6);
            BSP_LEDOn(BSP_LED_7);
            break;
    }
}

/*******************************************************************************
 End of File
 */
