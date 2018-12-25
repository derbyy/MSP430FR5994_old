/*
 * uart.c
 *
 *  Created on: 1. 2. 2018
 *      Author: derby
 */
#include "uart.h"

/*
void uartReceive(char data)
{
    static char rxInProgress = FALSE;
    static char pieceOfString[MAX_STR_LENGTH] = "";           // Holds the new addition to the string
    static char charCnt = 0;

    if( !rxInProgress)
    {
        if ((data != '\n') )
        {
            pieceOfString[0] = '\0';
            rxInProgress = TRUE;
            pieceOfString[0] = data;
            charCnt = 1;
        }
    }
    else
    { // in progress
        if((data == '\n'))
        {
            rxInProgress = FALSE;
            if (newStringReceived == FALSE)
            { // don't mess with the string while main processes it.
                pieceOfString[charCnt]='\0';
                __no_operation();
                charCnt++;
                strncpy(rxString,pieceOfString,charCnt);
                __no_operation();
                newStringReceived = TRUE;
                __no_operation();
            }
        }
        else
        {
            if (charCnt >= MAX_STR_LENGTH)
            {
                rxInProgress = FALSE;
            }
            else
            {
                pieceOfString[charCnt++] = data;
            }
        }
    }
}
*/

