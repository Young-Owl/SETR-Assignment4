#include "unity.h"
#include "string.h"

/* Defines for return codes of the functions */
#define CMD_SUCCESS       0                     /**<Return value when a correct command is read.*/
#define CMD_ERROR_STRING -1                     /**<Return value when an empty string or incomplete command was found.*/
#define CMD_INVALID      -2                     /**<Return value when an invalid command was found.*/
#define CS_ERROR         -3                     /**<Return value when a CS error is detected.*/
#define STR_WRONG_FORMAT -4                     /**<Return value when the string format is wrong.*/             

#define MAX_CMDSTRING_SIZE 20                   /**<Maximum size of the command string.*/ 
#define SOF_SYM '#'	                            /**<Start of Frame Symbol (#)*/
#define EOF_SYM 0xd                             /**<End of Frame Symbol (\r)*/

#define RXBUF_SIZE 200                              /**<RX buffer size */
#define TXBUF_SIZE 200                              /**<TX buffer size */
#define RX_TIMEOUT 1000                             /**<Inactivity period after the instant when last char was received that triggers an rx event (in us) */

volatile uint16_t i2cThreadPeriod = 1000;
volatile uint16_t ledThreadPeriod = 1000;
volatile uint16_t btnThreadPeriod = 1000;

volatile char cmdString[MAX_CMDSTRING_SIZE];    /**<Command string buffer. */
static unsigned char cmdStringLen = 0;          /**<Length of the command string. */

char msg[MAX_CMDSTRING_SIZE + 5];
int sizee;

int cmdProcessor(void);
void newCmdChar(unsigned char newChar, int index);

struct miniData {
    uint8_t led[4];                     /**<Led state vector */
    uint8_t buttonState[4];             /**<Button state vector */
    uint16_t temp;                      /**<Temperature value */
};

/* Initialize the miniData structure variables */
struct miniData miniData = {
    .led = {0,0,0,0},
    .buttonState = {0,0,0,0},
    .temp = 0
};

void setUp(void)
{
	return;
}
void tearDown(void)
{
	return;
}

void test_CMD_CmdProcess_CommandFound(void)
{
    // LF
	newCmdChar('#', 0);
    newCmdChar('L', 1);
    newCmdChar('F', 2);
    newCmdChar('1', 3);
    newCmdChar('\r', 4);
    cmdStringLen = 5;
	TEST_ASSERT_EQUAL_INT(CMD_SUCCESS, cmdProcessor());

    //BF
    newCmdChar('#', 0);
    newCmdChar('B', 1);
    newCmdChar('F', 2);
    newCmdChar('1', 3);
    newCmdChar('\r', 4);
    cmdStringLen = 5;
	TEST_ASSERT_EQUAL_INT(CMD_SUCCESS, cmdProcessor());

    //TF
    newCmdChar('#', 0);
    newCmdChar('T', 1);
    newCmdChar('F', 2);
    newCmdChar('1', 3);
    newCmdChar('\r', 4);
    cmdStringLen = 5;
	TEST_ASSERT_EQUAL_INT(CMD_SUCCESS, cmdProcessor());

    //L1S1
    newCmdChar('#', 0);
    newCmdChar('L', 1);
    newCmdChar('1', 2);
    newCmdChar('S', 3);
    newCmdChar('1', 4);
    newCmdChar('\r', 5);
    cmdStringLen = 6;
	TEST_ASSERT_EQUAL_INT(CMD_SUCCESS, cmdProcessor());

    //B1
    newCmdChar('#', 0);
    newCmdChar('B', 1);
    newCmdChar('1', 2);
    newCmdChar('\r', 3);
    cmdStringLen = 4;
	TEST_ASSERT_EQUAL_INT(CMD_SUCCESS, cmdProcessor());

    //T
    newCmdChar('#', 0);
    newCmdChar('T', 1);
    newCmdChar('\r', 2);
    cmdStringLen = 3;
    TEST_ASSERT_EQUAL_INT(CMD_SUCCESS, cmdProcessor());

}

void test_CMD_CmdProcess_CommandNotFound(void)
{
	// LF
	newCmdChar('#', 0);
    newCmdChar('A', 1);
    newCmdChar('F', 2);
    newCmdChar('1', 3);
    newCmdChar('\r', 4);
    cmdStringLen = 5;
	TEST_ASSERT_EQUAL_INT(CMD_INVALID, cmdProcessor());
}

void test_CMD_CmdProcess_ErrorParameters(void)
{
	// B4
	newCmdChar('#', 0);
    newCmdChar('B', 1);
    newCmdChar('5', 2);
    newCmdChar('\r', 3);
    cmdStringLen = 4;
	TEST_ASSERT_EQUAL_INT(STR_WRONG_FORMAT, cmdProcessor());
}

void test_CMD_CmdProcess_ErrorEOF(void)
{
	// T
	newCmdChar('#', 0);
    newCmdChar('T', 1);
    cmdStringLen = 2;
	TEST_ASSERT_EQUAL_INT(CMD_ERROR_STRING, cmdProcessor());

}

void test_CMD_CmdProcess_ErrorSOF(void)
{
	// LF
    newCmdChar('L', 0);
    newCmdChar('F', 1);
    newCmdChar('1', 2);
    newCmdChar('\r', 3);
    cmdStringLen = 4;
	TEST_ASSERT_EQUAL_INT(STR_WRONG_FORMAT, cmdProcessor());

}

int main(void)
{
	UNITY_BEGIN();
	
	RUN_TEST(test_CMD_CmdProcess_CommandFound);
	RUN_TEST(test_CMD_CmdProcess_CommandNotFound);
	RUN_TEST(test_CMD_CmdProcess_ErrorParameters);
	RUN_TEST(test_CMD_CmdProcess_ErrorEOF);
	RUN_TEST(test_CMD_CmdProcess_ErrorSOF);
		
	return UNITY_END();
}

int cmdProcessor(void)
{
	int i = 0, cmdStringRemain = 0;         /* Variables to store the index of the SOF and the remaining chars in the cmdString. */
    char F[3] = {};                         /* Variables to store the Frequency values for each char.*/
    int freq = 0;                           /* Variable to store the frequency value.*/

	/* Detect empty cmd string */
	if(cmdStringLen == 0)
		return CMD_ERROR_STRING; 
	
	/* Find index of SOF */
	for(i=0; i < cmdStringLen; i++) {
        if(cmdString[i] == SOF_SYM) {
			break;
		}
	}
	
	/* If a SOF was found look for commands */
	if(i < cmdStringLen) {
		if(cmdString[i+1] == 'L' && cmdString[i+2] == 'F') { /* LF command detected */ 
            cmdStringRemain = cmdStringLen - 4;              /* 4 non-relevant characters ('#','L','F' and "enter") */ 

            if(cmdStringRemain > 3) {                        /* 3 maximum digits for frequency */
                return STR_WRONG_FORMAT;
            }

            for(int k = 0; k < cmdStringRemain; k++) {
                F[k] = cmdString[k+3];
                if(F[k] < '0' || F[k] > '9') {               /* Check value of constants*/
				    return STR_WRONG_FORMAT;
			    }
            }
            
            /* Check character of EOF*/
			if(cmdString[cmdStringLen-1] != EOF_SYM){
				return CMD_ERROR_STRING;
			}

            /* Calculates frequency */
            for(int k = 0; k < cmdStringRemain; k++) {
                if (cmdStringRemain == 3){
                    if(k == 0){
                        freq += (F[k] - '0') * 100;
                    }
                    else if(k == 1){
                        freq += (F[k] - '0') * 10;
                    }
                    else if(k == 2){
                        freq += (F[k] - '0');
                    }        
                }
                if (cmdStringRemain == 2){
                    if(k == 0){
                        freq += (F[k] - '0') * 10;
                    }
                    else if(k == 1){
                        freq += (F[k] - '0');
                    }
                }
                if (cmdStringRemain == 1){
                    freq += (F[k] - '0');
                }
            }

            /* Change Led Thread Period */
            ledThreadPeriod = 1.0/freq*1000;

			return CMD_SUCCESS;
		}

        if(cmdString[i+1] == 'B' && cmdString[i+2] == 'F') { /* BF command detected */
            cmdStringRemain = cmdStringLen - 4;              /* 4 non-relevant characters ('#','B','F' and "enter") */ 

            if(cmdStringRemain > 3) {                        /* 3 maximum digits for frequency*/
                return STR_WRONG_FORMAT;
            }

            for(int k = 0; k < cmdStringRemain; k++) {
                F[k] = cmdString[k+3];
                if(F[k] < '0' || F[k] > '9') {               /* Check value of constants*/
				    return STR_WRONG_FORMAT;
			    }
            }
            
            /* Check character of EOF*/
			if(cmdString[cmdStringLen-1] != EOF_SYM){
				return CMD_ERROR_STRING;
			}

            /* Calculates frequency */
            for(int k = 0; k < cmdStringRemain; k++) {
                if (cmdStringRemain == 3){
                    if(k == 0){
                        freq += (F[k] - '0') * 100;
                    }
                    else if(k == 1){
                        freq += (F[k] - '0') * 10;
                    }
                    else if(k == 2){
                        freq += (F[k] - '0');
                    }        
                }
                if (cmdStringRemain == 2){
                    if(k == 0){
                        freq += (F[k] - '0') * 10;
                    }
                    else if(k == 1){
                        freq += (F[k] - '0');
                    }
                }
                if (cmdStringRemain == 1){
                    freq += (F[k] - '0');
                }
            }

            /* Change Button Thread Period */
            btnThreadPeriod = 1.0/freq*1000;

			return CMD_SUCCESS;
		}

        if(cmdString[i+1] == 'T' && cmdString[i+2] == 'F') { /* TF command detected */
            cmdStringRemain = cmdStringLen - 4;              /* 4 non-relevant characters ('#','T','F' and "enter") */ 

            if(cmdStringRemain > 3) {                        /* 3 maximum digits for frequency*/
                return STR_WRONG_FORMAT;
            }

            for(int k = 0; k < cmdStringRemain; k++) {
                F[k] = cmdString[k+3];
                if(F[k] < '0' || F[k] > '9') {               /* Check value of constants*/
				    return STR_WRONG_FORMAT;
			    }
            }
			
            /* Check character of EOF*/
			if(cmdString[cmdStringLen-1] != EOF_SYM){
				return CMD_ERROR_STRING;
			}

            /* Calculates frequency */
            for(int k = 0; k < cmdStringRemain; k++) {
                if (cmdStringRemain == 3){
                    if(k == 0){
                        freq += (F[k] - '0') * 100;
                    }
                    else if(k == 1){
                        freq += (F[k] - '0') * 10;
                    }
                    else if(k == 2){
                        freq += (F[k] - '0');
                    }        
                }
                if (cmdStringRemain == 2){
                    if(k == 0){
                        freq += (F[k] - '0') * 10;
                    }
                    else if(k == 1){
                        freq += (F[k] - '0');
                    }
                }
                if (cmdStringRemain == 1){
                    freq += (F[k] - '0');
                }
            }

            i2cThreadPeriod = 1.0/freq*1000;

			return CMD_SUCCESS;
		}
		
        if(cmdString[i+1] == 'L' && cmdString[i+3] == 'S') { /* LxSy command detected, x=[1,2,3,4] and y=[0,1] */
            if(cmdString[i+2] < '1' || cmdString[i+2] > '4') {   /* Check value of LED number */
                    return STR_WRONG_FORMAT;
            }

            if(cmdString[i+4] < '0' || cmdString[i+4] > '1') {   /* Check value of LED state */
                    return STR_WRONG_FORMAT;
            }
			
            /* Check character of EOF*/
			if(cmdString[cmdStringLen-1] != EOF_SYM){
				return CMD_ERROR_STRING;
			}

            /* Update LED State */
            miniData.led[cmdString[i+2] - '0' - 1] = cmdString[i+4] - '0';
			return CMD_SUCCESS;
		}

        if(cmdString[i+1] == 'B' && cmdStringLen == 4) {     /* Bx command detected */
            if(cmdString[i+2] < '1' || cmdString[i+2] > '4') {    /* Check value of Button number */
                    return STR_WRONG_FORMAT;
            }

            /* Check character of EOF*/
			if(cmdString[cmdStringLen-1] != EOF_SYM){
				return CMD_ERROR_STRING;
			}
            

			return CMD_SUCCESS;
		}

        if(cmdString[i+1] == 'T') {                          /* T command detected */
            /* Check character of EOF*/
			if(cmdString[cmdStringLen-1] != EOF_SYM){
				return CMD_ERROR_STRING;
			}


			return CMD_SUCCESS;
		}
        
		return CMD_INVALID;	/* No valid command found */
	}
	/* cmd string not null and SOF not found */
	return STR_WRONG_FORMAT;
}

void newCmdChar(unsigned char newChar, int index)
{  
    cmdString[index] = newChar;
}