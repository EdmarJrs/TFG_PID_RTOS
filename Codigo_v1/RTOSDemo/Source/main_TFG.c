/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "adc.h"
#include "timer16.h"
#include "uart.h"

/* Hardware specific includes. */
#include "lpc11Uxx.h"

/*Macros*/

# define TRUE  1
# define FALSE 0
# define period 1024  //Period for PWM

//xQueueHandle fila_msg;

/*Global Variables*/
uint16_t ADC0_Value = 0;
uint16_t ADC1_Value = 0;
uint8_t pBuf[50];
float kp = 10; //Proportional constant of PID
float ki = 10; //Integral constant of PID
float kd = 0; //Derivative constant of PID
float sp = 0; //Setpoint
float T = 0;  //Amostrage Time
/*
 void Task_LCD(void *pvParameters)
 {	oled_clearScreen(OLED_COLOR_WHITE);
 oled_putString(1, 1, (uint8_t*) " KP=  ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
 oled_putString(1, 9, (uint8_t*) " KI=  ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
 oled_putString(1, 17, (uint8_t*) " KD=  ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
 oled_putString(1, 25, (uint8_t*) " SP=  ", OLED_COLOR_BLACK,OLED_COLOR_WHITE);
 oled_putString(1, 33, (uint8_t*) " T=  ", OLED_COLOR_BLACK,OLED_COLOR_WHITE);
 for(;;)
 {
 //kp = ADCRead(0);
 floatToString(kp, buf, 30, 10);
 oled_fillRect((1+9*3),1, (1+9*10), 9, OLED_COLOR_WHITE);
 oled_putString((1+9*3),1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

 floatToString(ki, buf, 30, 10);
 oled_fillRect((1+9*3),9, (1+9*10), 17, OLED_COLOR_WHITE);
 oled_putString((1+9*3),9, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

 floatToString(kd, buf, 30, 10);
 oled_fillRect((1+9*3),17, (1+9*10), 25, OLED_COLOR_WHITE);
 oled_putString((1+9*3),17, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

 floatToString(sp, buf, 30, 10);
 oled_fillRect((1+9*3),25, (1+9*10), 33, OLED_COLOR_WHITE);
 oled_putString((1+9*3),25, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

 floatToString(T, buf, 30, 10);
 oled_fillRect((1+9*3),33, (1+9*10), 41, OLED_COLOR_WHITE);
 oled_putString((1+9*3),33, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

 vTaskDelay(1000);
 }
 }
 */
void Task_GET_Uart(void *pvParameters) {
	(void) pvParameters;
	float valor = 0;
	uint8_t estado = 0;
	uint8_t tipo = TRUE;
	float cont = 0.1;
	uint8_t data = 0;
	float kp_aux, ki_aux, kd_aux, T_aux, sp_aux;
	uint8_t status = FALSE;
	for (;;) {
		data = 0;
		if (UARTReceive(&data, 1, FALSE)) {
			switch (data) {
			case 0x2A:
				estado = 1;
				status = FALSE;
				break;
			case 0x50:
				if (estado == 1) {
					estado = 2;
					tipo = TRUE;
					valor = 0.0;
				}
				break;
			case 0x49:
				if (estado == 2) {
					estado = 3;
					tipo = TRUE;
					kp_aux = valor;
					valor = 0.0;
				}
				break;
			case 0x44:
				if (estado == 3) {
					estado = 4;
					tipo = TRUE;
					ki_aux = valor;
					valor = 0.0;
				}
				break;
			case 0x53:
				if (estado == 4) {
					estado = 5;
					tipo = TRUE;
					kd_aux = valor;
					valor = 0.0;
				}
				break;
			case 0x54:
				if (estado == 5) {
					estado = 6;
					tipo = TRUE;
					sp_aux = valor;
					valor = 0.0;
				}
				break;
			case 0x2E:
				tipo = FALSE;
				cont = 0.1;
				break;
			case 0x23:
				if (estado == 6) {
					estado = 0;
					T_aux = valor;
					valor = 0.0;
					status = TRUE;
				}
				break;
			default:
				if ((data == 48) || (data == 49) || (data == 50) || (data == 51)
						|| (data == 52) || (data == 53) || (data == 54)
						|| (data == 55) || (data == 56) || (data == 57)) {
					if ((tipo == TRUE) && (estado != 0) && (estado != 1)) {
						valor = valor * 10 + (data - 48);
					} else {
						if ((tipo == FALSE) && (estado != 0) && (estado != 1)) {
							valor = valor + cont * (data - 48);
							cont = cont * 0.1;
						} else {
							estado = 0;
						}
					}
				} else {
					estado = 0;
				}
			}
		}

		if (status == TRUE) {
			status = FALSE;
			kp = kp_aux;
			ki = ki_aux;
			kd = kd_aux;
			sp = sp_aux;
			T = T_aux;
		}
		vTaskDelay(100);
	}
}

void Task_GET_ADC(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		ADC0_Value = 0;
		ADC1_Value = 0;
		ADC0_Value = ADCRead(0);   //AD0  //11 clocks to read
		ADC1_Value = ADCRead(1);  //AD1
		vTaskDelay(1000);  //Get ADC in all amostrage time
	}
}
void Task_SEND_PWM(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		ADC0_Value = 0;
		ADC1_Value = 0;
		ADC0_Value = ADCRead(0);   //AD0  //11 clocks to read
		ADC1_Value = ADCRead(1);  //AD1

		//Has been used ADC Value for PWM, for while
		setMatch_timer16PWM(0, 1, 1024 - ADC0_Value);  //Set offset PWM0
		setMatch_timer16PWM(1, 1, 1024 - ADC1_Value);  //Set offset PWM1
		vTaskDelay(1000);  //Set PWM in all amostrage time
	}
}

void main_TFG() {
	//System clock configuration
	SystemInit();  //Configure the system to 48MHz
	SystemCoreClockUpdate();

	//PWM configurations
	//1024  - 3.3V
	//0     - 0.0V
	init_timer16PWM(0, period, MATCH1, 0);	//inicialize PWM0   PIO0_9  //
	init_timer16PWM(1, period, MATCH1, 0);	//inicialize PWM1   PIO0_22  //
	enable_timer16(0);
	enable_timer16(1);

	//ADC Configurations
	//3.3V  - 1024
	//0.0V  -  0
	//AD0-  PIO0_11
	//AD1-  PIO0_12
	//OBS: It's necessary to disconnect Jumpers Qa and Red Led in the board
	ADCInit(4500000); //ADC clock need to be 4.5MHz or less

	//UART Configurations
	//PIO0_18  -RX
	//PIO0_19  -TX
	UARTInit(9600);

	//Task Configurations
	/*xTaskCreate(Task_LCD, (signed char * ) "Task_LCD", configMINIMAL_STACK_SIZE+50,
	 NULL, (tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);*/

	//test

	T = 0.01;
	//end test
	//Task for UART
	xTaskCreate(Task_GET_Uart, (signed char *) "GET_Uart",
			configMINIMAL_STACK_SIZE,
			NULL, (tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

	//Task for ADC
	xTaskCreate(Task_GET_ADC, (signed char *) "Task_GET_ADC",
			configMINIMAL_STACK_SIZE,
			NULL, (tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

	//Task for PWM
	xTaskCreate(Task_SEND_PWM, (signed char *) "Task_SEND_PWM",
			configMINIMAL_STACK_SIZE,
			NULL, (tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

	// Start the scheduler
	vTaskStartScheduler();

	while (1) {
		//If has been all OK, never will be here
	}

}

