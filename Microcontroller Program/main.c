/*
 * COMPENG 2DX3 Deliverable 2 Code
 * Written by Tirth Nagar
 */

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include "init.h"

// PROJECT CONSTANTS / VARIABLES
// PN0 / D2 - Measurement Status LED
// PN1 / D1 - Additional Status LED
// PB2 --> SCL
// PB3 --> SDA

#define BUTTON 0b00000010
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0
#define SAMPLES 32
#define NUM_SCANS 3

int distances[SAMPLES * NUM_SCANS];
char motorSteps[] = {0b00000011, 0b00000110, 0b00001100, 0b00001001};
uint16_t dev = 0x29; // address of the ToF sensor as an I2C slave peripheral
int status = 0;
int spinMotor = 0;
int delay = 160000;

// Function to prove bus speed
void prove_bus(){
	while (1) {
		GPIO_PORTH_DATA_R=0b00001111;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R=0b00000000;
		SysTick_Wait10ms(1);  
 
	}
}

// Function to spin the motor
void spin(int steps, int direction) {
    for (int j = 0; j < steps; j++) {
        if (direction == CLOCKWISE) {
            for (int k = 0; k < 4; k++) {
                GPIO_PORTH_DATA_R = motorSteps[k];
                SysTick_Wait(delay);
            }
        } else if (direction == COUNTER_CLOCKWISE) {
            for (int k = 3; k >= 0; k--) {
                GPIO_PORTH_DATA_R = motorSteps[k];
                SysTick_Wait(delay);
            }
        }
    }
}

// Interrupt handler for GPIOJ
void GPIOJ_IRQHandler(void) {
    spinMotor = ~spinMotor;
		FlashLED1(1); // spin motor on
    // Acknowledge flag by setting proper bit in ICR register
    GPIO_PORTJ_ICR_R = 0x02;
}

// Function to take measurement
void take_measurement() {
    uint16_t Distance;
    uint8_t dataReady;
    uint8_t RangeStatus;
		static int i;
	
    if (spinMotor) {
		for (int j = 0; j < SAMPLES; j++) {
			while (dataReady == 0) {
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					FlashLED2(1);
					VL53L1_WaitMs(dev, 2);
			}

			dataReady = 0;
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);
			FlashLED2(1);

			distances[i] = Distance;
			status = VL53L1X_ClearInterrupt(dev);
			
			i++;
			spin(512 / SAMPLES, CLOCKWISE);
			SysTick_Wait10ms(25);
		}
	}
}

int main(void)
{
    char signal;
		uint8_t sensorState = 0; // useful variables
		
    // initialize
    PLL_Init();
    PortH_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortJ_Init();
    PortJ_Interrupt_Init();
	
		prove_bus(); // Run this function to demonstrate bus speed

    // initialize the sensor
    while (sensorState == 0)
    {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(25);
    }

    FlashAllLEDs();

    status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);

    status = VL53L1X_StartRanging(dev); /* This function has to be called to enable the ranging */
    Status_Check("StartRanging", status);
		
    // Startup Sequence to ensure both micro and pc are connected
    while(1){
			signal = UART_InChar(); // wait for acknowledgement from PC
			if (signal == 0x31)
			{
				FlashLED1(1);
				signal = 0;
				break;
			}
		}
		
		// Take all required measurements and as many scans as required
		for(int j = 0; j < NUM_SCANS; j++){
			take_measurement();
			spin(512, COUNTER_CLOCKWISE);
			FlashLED1(1);
			SysTick_Wait10ms(5);
		}
		
		// Wait for acknowledgment from Python
    while (1) {
			char signal = UART_InChar();
      if (signal == '1') {
				break;
      }
		}

		// Send measurement to Python
		for (int i = 0; i < SAMPLES * NUM_SCANS; i++) {
				// Print values to UART one by one from array
				sprintf(printf_buffer, "%u\r\n", distances[i]);
				UART_printf(printf_buffer); // Send the measurement
		}
}