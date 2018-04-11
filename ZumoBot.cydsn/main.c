/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

#include <project.h>
#include <stdio.h>
#include "Systick.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "I2C_made.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "IR.h"
#include "Ambient.h"
#include "Beep.h"
#include <time.h>
#include <sys/time.h>
int rread(void);


/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

// Main program entry point //

#if 0
// Development goes here

#define TIMELIMIT 50000

int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Systick_Start();
    
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    struct sensors_ dig;
    
    bool BatteryLed_Status = false;
    bool Motor_Status = false;
    
    
    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    
    uint timer = 0;                                 // Time measurement
    uint8 leftspeed = 0;                            // Left track speed
    uint8 rightspeed = 0;                           // Right track speed
    const uint8 travelspeed = 100;                  // Maximum travel speed
    const uint8 small_correction = 20;              // Speed adjustment for small corrections
    const uint8 large_correction = 60;              // Speed adjustment for larger corrections
    uint forward_counter = 0;                       // Continuous time without direction adjustment
    
    //uint8 left_status = 0;
    //uint8 right_status = 0;
    //uint left_time = 0;
    //uint right_time = 0;
    //bool turning = false;
    
    uint8 line_counter = 0;                         // Crossing lines pass. Zumo stops after the 3rd line
    bool line_found = false;                        // If zumo have found a crossing line
    
    CyDelay(2000);                                  // Ready, steady...
    
    for(;;)
    {
        // Check battery status
        ADC_Battery_StartConvert();
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for get ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            volts = (float)adcresult / 4095 * 5;
            volts *= 1.5f;
            if ((volts <= 4.0f) && (!BatteryLed_Status))
            {
                BatteryLed_Write(1);
                BatteryLed_Status = true;
                motor_stop();               // Failsafe: motor stoped if voltage gets too low
            }
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        // Check battery status - End
        
        if (!Motor_Status && timer <= TIMELIMIT)    // ...go!
        {
            motor_start();              // motor start
            Motor_Status = true;
            leftspeed = travelspeed;
            rightspeed = travelspeed;
        }

        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        //printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
        //print out 0 or 1 according to results of reflectance period

        
        if (dig.l1 && dig.r1)       // Left - black Right - black Bring back to straight / Increase speed
        {
            if ((forward_counter != 0) && (forward_counter % 500 == 0))
            {
                if (leftspeed < rightspeed && leftspeed < travelspeed - small_correction) leftspeed += small_correction;
                if (rightspeed < leftspeed && rightspeed < travelspeed - small_correction) rightspeed += small_correction;
            }
            
            //if (leftspeed < rightspeed && leftspeed < travelspeed - large_correction) leftspeed += large_correction;
            //if (rightspeed < leftspeed && rightspeed < travelspeed - large_correction) rightspeed += large_correction;
            
            if ((forward_counter != 0) && (forward_counter % 500 == 0))
            {
                if (leftspeed == rightspeed)
                {
                    if (leftspeed < travelspeed - small_correction)
                    {
                        leftspeed += small_correction;
                        rightspeed += small_correction;
                    }
                }
            }
        }
        
        if (dig.l1 && !dig.r1)      // Left - black Right - white Turn leftward
        {
            if (leftspeed > small_correction) leftspeed -= small_correction;
            if (rightspeed < travelspeed - small_correction) rightspeed += small_correction;
            forward_counter = 0;
        }
        
        if (!dig.l1 && dig.r1)      // Left - white Right - black Turn rightward
        {
            if (leftspeed < travelspeed - small_correction) leftspeed += small_correction;
            if (rightspeed > small_correction) rightspeed -= small_correction;
            forward_counter = 0;
        }
        
        if (dig.l2 && !dig.r2)
        {
            /*if (leftspeed > large_correction) leftspeed -= large_correction;
            else if (leftspeed > small_correction) leftspeed -= small_correction;
            if (rightspeed < travelspeed - large_correction) rightspeed += large_correction;
            else if (rightspeed < travelspeed - small_correction) rightspeed += small_correction;*/
            leftspeed = 0;
            rightspeed = travelspeed;
            forward_counter = 0;
        }
        
        if (dig.r2 && !dig.l2)
        {
            /*if (rightspeed > large_correction) rightspeed -= large_correction;
            else if (rightspeed > small_correction) rightspeed -= small_correction;
            if (leftspeed < travelspeed - large_correction) leftspeed += large_correction;
            else if (leftspeed < travelspeed - small_correction) leftspeed += small_correction;*/
            leftspeed = travelspeed;
            rightspeed = 0;
            forward_counter = 0;
        }
        
        
        /*
        if (!(dig.l1 || dig.r1))    // Left - white Right - white Stop
        {
            leftspeed = 0;
            rightspeed = 0;
            if (left_status == 1)
            {
                MotorDirLeft_Write(1);      // set LeftMotor forward mode
                MotorDirRight_Write(0);     // set RightMotor forward mode
                PWM_WriteCompare1(100); 
                PWM_WriteCompare2(100);
                while (!(dig.r1))
                {
                    reflectance_digital(&dig);
                }
                left_status = 0;
                right_status = 0;
                left_time = timer;
                right_time = timer;
                
                leftspeed = travelspeed;
                rightspeed = travelspeed;
            }
            else
            if (right_status == 1)
            {
                MotorDirLeft_Write(0);      // set LeftMotor forward mode
                MotorDirRight_Write(1);     // set RightMotor forward mode
                PWM_WriteCompare1(100); 
                PWM_WriteCompare2(100);
                while (!(dig.l1))
                {
                    reflectance_digital(&dig);
                }
                left_status = 0;
                right_status = 0;
                left_time = timer;
                right_time = timer;
                
                leftspeed = travelspeed;
                rightspeed = travelspeed;
            }

        }
        */
        /*
        if (timer > (left_time > right_time ? left_time : right_time) + 1000)
        {
            if (dig.l3 == 1)
            {
                left_status = 1;
                left_time = timer;
            }
            if (dig.r3 == 1)
            {
                right_status = 1;
                right_time = timer;
            }
            
            if (dig.l3 == 0)
            {
                if (timer > (left_time + 500))
                {
                left_status = 0;
                left_time = timer;
                }
            }
            if (dig.r3 == 0)
            {
                if (timer > (right_time + 500))
                {
                right_status = 1;
                right_time = timer;
                }
            }
        }
        */
        
        if (dig.l3 && dig.r3)    // All sensors black: found line
        {
            line_found = true;
        }
        
        if (!(dig.l3 && dig.r3))    // All sensors white
        {
            if (line_found)
            {
                line_counter++;
                line_found = false;
            }
        }
        
        
        if (Motor_Status && (timer > TIMELIMIT || line_counter >= 3) )  // If reach target / run out of time: Stop motors
        {
            motor_stop();               // motor stop
            Motor_Status = false;
        }
        
        MotorDirLeft_Write(0);      // set LeftMotor forward mode
        MotorDirRight_Write(0);     // set RightMotor forward mode
        PWM_WriteCompare1(leftspeed); 
        PWM_WriteCompare2(rightspeed);
        //printf("Setting speed to: %d %d\n", leftspeed, rightspeed);
        {
        //motor_forward(speed, 0);
        //motor_forward(100,2000);     // moving forward
        //motor_turn(200,50,2000);     // turn
        //motor_turn(50,200,2000);     // turn
        //motor_backward(100,2000);    // moving backward
        }
        
        CyDelay(10);
        timer += 10;
        forward_counter += 10;
    }                                       // Infinite loop end
}                                           // Main function end

#endif
/***************************************************************************************/

#if 1
// Flawed but working version
#define TIMELIMIT 50000

int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Systick_Start();
    
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    struct sensors_ dig;
    
    bool BatteryLed_Status = false;
    bool Motor_Status = false;
    
    
    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    
    uint timer = 0;                                 // Time measurement
    uint8 leftspeed = 0;                            // Left track speed
    uint8 rightspeed = 0;                           // Right track speed
    const uint8 travelspeed = 100;                  // Maximum travel speed
    const uint8 small_correction = 20;              // Speed adjustment for small corrections
    //const uint8 large_correction = 60;              // Speed adjustment for larger corrections
    uint forward_counter = 0;                       // Continuous time without direction adjustment
    
    uint8 line_counter = 0;                         // Crossing lines pass. Zumo stops after the 3rd line
    bool line_found = false;                        // If zumo have found a crossing line
    
    CyDelay(2000);                                  // Ready, steady...
    
    for(;;)
    {
        // Check battery status
        ADC_Battery_StartConvert();
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for get ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            volts = (float)adcresult / 4095 * 5;
            volts *= 1.5f;
            if ((volts <= 4.0f) && (!BatteryLed_Status))
            {
                BatteryLed_Write(1);
                BatteryLed_Status = true;
                motor_stop();               // Failsafe: motor stoped if voltage gets too low
            }
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        // Check battery status - End
        
        if (!Motor_Status && timer <= TIMELIMIT)    // ...go!
        {
            motor_start();              // motor start
            Motor_Status = true;
            leftspeed = travelspeed;
            rightspeed = travelspeed;
        }

        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        //printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
        //print out 0 or 1 according to results of reflectance period

        
        if (dig.l1 && dig.r1)       // Left - black Right - black Bring back to straight / Increase speed
        {
            if ((forward_counter != 0) && (forward_counter % 500 == 0))
            {
                if (leftspeed < rightspeed && leftspeed < travelspeed - small_correction) leftspeed += small_correction;
                if (rightspeed < leftspeed && rightspeed < travelspeed - small_correction) rightspeed += small_correction;
            }    
            if ((forward_counter != 0) && (forward_counter % 500 == 0))
            {
                if (leftspeed == rightspeed)
                {
                    if (leftspeed < travelspeed - small_correction)
                    {
                        leftspeed += small_correction;
                        rightspeed += small_correction;
                    }
                }
            }
        }
        
        if (dig.l1 && !dig.r1)      // Left - black Right - white Turn leftward
        {
            if (leftspeed > small_correction) leftspeed -= small_correction;
            if (rightspeed < travelspeed - small_correction) rightspeed += small_correction;
            forward_counter = 0;
        }
        
        if (!dig.l1 && dig.r1)      // Left - white Right - black Turn rightward
        {
            if (leftspeed < travelspeed - small_correction) leftspeed += small_correction;
            if (rightspeed > small_correction) rightspeed -= small_correction;
            forward_counter = 0;
        }
        
        if (dig.l2 && !dig.r2)
        {
            leftspeed = 0;
            rightspeed = travelspeed;
            forward_counter = 0;
        }
        
        if (dig.r2 && !dig.l2)
        {
            leftspeed = travelspeed;
            rightspeed = 0;
            forward_counter = 0;
        }
        
        if (dig.l3 && dig.r3)    // All sensors black: found line
        {
            line_found = true;
        }
        
        if (!(dig.l3 && dig.r3))    // All sensors white
        {
            if (line_found)
            {
                line_counter++;
                line_found = false;
            }
        }
        
        if (Motor_Status && (timer > TIMELIMIT || line_counter >= 3) )  // If reach target / run out of time: Stop motors
        {
            motor_stop();               // motor stop
            Motor_Status = false;
        }
        
        MotorDirLeft_Write(0);      // set LeftMotor forward mode
        MotorDirRight_Write(0);     // set RightMotor forward mode
        PWM_WriteCompare1(leftspeed); 
        PWM_WriteCompare2(rightspeed);
        
        CyDelay(10);
        timer += 10;
        forward_counter += 10;
    }                                       // Infinite loop end
}                                           // Main function end
#endif

#if 0
// Simple track run without sensors

void motor_calibration_track(void)
{
    MotorDirLeft_Write(0);
    MotorDirRight_Write(0);
    PWM_WriteCompare1(0);
    PWM_WriteCompare2(0);
    
    CyDelay(2000);
    // Move forward
    motor_forward(100,3400);
    
    // Turn right
    MotorDirLeft_Write(0);
    MotorDirRight_Write(1);
    PWM_WriteCompare1(100);
    PWM_WriteCompare2(100);
    CyDelay(580);
    
    // Move forward
    motor_forward(100,2700);
    
    // Turn right
    MotorDirLeft_Write(0);
    MotorDirRight_Write(1);
    PWM_WriteCompare1(100);
    PWM_WriteCompare2(100);
    CyDelay(580);
    
    // Move forward
    motor_forward(100,3100);
    
    // Turn right
    MotorDirLeft_Write(0);
    MotorDirRight_Write(1);
    PWM_WriteCompare1(100); 
    PWM_WriteCompare2(100); 
    CyDelay(660);

    // Full stop
    MotorDirLeft_Write(0);
    MotorDirRight_Write(0);
    PWM_WriteCompare1(0);
    PWM_WriteCompare2(0);
    CyDelay(2000);
    
    // Turn curve right

    motor_turn(100,70,1000);
    motor_turn(100,40,2700);
    
    // Move forward
    motor_forward(100,1000);
    
    // Full stop
    MotorDirLeft_Write(0);
    MotorDirRight_Write(0);
    PWM_WriteCompare1(0);
    PWM_WriteCompare2(0);
    CyDelay(2000);
}

int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Systick_Start();
    
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    //struct sensors_ dig;
    
    bool BatteryLed_Status = false;
    //bool Motor_Status = false;
    
    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    
    motor_start();
    motor_calibration_track();
    motor_stop();
    
    for(;;)
    {
        // Check battery status
        ADC_Battery_StartConvert();
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for get ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            volts = (float)adcresult / 4095 * 5;
            volts *= 1.5f;
            if ((volts <= 4.0f) && (!BatteryLed_Status))
            {
                BatteryLed_Write(1);
                BatteryLed_Status = true;
                motor_stop();               // Failsafe: motor stoped if voltage gets too low
            }
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        // Check battery status - End
        

        
            
        //motor_forward(100,2000);     // moving forward
        //motor_turn(200,50,2000);     // turn
        //motor_turn(50,200,2000);     // turn
        //motor_backward(100,2000);    // moving backward
        
        CyDelay(100);
    }                                       // Infinite loop end
}                                           // Main function end


/***************************************************************************************/
#endif


#if 0
//battery level//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Systick_Start();
    
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed

    for(;;)
    {
        
        ADC_Battery_StartConvert();
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for get ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        CyDelay(500);
        
    }
 }   
#endif


#if 0
// button
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Systick_Start();
    
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    bool led = false;
    
    for(;;)
    {
        // toggle led state when button is pressed
        if(SW1_Read() == 0) {
            led = !led;
            BatteryLed_Write(led);
            ShieldLed_Write(led);
            if(led) printf("Led is ON\n");
            else printf("Led is OFF\n");
            Beep(1000, 150);
            while(SW1_Read() == 0) CyDelay(10); // wait while button is being pressed
        }        
    }
 }   
#endif


#if 0
//ultrasonic sensor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Systick_Start();
    Ultra_Start();                          // Ultra Sonic Start function
    while(1) {
        int d = Ultra_GetDistance();
        //If you want to print out the value  
        printf("distance = %d\r\n", d);
        CyDelay(200);
    }
}   
#endif


#if 0
//IR receiver//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    IR_Start();
    
    uint32_t IR_val; 
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    IR_wait(); // wait for IR command
    printf("IR command received\n");
    
    // print received IR pulses and their lengths
    for(;;)
    {
        if(IR_get(&IR_val)) {
            int l = IR_val & IR_SIGNAL_MASK; // get pulse length
            int b = 0;
            if((IR_val & IR_SIGNAL_HIGH) != 0) b = 1; // get pulse state (0/1)
            printf("%d %d\r\n",b, l);
            //printf("%d %lu\r\n",IR_val & IR_SIGNAL_HIGH ? 1 : 0, (unsigned long) (IR_val & IR_SIGNAL_MASK));
        }
    }    
 }   
#endif


#if 0
//reflectance//
int main()
{
    struct sensors_ ref;
    struct sensors_ dig;

    Systick_Start();

    CyGlobalIntEnable; 
    UART_1_Start();
  
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    

    for(;;)
    {
        // read raw sensor values
        reflectance_read(&ref);
        printf("%5d %5d %5d %5d %5d %5d\r\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       // print out each period of reflectance sensors
        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);        //print out 0 or 1 according to results of reflectance period
        
        CyDelay(200);
    }
}   
#endif


#if 0
//motor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();

    motor_start();              // motor start

    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // movinb backward
       
    motor_stop();               // motor stop
    
    for(;;)
    {

    }
}
#endif


/* [] END OF FILE */
