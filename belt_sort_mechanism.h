#ifndef BELT_SORT_MECHANISM_H
#define BELT_SORT_MECHANISM_H

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/time.h>  // For struct itimerval and setitimer
#include <unistd.h>
#include <softPwm.h>
#include <pthread.h>

/* --- Type Definitions --- */
typedef unsigned char uchar;

/* --- Macro Definitions --- */
#define TRIG_PIN 4                 // GPIO pin for ultrasonic trigger
#define ECHO_PIN 5                 // GPIO pin for ultrasonic echo
#define IN1 0                      // GPIO pins for stepper motor control
#define IN2 1
#define IN3 2
#define IN4 3

#define DISTANCE_THRESHOLD 20      // Distance threshold in centimeters

// ADC and Servo definitions
#define ADC_CS    27 
#define ADC_DIO   28  
#define ADC_CLK   29  
#define ServoPin  23  

#define REFERENCE_VOLTAGE 5.0 
#define CLASS_1_THRESHOLD 3.5 
#define CLASS_2_THRESHOLD 4.9 
#define CLASS_3_THRESHOLD 2.9
#define DEBOUNCE_DELAY 1000 

#define NEUTRAL_POSITION 90
#define BIN_1_POSITION 0
#define BIN_2_POSITION 180

#define LOW_SPEED_DELAY 50  
#define HIGH_SPEED_DELAY 20 

#define NO_VALID_READING_FLAG -999 // Flag for ADC reading error

/* --- Global Variables (declared as extern) --- */
extern pthread_mutex_t sharedMutex1;
extern pthread_mutex_t sharedMutex2;
extern pthread_mutex_t sharedMutex3;
extern pthread_mutex_t sharedMutex4;
extern pthread_mutex_t sharedMutex5;

extern volatile float velocityReading;
extern volatile bool stopMotor;
extern volatile bool transition;
extern volatile int beltSpeed;
extern volatile int delayTime;
extern volatile bool shouldStopTimer;

/* --- Enum Definitions --- */
typedef enum {
    READ_SENSOR,
    CALCULATE_VELOCITY,
    RUN_MOTOR,
    TRACK,
    FINISHED
} State_1;    // Standby mode and stepper motor running modes

typedef enum {
    MEASURE,
    SORT_CLASS_1,
    SORT_CLASS_2
} State_2;      // Sorting modes

typedef enum {
    READ_SENSOR2,
    CALCULATE_VELOCITY2,
    FINISHED_2
} State_3; // Tracking modes

enum SystemState { STANDBY, RUNNING, ALLOW_SORTING };
extern enum SystemState sharedSystemState;
extern enum SystemState TrackState;

extern volatile State_2 currentState2;
extern volatile State_1 currentState1;
extern volatile State_3 currentState3;

/* --- Function Prototypes --- */
void initUltrasonicSensor(void);
void initStepperMotor(void);
float measureDistance(void);
void setStep(int a, int b, int c, int d);
void stopStepperMotor(void);
void alarmWakeup(int sig_num);
void alarmWakeup2(int sig_num);
float getAverageReading(void);
void setAngleGradually(int pin, int startAngle, int endAngle, int speedDelay);
void sortProduct(int productType);
uchar get_ADC_Result(void);
int Map(int angle, int fromLow, int fromHigh, int toLow, int toHigh);

void* ultrasonicAndMotorThread(void* arg);
void* voltageAndSortingThread(void* arg);
void* trackVelocityApproach(void* arg);

#endif // BELT_SORT_MECHANISM_H
