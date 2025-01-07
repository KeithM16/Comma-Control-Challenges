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

typedef unsigned char uchar;

#define TRIG_PIN 4                 // GPIO pin for ultrasonic trigger
#define ECHO_PIN 5                 // GPIO pin for ultrasonic echo
#define IN1 0                      // GPIO pins for stepper motor control
#define IN2 1
#define IN3 2
#define IN4 3

#define DISTANCE_THRESHOLD 20      // Distance threshold in centimeters

// Define ADC pins
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

#define NO_VALID_READING_FLAG -999 // Flag to signal that no valid reading was detected from ADC of Photoresistor due to data error


// Mutex declarations
pthread_mutex_t sharedMutex1;
pthread_mutex_t sharedMutex2;
pthread_mutex_t sharedMutex3;
pthread_mutex_t sharedMutex4;
pthread_mutex_t sharedMutex5;


volatile float velocityReading = 0.0;
volatile bool stopMotor = false; // Flag to stop the motor
volatile bool transition = false; // Flag to transition to tracking state while maintaining sorting and belt drive movement
volatile int beltSpeed = 0;
volatile int delayTime;
volatile bool shouldStopTimer = false; //Flag to cancel Timer that triggers switching off of the system when an object is detected within the time frame

// Define the states
typedef enum {
    READ_SENSOR,
    CALCULATE_VELOCITY,
    RUN_MOTOR,
    TRACK,
    FINISHED
} State_1;    //Standby mode and stepper motor running modes



typedef enum {
    MEASURE,
    SORT_CLASS_1,
    SORT_CLASS_2
} State_2;      //Sorting modes

typedef enum {
    READ_SENSOR2,
    CALCULATE_VELOCITY2,
    FINISHED_2
} State_3; //Tracking modes

enum SystemState { STANDBY, RUNNING, ALLOW_SORTING } sharedSystemState, TrackState;

volatile State_2 currentState2 = MEASURE;
volatile State_1 currentState1 = READ_SENSOR; // Start with the initial state
volatile State_3 currentState3 = READ_SENSOR2;



// Function prototypes
void initUltrasonicSensor();
void initStepperMotor();
float measureDistance();
void setStep(int a, int b, int c, int d);
void stopStepperMotor();
void alarmWakeup(int sig_num);
void alarmWakeup2(int sig_num);
float getAverageReading();
void setAngleGradually(int pin, int startAngle, int endAngle, int speedDelay);
void sortProduct(int productType);
uchar get_ADC_Result(void);

// Map the angle to PWM width
int Map(int angle, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (toHigh - toLow) * (angle - fromLow) / (fromHigh - fromLow) + toLow;
}

// Function to get ADC result for photoresistor sensor readings
uchar get_ADC_Result(void)
{
	//10:CH0
	//11:CH1
	uchar i;
	uchar dat1=0, dat2=0;

    // Make Chip Select Low
	digitalWrite(ADC_CS, 0);

    // From Datasheet: Max Clock = 400 KHz, converts to ~2.5microsecs
    // Clock in software is ~4microsec
    // (2 microsec LOW followed by 2 microsec HIGH)
    
    // Start Bit: HIGH
	digitalWrite(ADC_CLK,0);
	digitalWrite(ADC_DIO,1);	delayMicroseconds(2);
	digitalWrite(ADC_CLK,1);	delayMicroseconds(2);

    // SGL / BarDIF: 1
    digitalWrite(ADC_CLK,0);
	digitalWrite(ADC_DIO,1);    delayMicroseconds(2);
    digitalWrite(ADC_CLK,1);	delayMicroseconds(2);
	
    // ODD / SIGN: 0
    digitalWrite(ADC_CLK,0);
	digitalWrite(ADC_DIO,0);	delayMicroseconds(2);
    digitalWrite(ADC_CLK,1);	delayMicroseconds(2);
    
    // Add half a clock for settling time
    digitalWrite(ADC_CLK,0);    delayMicroseconds(2);

    // Get MSB first
	for(i=0;i<8;i++)
	{
		digitalWrite(ADC_CLK,1);	delayMicroseconds(2);
		digitalWrite(ADC_CLK,0);    delayMicroseconds(2);

		pinMode(ADC_DIO, INPUT);
		dat1=dat1<<1 | digitalRead(ADC_DIO);
	}

    // Get LSB first (same data as previous 8 bits)
	for(i=0;i<8;i++)
	{
		dat2 = dat2 | ((uchar)(digitalRead(ADC_DIO))<<i);
		digitalWrite(ADC_CLK,1); 	delayMicroseconds(2);
		digitalWrite(ADC_CLK,0);    delayMicroseconds(2);
	}

    // Make Chip Select High
	digitalWrite(ADC_CS,1);

    // Make DIO Pin output mode
	pinMode(ADC_DIO, OUTPUT);

    // Return dat1 only when dat1 and dat2 are same
	return(dat1==dat2) ? dat1 : 0;
}

// Gradually set servo angle during sorting movement
void setAngleGradually(int pin, int startAngle, int endAngle, int speedDelay) {
    int step = (startAngle < endAngle) ? 1 : -1;
    for (int angle = startAngle; angle != endAngle; angle += step) {
        int pwmVal = Map(angle, 0, 180, 5, 25);
        softPwmWrite(pin, pwmVal);
        delay(speedDelay);
    }
    int finalPwmVal = Map(endAngle, 0, 180, 5, 25);
    softPwmWrite(pin, finalPwmVal);
}

// Sort product based on type
void sortProduct(int productType) {
    int speedDelay = (beltSpeed == 1) ? LOW_SPEED_DELAY : HIGH_SPEED_DELAY;
    if (productType == 1) {
        setAngleGradually(ServoPin, NEUTRAL_POSITION, BIN_1_POSITION, speedDelay);
        printf("Sorted to Bin 1 (Class 1).\n");
    } else if (productType == 2) {
        setAngleGradually(ServoPin, NEUTRAL_POSITION, BIN_2_POSITION, speedDelay);
        printf("Sorted to Bin 2 (Class 2).\n");
    }
    setAngleGradually(ServoPin, (productType == 1) ? BIN_1_POSITION : BIN_2_POSITION, NEUTRAL_POSITION, speedDelay);
}

// Get average ADC reading
float getAverageReading() {
    float reading1, reading2, average;
    static unsigned long lastTriggerTime = 0; // Initialize to 0 for debounce logic
    static int isFirstDetection = 1;           // Flag to handle the first detection
    unsigned long currentTime = millis();      // Get the current time

    // Take the first reading
    uchar adcValue1 = get_ADC_Result();
    if (adcValue1 == 0) {
        printf("ADC Read Error: Invalid first reading\n");
        return NO_VALID_READING_FLAG;  // Return error flag
    }
    reading1 = (adcValue1 / 255.0) * REFERENCE_VOLTAGE;
    printf("First Reading Voltage: %.2f V\n", reading1);

    // Check if the first detection or debounce logic is satisfied
    if (reading1 >= CLASS_1_THRESHOLD && reading1 < CLASS_2_THRESHOLD && (isFirstDetection || (currentTime - lastTriggerTime) >= DEBOUNCE_DELAY)) {
        lastTriggerTime = currentTime;  // Update last trigger time
        isFirstDetection = 0;           // Clear the first detection flag after handling the first object

        // Take a second reading
        uchar adcValue2 = get_ADC_Result();
        if (adcValue2 == 0) {
        //printf("ADC Read Error: Invalid second reading\n");
        return NO_VALID_READING_FLAG;  // Return error flag
        }
        
        reading2 = (adcValue2 / 255.0) * REFERENCE_VOLTAGE;
        //printf("Second Reading Voltage: %.2f V\n", reading2);

        // Calculate average
        average = (reading1 + reading2) / 2.0;
        printf("Average Voltage: %.2f V\n", average);
        return average;

    } else if (reading1 < CLASS_1_THRESHOLD && (isFirstDetection || (currentTime - lastTriggerTime) >= DEBOUNCE_DELAY)) {
        lastTriggerTime = currentTime;  // Update last trigger time for debounce calculation
        isFirstDetection = 0;           // Clear the first detection flag after handling the first object

        // Take a second reading
        uchar adcValue2 = get_ADC_Result();
        if (adcValue2 == 0) {
        // printf("ADC Read Error: Invalid second reading\n");
        return NO_VALID_READING_FLAG;  // Return error flag
        }
        reading2 = (adcValue2 / 255.0) * REFERENCE_VOLTAGE;
        // printf("Second Reading Voltage: %.2f V\n", reading2);

        // Calculate average for low readings
        average = (reading1 + reading2) / 2.0;
        printf("Average Voltage: %.2f V\n", average);
        return average;
    } else {
        printf("No valid reading detected.\n");
        return NO_VALID_READING_FLAG;
    }
}

// Function to initialize the ultrasonic sensor pins
void initUltrasonicSensor() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW); // Ensure trigger pin is low at start
    delay(30);                   // Short delay for sensor stabilization
    printf("Ultrasonic sensor initialized.\n");
}

// Function to initialize the GPIO pins for stepper motor control
void initStepperMotor() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    printf("Stepper motor initialized.\n");
}

// Function to measure distance using the ultrasonic sensor
float measureDistance() {
    long startTime, travelTime;
    float distance;
    printf("Start measurement.\n");

    // Send a 10-microsecond pulse to trigger the ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for the echo to start (signal goes HIGH)
    while (digitalRead(ECHO_PIN) == LOW);

    // Measure the length of the pulse (time for signal to go HIGH)
    startTime = micros();
    while (digitalRead(ECHO_PIN) == HIGH);
    travelTime = micros() - startTime;

    // Calculate the distance in centimeters
    distance = (travelTime / 2.0) * 0.0343;
    printf("Distance measured: %.2f cm\n", distance);

    return distance;
}



// Function to set the motor steps
void setStep(int a, int b, int c, int d) {
    digitalWrite(IN1, a);
    digitalWrite(IN2, b);
    digitalWrite(IN3, c);
    digitalWrite(IN4, d);
}

// Function to stop the motor
void stopStepperMotor() {
    setStep(0, 0, 0, 0);  // Stop the motor by setting all step pins low
}

 //Function to handle the SIGALRM interrup
void alarmWakeup(int signum) {
    printf("ITIMER_REAL: Alarm 1 triggered!\n");
    pthread_mutex_lock(&sharedMutex4);
    transition = true;  // Set the flag to stop the motor after the threshold
    pthread_mutex_unlock(&sharedMutex4);
    printf("Transition alarm from System Running State to (System Running and Tracking New Object Velocity concurrently) triggered.\n");
}
    
void alarmWakeup2(int signum) {
        printf("ITIMER_VIRTUAL: Alarm 2 triggered!\n");
        pthread_mutex_lock(&sharedMutex4);
        stopMotor = true;  // Set the flag to stop the motor after the threshold
        printf("Idling system alarm triggered, sets off flag that turns off the system if it idles for 10 seconds in Tracking State with no new object detection within Ultrasound Sensor threshold.\n");
        pthread_mutex_unlock(&sharedMutex4);
    }






void* ultrasonicAndMotorThread(void* arg) {
    while (true) {
        

        switch (currentState1) {
            case READ_SENSOR:

                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);

                printf("State: READ_ULTRASOUND SENSOR \n");
                int distance = measureDistance();
                if (distance <= 20) { // Example threshold
                    printf("Object detected within threshold distance of drive. Distance: %d cm.\n Transitioning to CALCULATE_VELOCITY.\n", distance);
                    currentState1 = CALCULATE_VELOCITY;
                } else {
                    printf("No object detected within threshold. Distance: %d cm.\n", distance);
                    sleep(3);
                }
                break;

            case CALCULATE_VELOCITY:

                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);

                printf("State: CALCULATE_VELOCITY\n");
                static float previousDistance = 0;  // Store previous distance reading
                float currentDistance;
                long startTime, endTime;
                
                // Take the first distance measurement and record corresponding time
                previousDistance = measureDistance();
                startTime = millis();
                printf("First distance measurement to determine velocity of object\n");

                // If the initial distance is not below the threshold, return 0 (no object detected)
                if (previousDistance >= DISTANCE_THRESHOLD) {
                    printf("No object detected within threshold distance.\n");
                    currentState1 = READ_SENSOR;
                    break;
                
                }
                else {

                // Wait a short delay before taking the second measurement
                    delay(200); // 200 ms delay for potential movement, to be altered for analysis

                    // Take the second distance measurement and record the time
                    currentDistance = measureDistance();
                    endTime = millis();
                    printf("Second distance measurement in determining object velocity\n");

                    // Calculate the velocity if the object is still moving closer
                    if (currentDistance < previousDistance) {
                        printf("Object is approaching, calculating velocity...\n");

                        long timeElapsed = endTime - startTime;  // Calculate time elapsed in ms
                        float distanceChange = previousDistance - currentDistance;
        
                        // Calculate velocity in cm/s
                        velocityReading = (distanceChange / timeElapsed) * 1000;
                        printf("Velocity: %.2f cm/s\n", velocityReading);
                        currentState1 = RUN_MOTOR; 

                    }
                    else{
                        printf("Object moving away from system, error, back to MEASURE State");
                        currentState1 = READ_SENSOR;
                    }
                }
                break;

            case RUN_MOTOR:
                printf("State: RUN_STEPPER_MOTOR Belt Drive\n");

                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = ALLOW_SORTING;
                pthread_mutex_unlock(&sharedMutex1);
                
                // Set up the timer interrupt to trigger after 10 seconds (10000 ms)
                // Define velocity threshold and set delay based on velocity
                const float VELOCITY_THRESHOLD = 10.0;  // Example threshold velocity in cm/s
                

                if (velocityReading <= VELOCITY_THRESHOLD) {
                    delayTime = 10; // Longer delay for low velocities
                    pthread_mutex_unlock(&sharedMutex3);
                    beltSpeed = 1; // belt drive set, determines speed of sorting mechanism in sync with stepper motor drive motion
                    pthread_mutex_unlock(&sharedMutex3);
                } else {
                      delayTime = 5;   // Shorter delay for high velocities
                      pthread_mutex_lock(&sharedMutex3);
                      beltSpeed = 0;
                      pthread_mutex_unlock(&sharedMutex3);
                }

                printf("Stepper Motor (Belt Drive) delay time set to: %d ms\n", delayTime);
                // Configure ITIMER_REAL (Alarm 1)
                signal(SIGALRM, alarmWakeup); 

                struct itimerval timer1;
                        timer1.it_value.tv_sec = 5;        // First alarm in 5 seconds
                        timer1.it_value.tv_usec = 0;
                        timer1.it_interval.tv_sec = 0;    // No repetition
                        timer1.it_interval.tv_usec = 0;

                //Set the timer
                setitimer(ITIMER_REAL, &timer1, NULL);    // Start first timer
               

                // Motor control loop
                while (!transition) {  // Keep running until stopMotor flag is set
                    setStep(1, 0, 0, 0);
                    delay(delayTime);  // Delay for stepper motor timing
                    setStep(0, 1, 0, 0);
                    delay(delayTime);
                    setStep(0, 0, 1, 0);
                    delay(delayTime);
                    setStep(0, 0, 0, 1);
                    delay(delayTime);
                }

                // After the motor stops, call stop function
                //stopStepperMotor();
                pthread_mutex_lock(&sharedMutex4);
                transition = false;  // Reset the stop flag for future iterations
                currentState1 = TRACK;
                pthread_mutex_unlock(&sharedMutex4);
                break;

            case TRACK:
                printf("Track state of stepper motor to determine new velocity.\n");

                pthread_mutex_lock(&sharedMutex5);
                TrackState = RUNNING;
                pthread_mutex_unlock(&sharedMutex5);
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = ALLOW_SORTING;
                pthread_mutex_unlock(&sharedMutex1);

                signal(SIGVTALRM, alarmWakeup2); 

                 struct itimerval timer2;
                    timer2.it_value.tv_sec = 10;       // Timer expires in 10 seconds
                    timer2.it_value.tv_usec = 0;
                    timer2.it_interval.tv_sec = 0;     // One-time timer (no periodic interval)
                    timer2.it_interval.tv_usec = 0;

                    // Set the timer
                    setitimer(ITIMER_VIRTUAL, &timer2, NULL);
    
                // Maintain motor movement
                while (!shouldStopTimer && !stopMotor) {
                    
                    // Perform motor stepping
                    setStep(1, 0, 0, 0);
                    delay(delayTime);
                    setStep(0, 1, 0, 0);
                    delay(delayTime);
                    setStep(0, 0, 1, 0);
                    delay(delayTime);
                    setStep(0, 0, 0, 1);
                    delay(delayTime);
                }

                // Stop motor and reset flags
                if(shouldStopTimer){
                     // Set the timer to zero, effectively stopping it
                    timer2.it_value.tv_sec = 0;
                    timer2.it_value.tv_usec = 0;
                    timer2.it_interval.tv_sec = 0;
                    timer2.it_interval.tv_usec = 0;

                    // Disable the timer by setting it to zero
                    setitimer(ITIMER_VIRTUAL, &timer2, NULL);
                    printf("Virtual Timer stopped!\n");
                    stopStepperMotor();
                    pthread_mutex_lock(&sharedMutex5);
                    TrackState = STANDBY;
                    shouldStopTimer = false;
                    currentState1 = RUN_MOTOR;
                    pthread_mutex_unlock(&sharedMutex5);
                }

                else if (stopMotor) {
                    pthread_mutex_lock(&sharedMutex4);
                    stopMotor = false;
                    pthread_mutex_unlock(&sharedMutex4);
                    pthread_mutex_lock(&sharedMutex1);
                    sharedSystemState = STANDBY;
                    TrackState = STANDBY;
                    currentState1 = READ_SENSOR;
                    pthread_mutex_unlock(&sharedMutex1);
                    
                }
                break;
            case FINISHED:
                
                pthread_mutex_lock(&sharedMutex1);
                   sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);
                pthread_mutex_lock(&sharedMutex5);
                    TrackState = STANDBY; 
                pthread_mutex_unlock(&sharedMutex5);

                printf("State: FINISHED\n");
                printf("Process complete. Restarting...\n");
                currentState1 = READ_SENSOR; // Optional restart
                break;

            default:
                
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);
                pthread_mutex_lock(&sharedMutex5);
                    TrackState = STANDBY; 
                pthread_mutex_unlock(&sharedMutex5);

                printf("Unknown state! Resetting...\n");
                currentState1 = READ_SENSOR; // Fallback to initial state
                break;
        }
        
        usleep(100000);  // Sleep for 100 ms to avoid busy looping
    }
    return NULL;
}

void* voltageAndSortingThread(void* arg) {
    while (true) {
        
         // Check if sorting is allowed
        if (sharedSystemState == ALLOW_SORTING) {
         switch (currentState2) {
            case MEASURE: {

                pthread_mutex_lock(&sharedMutex2);
                float averageVoltage = getAverageReading();
                pthread_mutex_unlock(&sharedMutex2);

                if (averageVoltage == NO_VALID_READING_FLAG) {
                    // Debounce not satisfied; keep polling
                    printf("Error in Sorting Gate Sensor Reading i.e ADC Read Error.\n");
                    break;
                }

                // Debounce satisfied; check thresholds and transition to SORT
                if (averageVoltage < CLASS_1_THRESHOLD && averageVoltage > CLASS_3_THRESHOLD) {
                    currentState2 = SORT_CLASS_1;
                    break;
                } else if (averageVoltage >= CLASS_1_THRESHOLD && averageVoltage < CLASS_2_THRESHOLD) {
                    currentState2 = SORT_CLASS_2;
                    break;
                } else {
                    printf("No product detected at sorting gate.\n");
                    sleep(3);
                    break;
                }
            }
            case SORT_CLASS_1: {
                sortProduct(1);
                currentState2 = MEASURE;
                break;
            }
            case SORT_CLASS_2: {
                sortProduct(2);
                currentState2 = MEASURE;
                break;
            }
            default: {
                printf("Unknown state!\n");
                currentState2 = MEASURE;
                break;
            }
        }
        }
        else {
            // If sorting is not allowed, sleep for a while before rechecking
            printf("Thread 2: System in STANDBY, No sorting.\n");
        }


        
        sleep(3);  // Sleep for 3s to avoid busy looping, to be adjusted to analyse effect of change of sleep time.
    }
    return NULL;
}

void* trackVelocityApproach(void* arg) {
    while (true) {
        if (TrackState == RUNNING) {
            switch (currentState3) {
            case READ_SENSOR2:
                printf("Mode: Tracking Objects Within Ultrasound Sensor Threshold to trigger Possible Velocity Adjustment.\n");
                int distance2 = measureDistance();
                if (distance2 <= 20) { // Example threshold
                    printf("Object detected. Distance: %d cm..\n", distance2);
                    currentState3 = CALCULATE_VELOCITY2;
                } else {
                    printf("No object detected. Distance: %d cm.\n", distance2);
                    sleep(1);
                }
                break;

            case CALCULATE_VELOCITY2: {
                printf("Mode: Tracking possible velocity adjustment\n");

                static float previousDistance1 = 0;  // Store previous distance reading
                float currentDistance1;
                long startTime1, endTime1;

                printf("Calculating New Velocity\n");

                // Take the first distance measurement and record the time
                pthread_mutex_lock(&sharedMutex5);
                previousDistance1 = measureDistance();
                pthread_mutex_unlock(&sharedMutex5);
                printf("First distance measurement in tracking velocity change, Distance: %.2f cm/s\n", previousDistance1);

                startTime1 = millis();

                if (previousDistance1 >= DISTANCE_THRESHOLD) {
                    printf("No object detected within threshold.\n");
                    currentState3 = READ_SENSOR2;
                    break;
                }

                delay(200); // 200 ms delay for potential movement, to be adjusted for analysis

                pthread_mutex_lock(&sharedMutex5);
                currentDistance1 = measureDistance();
                endTime1 = millis();
                pthread_mutex_unlock(&sharedMutex5);

                printf("Second distance measurement in tracking velocity change, Distance: %.2f cm/s\n", currentDistance1);

                if (currentDistance1 < previousDistance1) {
                    printf("Object is approaching, calculating velocity...\n");

                    long timeElapsed1 = endTime1 - startTime1;  // Time elapsed in ms
                    float distanceChange1 = previousDistance1 - currentDistance1;

                    // Calculate velocity in cm/s
                    pthread_mutex_lock(&sharedMutex5);
                    velocityReading = (distanceChange1 / timeElapsed1) * 1000;
                    pthread_mutex_unlock(&sharedMutex5);

                    printf("New Velocity: %.2f cm/s\n", velocityReading);

                    pthread_mutex_lock(&sharedMutex5);
                    shouldStopTimer = true;  // Stop the timer
                    currentState3 = FINISHED_2;
                    pthread_mutex_unlock(&sharedMutex5);
                } else {
                    printf("Object moving away from system, error.\n Back to determine object presence within threshold distance\n");
                    currentState3 = READ_SENSOR2;
                }
                break;
            }

            case FINISHED_2:
                pthread_mutex_lock(&sharedMutex5);
                TrackState = STANDBY;
                pthread_mutex_unlock(&sharedMutex5);

                printf("Mode: Finished Tracking New System Velocity\n");
                printf("Speed Adjustment Initiated ...\n");
                currentState3 = READ_SENSOR2; // Restart
                break;

            default:
                printf("Unknown State!\n");
                currentState3 = READ_SENSOR2;
                break;
            }
        } else {
            printf("Thread 3: Velocity Tracking System is in STANDBY.\n");
            sleep(3); //To be adjusted to test effect of longer downtime for system operation
        }
    }
}





// Main program
int main(void) {
    pthread_t thread1, thread2, thread3;

    //struct sigevent sev1, sev2;
    //timer_t timer1, timer2;
    //struct itimerspec its1, its2;
     signal(SIGALRM, alarmWakeup);
    signal(SIGVTALRM, alarmWakeup2);

    //struct itimerval timer1, timer2;

    

    if (pthread_mutex_init(&sharedMutex1, NULL) != 0) {
        printf("Error: Failed to initialize sharedMutex1\n");
        return EXIT_FAILURE;
    }
    if (pthread_mutex_init(&sharedMutex2, NULL) != 0) {
        printf("Error: Failed to initialize sharedMutex2\n");
        return EXIT_FAILURE;
    }
    if (pthread_mutex_init(&sharedMutex3, NULL) != 0) {
        printf("Error: Failed to initialize sharedMutex3\n");
        return EXIT_FAILURE;
    }
    if (pthread_mutex_init(&sharedMutex4, NULL) != 0) {
        printf("Error: Failed to initialize sharedMutex4\n");
        return EXIT_FAILURE;
    }
    if (pthread_mutex_init(&sharedMutex5, NULL) != 0) {
        printf("Error: Failed to initialize sharedMutex5\n");
        return EXIT_FAILURE;
    }

    // Initialize WiringPi and hardware setup
    if (wiringPiSetup() == -1) {
        printf("Setup failed!\n");
        return 1;
    }
    initUltrasonicSensor();
    initStepperMotor();
    pinMode(ADC_CS, OUTPUT);
    pinMode(ADC_CLK, OUTPUT);
    softPwmCreate(ServoPin, 0, 100);

    // Create threads
    pthread_create(&thread1, NULL, ultrasonicAndMotorThread, NULL);
    pthread_create(&thread2, NULL, voltageAndSortingThread, NULL);
    pthread_create(&thread3, NULL, trackVelocityApproach, NULL);

    // Wait for threads to finish (optional, here it's an infinite loop)
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);

    // Clean up
    // Destroy mutexes
    pthread_mutex_destroy(&sharedMutex1);
    pthread_mutex_destroy(&sharedMutex2);
    pthread_mutex_destroy(&sharedMutex3);
    pthread_mutex_destroy(&sharedMutex4);
    pthread_mutex_destroy(&sharedMutex5);

    return 0;
}



