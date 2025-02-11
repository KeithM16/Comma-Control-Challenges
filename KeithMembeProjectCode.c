#include "belt_sort_mechanism.h"

/* --- Global Variable Definitions --- */
pthread_mutex_t sharedMutex1;
pthread_mutex_t sharedMutex2;
pthread_mutex_t sharedMutex3;
pthread_mutex_t sharedMutex4;
pthread_mutex_t sharedMutex5;

volatile float velocityReading = 0.0;
volatile bool stopMotor = false;
volatile bool transition = false;
volatile int beltSpeed = 0;
volatile int delayTime;
volatile bool shouldStopTimer = false;

enum SystemState sharedSystemState = STANDBY, TrackState = STANDBY;
volatile State_2 currentState2 = MEASURE;
volatile State_1 currentState1 = READ_SENSOR;
volatile State_3 currentState3 = READ_SENSOR2;

/* --- Function Definitions --- */

// Map the angle to PWM width
int Map(int angle, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (toHigh - toLow) * (angle - fromLow) / (fromHigh - fromLow) + toLow;
}

// Function to get ADC result for photoresistor sensor readings
uchar get_ADC_Result(void)
{
	uchar i;
	uchar dat1 = 0, dat2 = 0;

	// Make Chip Select Low
	digitalWrite(ADC_CS, 0);

	// Start Bit: HIGH
	digitalWrite(ADC_CLK, 0);
	digitalWrite(ADC_DIO, 1);	delayMicroseconds(2);
	digitalWrite(ADC_CLK, 1);	delayMicroseconds(2);

	// SGL / BarDIF: 1
	digitalWrite(ADC_CLK, 0);
	digitalWrite(ADC_DIO, 1);    delayMicroseconds(2);
	digitalWrite(ADC_CLK, 1);	delayMicroseconds(2);
	
	// ODD / SIGN: 0
	digitalWrite(ADC_CLK, 0);
	digitalWrite(ADC_DIO, 0);	delayMicroseconds(2);
	digitalWrite(ADC_CLK, 1);	delayMicroseconds(2);
    
	// Settling time
	digitalWrite(ADC_CLK, 0);    delayMicroseconds(2);

	// Get MSB first
	for(i = 0; i < 8; i++)
	{
		digitalWrite(ADC_CLK, 1);	delayMicroseconds(2);
		digitalWrite(ADC_CLK, 0);    delayMicroseconds(2);

		pinMode(ADC_DIO, INPUT);
		dat1 = (dat1 << 1) | digitalRead(ADC_DIO);
	}

	// Get LSB first
	for(i = 0; i < 8; i++)
	{
		dat2 = dat2 | ((uchar)(digitalRead(ADC_DIO)) << i);
		digitalWrite(ADC_CLK, 1); 	delayMicroseconds(2);
		digitalWrite(ADC_CLK, 0);    delayMicroseconds(2);
	}

	// Make Chip Select High and reset DIO mode
	digitalWrite(ADC_CS, 1);
	pinMode(ADC_DIO, OUTPUT);

	// Return dat1 only when dat1 and dat2 are the same
	return (dat1 == dat2) ? dat1 : 0;
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
    static unsigned long lastTriggerTime = 0;
    static int isFirstDetection = 1;
    unsigned long currentTime = millis();

    // First reading
    uchar adcValue1 = get_ADC_Result();
    if (adcValue1 == 0) {
        printf("ADC Read Error: Invalid first reading\n");
        return NO_VALID_READING_FLAG;
    }
    reading1 = (adcValue1 / 255.0) * REFERENCE_VOLTAGE;
    printf("First Reading Voltage: %.2f V\n", reading1);

    if (reading1 >= CLASS_1_THRESHOLD && reading1 < CLASS_2_THRESHOLD &&
       (isFirstDetection || (currentTime - lastTriggerTime) >= DEBOUNCE_DELAY)) {
        lastTriggerTime = currentTime;
        isFirstDetection = 0;
        uchar adcValue2 = get_ADC_Result();
        if (adcValue2 == 0) {
            return NO_VALID_READING_FLAG;
        }
        reading2 = (adcValue2 / 255.0) * REFERENCE_VOLTAGE;
        average = (reading1 + reading2) / 2.0;
        printf("Average Voltage: %.2f V\n", average);
        return average;
    } else if (reading1 < CLASS_1_THRESHOLD &&
              (isFirstDetection || (currentTime - lastTriggerTime) >= DEBOUNCE_DELAY)) {
        lastTriggerTime = currentTime;
        isFirstDetection = 0;
        uchar adcValue2 = get_ADC_Result();
        if (adcValue2 == 0) {
            return NO_VALID_READING_FLAG;
        }
        reading2 = (adcValue2 / 255.0) * REFERENCE_VOLTAGE;
        average = (reading1 + reading2) / 2.0;
        printf("Average Voltage: %.2f V\n", average);
        return average;
    } else {
        printf("No valid reading detected.\n");
        return NO_VALID_READING_FLAG;
    }
}

// Initialize the ultrasonic sensor pins
void initUltrasonicSensor() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delay(30);
    printf("Ultrasonic sensor initialized.\n");
}

// Initialize GPIO pins for stepper motor control
void initStepperMotor() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    printf("Stepper motor initialized.\n");
}

// Measure distance using the ultrasonic sensor
float measureDistance() {
    long startTime, travelTime;
    float distance;
    printf("Start measurement.\n");

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    while (digitalRead(ECHO_PIN) == LOW);
    startTime = micros();
    while (digitalRead(ECHO_PIN) == HIGH);
    travelTime = micros() - startTime;

    distance = (travelTime / 2.0) * 0.0343;
    printf("Distance measured: %.2f cm\n", distance);
    return distance;
}

// Set the motor steps
void setStep(int a, int b, int c, int d) {
    digitalWrite(IN1, a);
    digitalWrite(IN2, b);
    digitalWrite(IN3, c);
    digitalWrite(IN4, d);
}

// Stop the motor
void stopStepperMotor() {
    setStep(0, 0, 0, 0);
}

// Signal handler for SIGALRM
void alarmWakeup(int signum) {
    printf("ITIMER_REAL: Alarm 1 triggered!\n");
    pthread_mutex_lock(&sharedMutex4);
    transition = true;
    pthread_mutex_unlock(&sharedMutex4);
    printf("Transition alarm triggered.\n");
}

// Signal handler for SIGVTALRM
void alarmWakeup2(int signum) {
    printf("ITIMER_VIRTUAL: Alarm 2 triggered!\n");
    pthread_mutex_lock(&sharedMutex4);
    stopMotor = true;
    printf("Idling system alarm triggered.\n");
    pthread_mutex_unlock(&sharedMutex4);
}

// Thread: Ultrasonic sensor and motor control
void* ultrasonicAndMotorThread(void* arg) {
    while (true) {
        switch (currentState1) {
            case READ_SENSOR:
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);

                printf("State: READ_ULTRASOUND SENSOR\n");
                int distance = measureDistance();
                if (distance <= 20) {
                    printf("Object detected (Distance: %d cm). Transitioning to CALCULATE_VELOCITY.\n", distance);
                    currentState1 = CALCULATE_VELOCITY;
                } else {
                    printf("No object detected (Distance: %d cm).\n", distance);
                    sleep(3);
                }
                break;

            case CALCULATE_VELOCITY:
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);

                printf("State: CALCULATE_VELOCITY\n");
                static float previousDistance = 0;
                float currentDistance;
                long startTime, endTime;
                
                previousDistance = measureDistance();
                startTime = millis();
                printf("First distance measurement taken.\n");

                if (previousDistance >= DISTANCE_THRESHOLD) {
                    printf("No object within threshold.\n");
                    currentState1 = READ_SENSOR;
                    break;
                } else {
                    delay(200);
                    currentDistance = measureDistance();
                    endTime = millis();
                    printf("Second distance measurement taken.\n");

                    if (currentDistance < previousDistance) {
                        printf("Object approaching. Calculating velocity...\n");
                        long timeElapsed = endTime - startTime;
                        float distanceChange = previousDistance - currentDistance;
                        velocityReading = (distanceChange / timeElapsed) * 1000;
                        printf("Velocity: %.2f cm/s\n", velocityReading);
                        currentState1 = RUN_MOTOR; 
                    } else {
                        printf("Object moving away. Resetting to READ_SENSOR.\n");
                        currentState1 = READ_SENSOR;
                    }
                }
                break;

            case RUN_MOTOR:
                printf("State: RUN_STEPPER_MOTOR (Belt Drive)\n");
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = ALLOW_SORTING;
                pthread_mutex_unlock(&sharedMutex1);
                
                const float VELOCITY_THRESHOLD = 10.0;
                if (velocityReading <= VELOCITY_THRESHOLD) {
                    delayTime = 10;
                    pthread_mutex_unlock(&sharedMutex3);
                    beltSpeed = 1;
                    pthread_mutex_unlock(&sharedMutex3);
                } else {
                    delayTime = 5;
                    pthread_mutex_lock(&sharedMutex3);
                    beltSpeed = 0;
                    pthread_mutex_unlock(&sharedMutex3);
                }

                printf("Stepper Motor delay time: %d ms\n", delayTime);
                signal(SIGALRM, alarmWakeup); 

                struct itimerval timer1;
                timer1.it_value.tv_sec = 5;
                timer1.it_value.tv_usec = 0;
                timer1.it_interval.tv_sec = 0;
                timer1.it_interval.tv_usec = 0;
                setitimer(ITIMER_REAL, &timer1, NULL);
               
                while (!transition) {
                    setStep(1, 0, 0, 0);
                    delay(delayTime);
                    setStep(0, 1, 0, 0);
                    delay(delayTime);
                    setStep(0, 0, 1, 0);
                    delay(delayTime);
                    setStep(0, 0, 0, 1);
                    delay(delayTime);
                }

                pthread_mutex_lock(&sharedMutex4);
                transition = false;
                currentState1 = TRACK;
                pthread_mutex_unlock(&sharedMutex4);
                break;

            case TRACK:
                printf("State: TRACK (Adjusting velocity)\n");
                pthread_mutex_lock(&sharedMutex5);
                TrackState = RUNNING;
                pthread_mutex_unlock(&sharedMutex5);
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = ALLOW_SORTING;
                pthread_mutex_unlock(&sharedMutex1);

                signal(SIGVTALRM, alarmWakeup2); 
                struct itimerval timer2;
                timer2.it_value.tv_sec = 10;
                timer2.it_value.tv_usec = 0;
                timer2.it_interval.tv_sec = 0;
                timer2.it_interval.tv_usec = 0;
                setitimer(ITIMER_VIRTUAL, &timer2, NULL);
    
                while (!shouldStopTimer && !stopMotor) {
                    setStep(1, 0, 0, 0);
                    delay(delayTime);
                    setStep(0, 1, 0, 0);
                    delay(delayTime);
                    setStep(0, 0, 1, 0);
                    delay(delayTime);
                    setStep(0, 0, 0, 1);
                    delay(delayTime);
                }

                if (shouldStopTimer) {
                    timer2.it_value.tv_sec = 0;
                    timer2.it_value.tv_usec = 0;
                    timer2.it_interval.tv_sec = 0;
                    timer2.it_interval.tv_usec = 0;
                    setitimer(ITIMER_VIRTUAL, &timer2, NULL);
                    printf("Virtual Timer stopped!\n");
                    stopStepperMotor();
                    pthread_mutex_lock(&sharedMutex5);
                    TrackState = STANDBY;
                    shouldStopTimer = false;
                    currentState1 = RUN_MOTOR;
                    pthread_mutex_unlock(&sharedMutex5);
                } else if (stopMotor) {
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
                currentState1 = READ_SENSOR;
                break;

            default:
                pthread_mutex_lock(&sharedMutex1);
                sharedSystemState = STANDBY;
                pthread_mutex_unlock(&sharedMutex1);
                pthread_mutex_lock(&sharedMutex5);
                TrackState = STANDBY; 
                pthread_mutex_unlock(&sharedMutex5);
                printf("Unknown state! Resetting...\n");
                currentState1 = READ_SENSOR;
                break;
        }
        usleep(100000);
    }
    return NULL;
}

// Thread: Voltage reading and sorting
void* voltageAndSortingThread(void* arg) {
    while (true) {
        if (sharedSystemState == ALLOW_SORTING) {
            switch (currentState2) {
                case MEASURE: {
                    pthread_mutex_lock(&sharedMutex2);
                    float averageVoltage = getAverageReading();
                    pthread_mutex_unlock(&sharedMutex2);

                    if (averageVoltage == NO_VALID_READING_FLAG) {
                        printf("Error in Sorting Gate Sensor Reading (ADC Read Error).\n");
                        break;
                    }

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
        } else {
            printf("Thread 2: System in STANDBY, no sorting.\n");
        }
        sleep(3);
    }
    return NULL;
}

// Thread: Velocity tracking
void* trackVelocityApproach(void* arg) {
    while (true) {
        if (TrackState == RUNNING) {
            switch (currentState3) {
                case READ_SENSOR2:
                    printf("Mode: Tracking objects for possible velocity adjustment.\n");
                    int distance2 = measureDistance();
                    if (distance2 <= 20) {
                        printf("Object detected (Distance: %d cm).\n", distance2);
                        currentState3 = CALCULATE_VELOCITY2;
                    } else {
                        printf("No object detected (Distance: %d cm).\n", distance2);
                        sleep(1);
                    }
                    break;

                case CALCULATE_VELOCITY2: {
                    printf("Mode: Tracking possible velocity adjustment\n");
                    static float previousDistance1 = 0;
                    float currentDistance1;
                    long startTime1, endTime1;
                    printf("Calculating new velocity\n");

                    pthread_mutex_lock(&sharedMutex5);
                    previousDistance1 = measureDistance();
                    pthread_mutex_unlock(&sharedMutex5);
                    printf("First measurement (Distance: %.2f cm)\n", previousDistance1);
                    startTime1 = millis();

                    if (previousDistance1 >= DISTANCE_THRESHOLD) {
                        printf("No object detected within threshold.\n");
                        currentState3 = READ_SENSOR2;
                        break;
                    }

                    delay(200);
                    pthread_mutex_lock(&sharedMutex5);
                    currentDistance1 = measureDistance();
                    endTime1 = millis();
                    pthread_mutex_unlock(&sharedMutex5);
                    printf("Second measurement (Distance: %.2f cm)\n", currentDistance1);

                    if (currentDistance1 < previousDistance1) {
                        printf("Object is approaching. Calculating velocity...\n");
                        long timeElapsed1 = endTime1 - startTime1;
                        float distanceChange1 = previousDistance1 - currentDistance1;
                        pthread_mutex_lock(&sharedMutex5);
                        velocityReading = (distanceChange1 / timeElapsed1) * 1000;
                        pthread_mutex_unlock(&sharedMutex5);
                        printf("New Velocity: %.2f cm/s\n", velocityReading);

                        pthread_mutex_lock(&sharedMutex5);
                        shouldStopTimer = true;
                        currentState3 = FINISHED_2;
                        pthread_mutex_unlock(&sharedMutex5);
                    } else {
                        printf("Object moving away; restarting sensor check.\n");
                        currentState3 = READ_SENSOR2;
                    }
                    break;
                }
                case FINISHED_2:
                    pthread_mutex_lock(&sharedMutex5);
                    TrackState = STANDBY;
                    pthread_mutex_unlock(&sharedMutex5);
                    printf("Mode: Finished tracking new velocity\n");
                    printf("Speed adjustment initiated...\n");
                    currentState3 = READ_SENSOR2;
                    break;
                default:
                    printf("Unknown state!\n");
                    currentState3 = READ_SENSOR2;
                    break;
            }
        } else {
            printf("Thread 3: Velocity Tracking System is in STANDBY.\n");
            sleep(3);
        }
    }
    return NULL;
}

// Main program
int main(void) {
    pthread_t thread1, thread2, thread3;

    signal(SIGALRM, alarmWakeup);
    signal(SIGVTALRM, alarmWakeup2);

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

    if (wiringPiSetup() == -1) {
        printf("Setup failed!\n");
        return 1;
    }
    initUltrasonicSensor();
    initStepperMotor();
    pinMode(ADC_CS, OUTPUT);
    pinMode(ADC_CLK, OUTPUT);
    softPwmCreate(ServoPin, 0, 100);

    pthread_create(&thread1, NULL, ultrasonicAndMotorThread, NULL);
    pthread_create(&thread2, NULL, voltageAndSortingThread, NULL);
    pthread_create(&thread3, NULL, trackVelocityApproach, NULL);

    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);

    pthread_mutex_destroy(&sharedMutex1);
    pthread_mutex_destroy(&sharedMutex2);
    pthread_mutex_destroy(&sharedMutex3);
    pthread_mutex_destroy(&sharedMutex4);
    pthread_mutex_destroy(&sharedMutex5);

    return 0;
}
