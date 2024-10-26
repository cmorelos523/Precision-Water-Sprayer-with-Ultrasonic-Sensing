
////////////////////////////////////////////////
// Kyshawn Savone-Warren
// Carlos Morelos Escalera
//
// Lab 4 - FreeRToS
//
////////////////////////////////////////////////

// Importing Libraries 
#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include <Stepper.h>
#include <queue.h>
#include <task.h>


// Define the number of steps per revolution (depends on your stepper motor)
#define STEPS_PER_REV 200
#define STEPS_PER_180 (STEPS_PER_REV / 2)
#define OFFB_LED 22
#define trigPin 9
#define echoPin 10
#define externalPin 12

// Define statements for Speaker
#define E 659 // Hz
#define C 523 // Hz
#define G 784 // Hz
#define g 392 // Hz
#define R 0 
#define BIT_3 1<<3 // digital pin 6

// Song array
int song[] = {E, R, E, R, R, E, R, R, C, R, E, R, R, G, R, R, R, R, R, g, R};

// #define LED_pin = 8
// Global variables for the sensor 
float duration, distance;

// Initialize the Stepper library on pins 2, 4, 3, 5
Stepper myStepper(STEPS_PER_REV, 2, 4, 3, 5);


// define tasks 
void TaskBlink( void *pvParameters );
void TaskSpeaker( void *pvParameters );
void TaskBlinkOffBoard( void *pvParameters );
void TaskJoyStick( void *pvParameters );
void TaskStepperMotor( void *pvParameters );
void TaskSensor( void *pvParameters );
void TaskExternal( void *pvParameters );

// Queue to assign values to the stepper motor
QueueHandle_t x_values;


// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  } 
  TaskHandle_t xTaskExternalHandle; // Initilize handles

  // Create tasks for independance
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  xTaskCreate(
    TaskSpeaker
    ,  "Speaker"   // A name just for humans
    ,  128  // This stack size 
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  xTaskCreate(
    TaskBlinkOffBoard
    ,  "BlinkOffBoard"   // A name just for humans
    ,  128  // This stack size 
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  xTaskCreate(
    TaskJoyStick
    ,  "JoyStick"
    ,  128  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  xTaskCreate(
    TaskStepperMotor
    ,  "StepperMotor"
    ,  128  // Stack size
    ,  NULL
    ,  0 // Priority
    ,  NULL );

  xTaskCreate(
    TaskExternal
    ,  "External"
    ,  1000  // Stack size
    ,  NULL
    ,  4  // Priority
    ,  &xTaskExternalHandle ); // Handle name 


  xTaskCreate(
    TaskSensor
    ,  "Sensor"
    ,  1000  // Stack size
    ,  (void *) xTaskExternalHandle //Can call this handle
    ,  4  // Priority
    ,  NULL );
  
  
  x_values = xQueueCreate(10, sizeof(int)); // A queue if size 10

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  //  (note how the above comment is WRONG!!!)
  vTaskStartScheduler();

  for(;;);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
 // (void) pvParameters;  // allocate stack space for params
  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 250 / portTICK_PERIOD_MS ); // wait for 250 ms
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 ms
  }
}
// Task for playing speaker and stopping the speaker after 3 plays 
void TaskSpeaker( void *pvParameters ) {
    // put your setup code here, to run once:
  TCCR2B = 0x001;
  // TCCR2B |= (1<<CS22) | (1<<CS20); // Prescaler 128
  // TCCR2B &= ~(1<<CS21); // CS22 = 1, CS21 = 0, CS20 = 1 -> prescale 128
  // Speaker setup
  DDRH |= BIT_3; // digital pin 6
  TCNT4 = 0;
  int i = 0;
  int count = 0;
  TCCR4A = 0b01000000;
  TCCR4B = 0b00001010;

// Forever loop
  for(;;) {
    if (i == 21) {
      i = 0;
      vTaskDelay( 1500 / portTICK_PERIOD_MS ); // wait for one second
      count++;
      if (count == 3) {
        vTaskSuspend(NULL);
      }
    }
    else {
      OCR4A = (16000000 / (2 * 8 * (song[i] - 1))) ;
      i++;
      vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for one second
    }
  }
}

// Task to blink the offboard LED
void TaskBlinkOffBoard( void *pvParameters ) {
  pinMode(OFFB_LED, OUTPUT);

  // Forver loop
  for(;;) {
    digitalWrite(OFFB_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS); // wait 100 ms
    digitalWrite(OFFB_LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS); // wait 100 ms
  }
}

// Task for controlling the joystick
void TaskJoyStick(void *pvParameters)  // This is a task.
{
  // Forever loop
  for (;;)
  {
    // read the input on analog pin 1:
    int sensorValue = analogRead(A1);  
    xQueueSend(x_values, &sensorValue, 0); // add values to a queue 
    // print out the value you read:
    Serial.println(sensorValue);
    vTaskDelay(500/portTICK_PERIOD_MS);  // 500 ms in between reads for stability
  }
}


void TaskStepperMotor(void *pvParameters)  // This is a for controlling stepper motor.
{
  int stepsToMove; //the value from the queue

  for (;;)
  {
    xQueueReceive(x_values, &stepsToMove, 0); // assign value 
    myStepper.setSpeed(STEPS_PER_REV); // speed
    // myStepper.step(1);
    // move right
    if (stepsToMove > 530) {
      myStepper.step(10);
    }
    // move left
    else if (stepsToMove < 500) {
      myStepper.step(-10);
    }
  }
}

// Task for taking input from the Ultrasonic sensor 
void TaskSensor( void * pvParameters ) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  TaskHandle_t xTaskExternalHandle = (TaskHandle_t) pvParameters; // used to notify the external task
// Forever loop that goes through the cycle of taking input 
  for(;;) {
    digitalWrite(trigPin, LOW);
    vTaskDelay(2);
    digitalWrite(trigPin, HIGH);
    vTaskDelay(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    // Serial.print("Distance: ");
    // Serial.println(distance);
    vTaskDelay(20);
    // If too close turn on motor 
    if (distance < 10) {
      xTaskNotify(xTaskExternalHandle, 1, eNoAction);
    }
  }
}
// Task for the external motor 
void TaskExternal ( void *pvParameters ) {
  pinMode(externalPin, OUTPUT);
  uint32_t ulNotificationValue;// Take in notification
  
// Forever loop that runs through process of turning on motor
  for (;;) {
    // Serial.println("distance");
    ulNotificationValue = ulTaskNotifyTake(pdFALSE, portMAX_DELAY); // Wait until the notification is received to run the task
      //Serial.println("distance");

      digitalWrite(externalPin, HIGH);
      vTaskDelay(50);
      digitalWrite(externalPin, LOW);
  }  
}

