#include "motor.h"
#include "angleSensor.h"
#include "usb.h"

#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>
#include <Wire.h>

// #include "Keyboard.h"
// #include <MediaKeyboard.h>

#define VOLTAGE_SUPPLY (5U)   // V
#define CURRENT_LIMIT  (1.0f) //  Amps
#define VELOCITY_LIMIT (10.0f)
#define VELOCITY_LPF   (0.005f) // the lower the less filtered

static QueueHandle_t modeQueue;
static motor_state_E state = HOLD;
static float target_angle = 0;
static float current_angle = 0;
// instantiate the commander
// Commander command = Commander(Serial);
// void onMotor(char* cmd){command.motor(&motor,cmd);}
// void onPid(char* cmd){command.pid(&torquePID,cmd);}

static void click(BLDCMotor* motor, float torque, int duration_ms)
{
  // Delay for 1ms
  // const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  // motor->motion_downsample = 0;
  // motor->move(torque);
  // for (int i = 0; i<duration_ms; i++)
  // {
  //   motor->loopFOC();
  //   vTaskDelay(xDelay);
  // }
  // motor->move(-torque);
  // for (int i = 0; i<duration_ms; i++)
  // {
  //   motor->loopFOC();
  //   vTaskDelay(xDelay);
  // }
  // motor->move(0);
}

static void TaskMotor(void *pvParameters)
{
  (void) pvParameters;
  modeQueue = xQueueCreate( 5, sizeof( motor_state_E ) );

  // Run every 1ms
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  // Initialize keyboard
  // Keyboard.begin();
  // MediaKeyboard.begin();
  int time = micros();
  int volumeCount = 0;

  // BLDC motor & driver instance
  // BLDCMotor( pp number , phase resistance)
  BLDCMotor motor = BLDCMotor(7);
  BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6);
  PIDController torquePID = PIDController(25, 0.1, 0.21, 1000, VOLTAGE_SUPPLY);
  // Angle sensor
  GenericSensor sensor = GenericSensor(getAngleRad, initAngleSensor);

  
  motor_state_E prevState = HOLD;
  
  float forceLPF = 0.15f; // Set to 1 for no filter
  float angleLPF = 0.9f; // Set to 1 for no filter
  float prevAngle = 0.0f;
  float downsample = 0.0f;

  // initialize motor
  driver.voltage_power_supply = VOLTAGE_SUPPLY;
  driver.init();

  // init angle sensor hardware with fast i2c readings
  sensor.init();
  // Wire.begin();
  // Wire.setClock(1000000L);
  // link the motor and the driver and sensor
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.current_limit = CURRENT_LIMIT;
 
  // closed-loop torque control config
  motor.controller = MotionControlType::torque;

  motor.LPF_velocity.Tf = VELOCITY_LPF;
  motor.velocity_limit = VELOCITY_LIMIT;

  // downsample motion to prevent motor jitter
  motor.motion_downsample = (int)downsample;

  // init motor
  // motor.useMonitoring(Serial);
  // motor.init();
  // motor.initFOC(); // calibration params

  // // add motor command M
  // command.add('M',onMotor,"motor command");
  // command.add('C',onPid,"torque pid command");

  if( modeQueue != 0 )
  {
    if(xQueueSend(modeQueue, ( void * ) &state, (TickType_t)10) != pdPASS)
    {
        /* Failed to post the message, even after 10 ticks. */
    }
  }

  while(1)
  {
    xQueueReceive(modeQueue, &state, (TickType_t)0);
    // sensor.update(); // Update manually until FOC works
    float currentAngleFiltered = angleLPF*sensor.getAngle() + (1-angleLPF)*prevAngle;
    float outputTorque = torquePID(target_angle-currentAngleFiltered);
    int localCurrentForce = 0;
    bool forceValid = false;

    switch(state)
    {
      case HOLD:
      {
        // change PID for hold to prevent overshoot
        break;
      }
      case SELECT:
      {
        if (sensor.getAngle() > target_angle + 0.785398) {
          target_angle += 2*0.785398;
        } else if (sensor.getAngle() < target_angle - 0.785398) {
          target_angle -= 2*0.785398;
        }
        break;
      }
      case SMALL_SELECT:
      {
        if (sensor.getAngle() > target_angle + 0.0436332) {
          target_angle += 2*0.0436332;
          // volumeCount++;
          // sendKey(VOLUME_UP);
          click(&motor, 2, 1);
        } else if (sensor.getAngle() < target_angle - 0.0436332) {
          target_angle -= 2*0.0436332;
          // sendKey(VOLUME_DOWN);
          // volumeCount--;
          click(&motor, 2, 1);
        }
        break;
      }
      case BUTTON_PRESS:
      {
        click(&motor, 4, 3);
        
        state = prevState;
        break;
      }
      case BUTTON_RELEASE:
      {
        // Keyboard.write((uint8_t)'a');
        click(&motor, 1, 3);
        
        state = prevState;
        // Change state on release
        state = motor_state_E((state + 1U) % BUTTON_PRESS);
        break;
      }
      case FREE:
      default:
      {
        target_angle = sensor.getAngle();
        outputTorque = 0;
        break;
      }
    }

    // motor.loopFOC();
    // motor.move(outputTorque);

    // if(volumeCount >= 5) {
      
    //   MediaKeyboard.press(VOLUME_UP);
    //   volumeCount = 0;
    // }
    // if (volumeCount <= -5)
    // {
    //   MediaKeyboard.press(VOLUME_DOWN);
    //   volumeCount = 0;
    // }
    // Serial.println(micros() - time);
    // time = micros();
    // command.run();
    // Serial.println(currentAngleFiltered);
    prevAngle = currentAngleFiltered;
    current_angle = prevAngle;
    prevState = state;
    vTaskDelay(xDelay);
  }
}

void initMotorTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskMotor
    ,  (const portCHAR *)"Motor"
    ,  512
    ,  NULL
    ,  priority
    ,  NULL );
}

int16_t getMotorAngleDeg(void)
{
    return (int16_t)(current_angle*(RAD_TO_DEG));
}

int16_t getMotorTargetAngleDeg(void)
{
    return (int16_t)(target_angle*RAD_TO_DEG);
}

void sendMotorState(motor_state_E state)
{
    xQueueSend(modeQueue, ( void * ) &state, (TickType_t)10);
}

motor_state_E getMotorState(void)
{
    return state;
}
