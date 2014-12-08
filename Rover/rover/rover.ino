
/**
 * @file rover.ino
 * @brief Arduino robot vehicle firmware.
 * @author Ryan Saunderson
 */

int enablePin1 = 3;  //Enable Pin 1 on L293
int motor1Pin1 = 4;  //Motor pin 2 on L293
int motor1Pin2 = 5;  // Motor pin 7 on L293
int enablePin2 = 10; // Enable Pin 9 on L293
int motor2Pin1 = 8;  // Motor Pin 10 on the L293
int motor2Pin2 = 9;  // Motor Pin 15 on L293
 
#define LOGGING

// Device drivers
// Enable one driver in each category

// Remote control:
//#define ENABLE_BLUESTICK_REMOTE_CONTROL_DRIVER
#define ENABLE_ROCKETBOT_REMOTE_CONTROL_DRIVER

// constants
#define TOO_CLOSE 10                    /**< distance to obstacle in centimeters */
#define MAX_DISTANCE (TOO_CLOSE * 20)   /**< maximum distance to track with sensor */
#define RANDOM_ANALOG_PIN 5             /**< unused analog pin to use as random seed */
#define BT_RX_PIN 16                    /**< RX pin for Bluetooth communcation */
#define BT_TX_PIN 17                    /**< TX pin for Bluetooth communcation */

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

#ifdef ENABLE_BLUESTICK_REMOTE_CONTROL_DRIVER
#include "bluestick_remote_control.h"
#define REMOTE_CONTROL_INIT
#endif

#ifdef ENABLE_ROCKETBOT_REMOTE_CONTROL_DRIVER
#include "rocketbot_remote_control.h"
#define REMOTE_CONTROL_INIT 10,50
#endif

#include "logging.h"
#include "moving_average.h"

namespace Rover
{
    class Robot
    {
    public:
        /*
         * @brief Class constructor.
         */
        Robot()
            : remoteControl(REMOTE_CONTROL_INIT)
        {
            initialize();
        }
        
        /*
         * @brief Initialize the robot state.
         */
        void initialize()
        {
            randomSeed(analogRead(RANDOM_ANALOG_PIN));
            remote();
        }
        
        /*
         * @brief Update the state of the robot based on input from sensor and remote control.
         *  Must be called repeatedly while the robot is in operation.
         */
        void run()
        {
            unsigned long currentTime = millis();
            RemoteControlDriver::command_t remoteCmd;
            bool haveRemoteCmd = remoteControl.getRemoteCommand(remoteCmd);
            log("state: %d, currentTime: %lu, remote: (%d,l:%d,r:%d,k:%d)\n", 
                state, currentTime,  
                haveRemoteCmd, remoteCmd.left, remoteCmd.right, remoteCmd.key);
            
            if (remoteControlled()) {
                if (haveRemoteCmd) {
                    switch (remoteCmd.key) {
                    case RemoteControlDriver::command_t::keyF1:
                        // start "roomba" mode
                        move();
                        break;
                    case RemoteControlDriver::command_t::keyNone:
                        // this is a directional command
                        leftMotor.setSpeed(remoteCmd.left);
                        rightMotor.setSpeed(remoteCmd.right);
                        break;
                    default:
                        break;
                    }
                }
            }
        }

    protected:
        void remote()
        {
            digitalWrite(motor1Pin1, LOW);   // set pin 2 on L293D low
			digitalWrite(motor1Pin2, LOW);  // set pin 7 on L293D low
			digitalWrite(motor2Pin1, LOW);   // set pin 10 on L293D low
			digitalWrite(motor2Pin2, LOW);  // set pin 15 on L293D low
            state = stateRemote;
        }
        
        void move()
        {
            digitalWrite(motor1Pin1, HIGH);   // set pin 2 on L293D high
			digitalWrite(motor1Pin2, LOW);  // set pin 7 on L293D low
			digitalWrite(motor2Pin1, HIGH);   // set pin 10 on L293D high
			digitalWrite(motor2Pin2, LOW);  // set pin 15 on L293D low
            state = stateMoving;
        }
        
        void stop()
        {
            digitalWrite(motor1Pin1, LOW);   // set pin 2 on L293D low
			digitalWrite(motor1Pin2, LOW);  // set pin 7 on L293D low
			digitalWrite(motor2Pin1, LOW);   // set pin 10 on L293D low
			digitalWrite(motor2Pin2, LOW);  // set pin 15 on L293D low
            state = stateStopped;
        }
        
        bool turn(unsigned long currentTime)
        {
            if (random(2) == 0) {
                digitalWrite(motor1Pin1, HIGH);   // set pin 2 on L293D high
				digitalWrite(motor1Pin2, LOW);  // set pin 7 on L293D low
				digitalWrite(motor2Pin1, LOW);   // set pin 10 on L293D low
				digitalWrite(motor2Pin2, HIGH);  // set pin 15 on L293D high
            }
            else {
                digitalWrite(motor1Pin1, LOW);   // set pin 2 on L293D low
				digitalWrite(motor1Pin2, HIGH);  // set pin 7 on L293D high
				digitalWrite(motor2Pin1, HIGH);   // set pin 10 on L293D high
				digitalWrite(motor2Pin2, LOW);  // set pin 15 on L293D low
            }
            state = stateTurning;
            endStateTime = currentTime + random(500, 1000);
        }
        
        bool doneTurning(unsigned long currentTime)
        {
            if (currentTime >= endStateTime)
            return false;
        }
        
        bool moving() { return (state == stateMoving); }
        bool turning() { return (state == stateTurning); }
        bool stopped() { return (state == stateStopped); }
        bool remoteControlled() { return (state == stateRemote); }

    private:
        RemoteControl remoteControl;
        enum state_t { stateStopped, stateMoving, stateTurning, stateRemote };
        state_t state;
        unsigned long endStateTime;
    };
};

Rover::Robot robot;

void setup()
{
    Serial.begin(9600);
    BTSerial.begin(9600);
	pinMode(enablePin1, OUTPUT);
	pinMode(motor1Pin1, OUTPUT);
	pinMode(motor1Pin2, OUTPUT);
	pinMode(enablePin2, OUTPUT);
	pinMode(motor2Pin1, OUTPUT);
	pinMode(motor2Pin2, OUTPUT);
	
	digitalWrite(enablePin1, HIGH);
	digitalWrite(enablePin2, HIGH);
}

void loop()
{
    robot.run();
}
