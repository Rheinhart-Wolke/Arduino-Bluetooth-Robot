/**
 * @file remote_control.h
 * @brief remote control driver definition for the rover robot.
 * @author Ryan Saunderson
 */

int enablePin1 = 3;  //Enable Pin 1 on L293
int motor1Pin1 = 4;  //Motor pin 2 on L293
int motor1Pin2 = 5;  // Motor pin 7 on L293
int enablePin2 = 10; // Enable Pin 9 on L293
int motor2Pin1 = 8;  // Motor Pin 10 on the L293
int motor2Pin2 = 9;  // Motor Pin 15 on L293
 
namespace Rover
{
    class RemoteControlDriver
    {
    public:
        /**
          * @brief abstract representation of a remote command.
          */
        struct command_t {
            enum key_t { keyNone, keyF1, keyF2, keyF3, keyF4 };
            key_t key;  /**< function key. */
            
            command_t() : key(keyNone) {}
            void goForward()
            {
            digitalWrite(motor1Pin1, HIGH);
			digitalWrite(motor1Pin2, LOW);
			digitalWrite(motor2Pin1, HIGH);
			digitalWrite(motor2Pin2, LOW);
            }
            void goBack()
            {
            digitalWrite(motor1Pin1, LOW);
			digitalWrite(motor1Pin2, LOW);
			digitalWrite(motor2Pin1, LOW);
			digitalWrite(motor2Pin2, LOW);
            }
            void turnLeft()
            {
            digitalWrite(motor1Pin1, High);
			digitalWrite(motor1Pin2, LOW);
			digitalWrite(motor2Pin1, LOW);
			digitalWrite(motor2Pin2, LOW);
            }
            void turnRight()
            {
            digitalWrite(motor1Pin1, LOW);
			digitalWrite(motor1Pin2, LOW);
			digitalWrite(motor2Pin1, HIGH);
			digitalWrite(motor2Pin2, LOW);
            }
            void stop()
            {
            digitalWrite(motor1Pin1, LOW);
			digitalWrite(motor1Pin2, LOW);
			digitalWrite(motor2Pin1, LOW);
			digitalWrite(motor2Pin2, LOW);
            }
            void leftAndRightSliders(int l, int r)
            {
                left = l;
                right = r;
            }
        };
        
        /**
          * @brief Class constructor.
          */
        RemoteControlDriver() {}

        /**
         * @brief Return the next remote command, if available.
         * @param cmd a reference to a command_t struct where the command
         *   information will be stored.
         * @return true if a remote command is available, false if not.
         */
        virtual bool getRemoteCommand(command_t& cmd) = 0;
    };
};
