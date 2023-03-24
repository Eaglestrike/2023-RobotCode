/**
 * Code for the dual-motor cube intake
*/

#include "Constants.h"
#include "ctre/Phoenix.h"

class CubeGrabber {
public:
    enum State {
        INTAKING,
        OUTTAKING,
        STOPPED,
    };

    CubeGrabber();

    State getState();

    void Intake();
    void Stop();
    void Outtake(); 

private:
    State grabber_status{STOPPED};
    WPI_TalonSRX  left_motor{CubeGrabberConstants::LEFT_MOTOR_ID};
    WPI_TalonSRX right_motor{CubeGrabberConstants::RIGHT_MOTOR_ID};
};