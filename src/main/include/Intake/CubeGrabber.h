/**
 * Code for the dual-motor cube intake
*/
#include "ctre/Phoenix.h"

#include "IntakeConstants.h"

class CubeGrabber {
public:
    enum State {
        INTAKING,
        OUTTAKING,
        OUTTAKING_SLOW, //Default during teleop - to not catch rogue cubes
        STOPPED,
    };

    CubeGrabber();

    State getState();
    bool isIntaking();
    bool isOuttaking();

    void Intake();
    void Stop();
    void Outtake(); 
    void OuttakeSlow(); 

private:
    State grabber_status{STOPPED};
    WPI_TalonSRX  left_motor{CubeGrabberConstants::LEFT_MOTOR_ID};
    WPI_TalonSRX right_motor{CubeGrabberConstants::RIGHT_MOTOR_ID};
};