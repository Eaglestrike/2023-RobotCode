/**
 * Code for the dual-motor cube intake
*/

#include "Constants.h"
#include "rev/CANSparkMax.h"

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
    rev::CANSparkMax left_motor{CubeGrabberConstants::LEFT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax right_motor{CubeGrabberConstants::RIGHT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
};