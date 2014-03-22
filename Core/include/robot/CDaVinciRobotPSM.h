#include "robot/CDaVinciRobot.h"
//===================================================================
class CDaVinciRobotPSM : public CDaVinciRobot
{
public:
    //! Constructor for CDaVinciRobotPSM class
    CDaVinciRobotPSM();
    CDaVinciRobotPSM(FirewirePort *a_port, int a_boardNum1, int a_boardNum2);

    //! Destructor for CDaVinciRobotPSM class
    ~CDaVinciRobotPSM();

    //! Functions to get the various robot variables
    void SetupRobotParam(const std::string& a_kinematicFilename, const std::string& a_IOCharacterFilename);
    double GetGripperAngle();
    double GetGripperVelocity();

    //! Functions to get torques / forces
    void SetForceTorqueCartesian(cVector3d a_force, cVector3d a_torque, double a_gripperTorque);
    void SetForceTorqueJoint(double *a_torque);

    //! Input from the buttons on the PSM arm
    bool GetInputTool();
    bool GetInputShoulder();

    //! Misc functions
    void SetJointLimitsUpper(double *a_qlimits);
    void SetJointLimitsLower(double *a_qlimits);
    void SetJointLimitsStiffness(double *a_qlimitsK);
    void SetJointLimitsDamping(double *a_qlimitsD);

private:

    void FetchDigitalInput();
    void CalculateEECartesianVelocity();
    void CalculateJointFeedbackTorque();
    void CalculateJointLimitTorques();
};
//===================================================================
