#include "robot/CDaVinciRobot.h"
//===================================================================
class CDaVinciRobotMTM : public CDaVinciRobot
{
public:
    //! Constructor for CDaVinciRobotMTM class
    CDaVinciRobotMTM();
    CDaVinciRobotMTM(FirewirePort *a_port, int a_boardNum1, int a_boardNum2);

    //! Destructor for CDaVinciRobotMTM class
    ~CDaVinciRobotMTM();

    //! Functions to get the various robot variables
    void SetupRobotParam(const std::string& a_kinematicFilename, const std::string& a_IOCharacterFilename);
    double GetGripperAngle();

    //! Functions to get torques / forces
    void SetForceTorqueCartesian(cVector3d a_force, cVector3d a_torque);
    void SetForceTorqueJoint(double *a_torque);

    //! Input from the foot pedal
    bool GetPedalClutch();
    void GetFootPedalInputs(bool *footPedalInputs);

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
