#ifndef CDAVINCIROBOT_H
#define CDAVINCIROBOT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdexcept>

#include "io/FirewirePort.h"
#include "io/AmpIO.h"
#include "xmlreader/xmlReader.h"
#include "DataTypes.h"
#include "cisstRobot.h"
//===================================================================
typedef vctFixedSizeVector<double, 3> cVector3d;
typedef vctFixedSizeMatrix<double, 3, 3> cMatrix3d;

//===================================================================
class CDaVinciRobot
{
public:
    //! Constructor for CDaVinciRobot class
    CDaVinciRobot();

    //! Destructor for CDaVinciRobot class
    ~CDaVinciRobot();

    //! Initialization
    void Init();

    //! Functions to get the various robot variables
    void GetActuatorAnglesEncoder(double *q);
    void GetActuatorAnglesPotentiometer(double *q);
    void GetJointAnglesEncoder(double *q);
    void GetJointAnglesPotentiometer(double *q);
    void GetJointVelocityEncoder(double *dq);
    bool GetDigitalInput(int ChannelNum);
    cVector3d GetEECartesianPosition();
    cVector3d GetEECartesianLinearVelocity();
    cVector3d GetEECartesianAngularVelocity();
    cMatrix3d GetEEOrientation();

    //! Functions to get torques / forces
    virtual void SetForceTorqueJoint(double *a_torque) = 0;

    //! Misc functions
    void SetForceTorqueEnable(bool a_enable);
    void SetPowerEnable(bool a_enable);
    void SetEncoderFromPotentiometer();
    void SetGravityForceValue(cVector3d a_gravityForce);
    void SetSafetyRelay(bool a_enable);
    virtual void SetJointLimitsUpper(double *a_qlimits) = 0;
    virtual void SetJointLimitsLower(double *a_qlimits) = 0;
    virtual void SetJointLimitsStiffness(double *a_qlimitsK) = 0;
    virtual void SetJointLimitsDamping(double *a_qlimitsD) = 0;
    void SetJointLimitsEnable(bool a_enable);

    //! Function to update all variables
    void UpdateAllVariables();

protected:

    // ------------------ Private members --------------------
    //! Device Input/Output
    FirewirePort *p_fireWirePort;
    AmpIO *p_amplifierBoard1;
    AmpIO *p_amplifierBoard2;    

    //! Cisst robManipulator class to calculate robot kinematics and Jacobian
    robManipulator m_robot;

    //! IO information for the robot
    IOInfo m_ioInfo;

    //! Robot raw information
    bool m_digitalInput[12];
    AmpIO_UInt32 m_rawEncoder[8];
    AmpIO_UInt32 m_rawPotentiometer[8];
    AmpIO_UInt32 m_rawVelocityEncoder[8];
    AmpIO_UInt32 m_rawMotorFeedbackTorque[8];

    //! Calculated joint angles information
    double m_actuatorAnglesEncoder[8];
    double m_actuatorAnglesEncoderSI[8];
    double m_actuatorAnglePotentiometer[8];
    double m_actuatorAnglePotentiometerSI[8];
    double m_jointAnglesEncoder[8];
    double m_jointAnglesEncoderSI[8];
    double m_jointAnglesPotentiometer[8];
    double m_jointAnglesPotentiometerSI[8];
    double m_jointVelocityEncoder[8];
    double m_jointVelocityEncoderSI[8];

    //! Robot end effector position, rotation, and velocity
    cVector3d m_eeCartesianPosition;
    cVector3d m_eeCartesianVelocityLinear;
    cVector3d m_eeCartesianVelocityRotary;
    cMatrix3d m_eeOrientation;

    //! Robot Feedback Torques
    double m_motorFeedbackTorque[8];
    double m_jointFeedbackTorque[8];

    //! Gravity compensation force value
    cVector3d m_gravityForce;

    //! Potential field torques to impose software joint limits (in radians)
    double m_jointLimitTorques[8];
    double m_jointLimitUpper[8];
    double m_jointLimitLower[8];
    double m_jointLimitStiffness[8];
    double m_jointLimitDamping[8];
    bool m_flagJointLimitEnable;

    // ------------------ Private functions ---------------------
    virtual void SetupRobotParam(const std::string& a_kinematicFilename, const std::string& a_IOCharacterFilename) = 0;

    // Functions to fetch data from the controller
    void FetchRawEncoder();
    void FetchRawPotentiometer();
    void FetchRawVelocity();
    void FetchMotorFeedbackAmp();
    virtual void FetchDigitalInput() = 0;

    // Functions to calculate the joint angles and velocities based on the fetched data from the controller
    void CalculateJointAnglesEncoder();
    void CalculateJointAnglesPotentiometer();
    void CalculateJointVelocity();

    // Functions to determine the end effector position, rotation, and velocity
    void CalculateEECartesianPosRot();
    virtual void CalculateEECartesianVelocity() = 0;
    void CalculateSpatialJacobian();
    virtual void CalculateJointFeedbackTorque() = 0;

    // Function to calculate the software joint limit torques
    virtual void CalculateJointLimitTorques() = 0;
};
//===================================================================

#endif
