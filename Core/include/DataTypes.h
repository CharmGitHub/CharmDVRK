#ifndef DATATYPES_H
#define DATATYPES_H

#include "io/AmpIO.h"

enum JointType {REVOLUTE, PRISMATIC};

typedef struct
{
    JointType m_jointType[8];
    double m_driveAmpsToBitsOffset[8];
    double m_driveAmpsToBitsScale[8];
    double m_driveBitsToFeedbackAmpsOffset[8];
    double m_driveBitsToFeedbackAmpsScale[8];
    double m_driveTorqueNmToAmpsScale[8];
    double m_driveMaxAmpsValue[8];

    AmpIO_UInt32 m_encoderBitsOffset[8];
    double m_encoderBitsToAngleDegOffset[8];
    double m_encoderBitsToAngleDegScale[8];
    double m_encoderBitsToAngleVelocityDegPSecOffset[8];
    double m_encoderBitsToAngleVelocityDegPSecScale[8];
    double m_encoderZeroOffset[8];

    double m_potentiometerBitsToVoltageOffset[8];
    double m_potentiometerBitsToVoltageScale[8];
    double m_potentiometerVoltageToAngleDegOffset[8];
    double m_potentiometerVoltageToAngleDegScale[8];

    double m_couplingJointToActuatorPosition[8][8];
    double m_couplingAcutatorToJointPosition[8][8];
    double m_couplingJointToActuatorTorque[8][8];
    double m_couplingAcutatorToJointTorque[8][8];
} IOInfo;

#endif
