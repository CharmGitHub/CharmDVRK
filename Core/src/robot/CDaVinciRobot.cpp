#include "robot/CDaVinciRobot.h"
//===================================================================
CDaVinciRobot::CDaVinciRobot()
{

}

//===================================================================
CDaVinciRobot::~CDaVinciRobot()
{

}

//===================================================================
void CDaVinciRobot::Init()
{
    // Set the encoder value to a pre-determined value (probably
    // mid value to prevent overflowing/underflowing)
    for (int i = 0; i < 8; i++)
    {
        m_ioInfo.m_encoderBitsOffset[i] = 0x7FFFFF;
    }


    for (int i = 0; i < 4; i++)
    {
        p_amplifierBoard1->WriteEncoderPreload(i, 0x7FFFFF);
        p_amplifierBoard2->WriteEncoderPreload(i, 0x7FFFFF);
    }

    // Get the potentiometer joint angles
    FetchRawPotentiometer();
    CalculateJointAnglesPotentiometer();

    // Set the current encoder offset from the potentiometer joint angles
    SetEncoderFromPotentiometer();

    // Set the gravity compensation force value to zero
    m_gravityForce.Zeros();
}

//===================================================================
void CDaVinciRobot::UpdateAllVariables()
{
    FetchRawPotentiometer();
    CalculateJointAnglesPotentiometer();

    FetchRawEncoder();
    CalculateJointAnglesEncoder();

    FetchRawVelocity();
    CalculateJointVelocity();

    CalculateEECartesianPosRot();
    CalculateSpatialJacobian();
    CalculateEECartesianVelocity();

    FetchDigitalInput();
}

//===================================================================
void CDaVinciRobot::FetchRawEncoder()
{
    m_rawEncoder[0] = p_amplifierBoard1->GetEncoderPosition(0);
    m_rawEncoder[1] = p_amplifierBoard1->GetEncoderPosition(1);
    m_rawEncoder[2] = p_amplifierBoard1->GetEncoderPosition(2);
    m_rawEncoder[3] = p_amplifierBoard1->GetEncoderPosition(3);

    m_rawEncoder[4] = p_amplifierBoard2->GetEncoderPosition(0);
    m_rawEncoder[5] = p_amplifierBoard2->GetEncoderPosition(1);
    m_rawEncoder[6] = p_amplifierBoard2->GetEncoderPosition(2);
    m_rawEncoder[7] = p_amplifierBoard2->GetEncoderPosition(3);
}

//===================================================================
void CDaVinciRobot::FetchRawPotentiometer()
{
    m_rawPotentiometer[0] = p_amplifierBoard1->GetAnalogInput(0);
    m_rawPotentiometer[1] = p_amplifierBoard1->GetAnalogInput(1);
    m_rawPotentiometer[2] = p_amplifierBoard1->GetAnalogInput(2);
    m_rawPotentiometer[3] = p_amplifierBoard1->GetAnalogInput(3);

    m_rawPotentiometer[4] = p_amplifierBoard2->GetAnalogInput(0);
    m_rawPotentiometer[5] = p_amplifierBoard2->GetAnalogInput(1);
    m_rawPotentiometer[6] = p_amplifierBoard2->GetAnalogInput(2);
    m_rawPotentiometer[7] = p_amplifierBoard2->GetAnalogInput(3);
}

//===================================================================
void CDaVinciRobot::FetchRawVelocity()
{
    m_rawVelocityEncoder[0] = p_amplifierBoard1->GetEncoderVelocity(0);
    m_rawVelocityEncoder[1] = p_amplifierBoard1->GetEncoderVelocity(1);
    m_rawVelocityEncoder[2] = p_amplifierBoard1->GetEncoderVelocity(2);
    m_rawVelocityEncoder[3] = p_amplifierBoard1->GetEncoderVelocity(3);

    m_rawVelocityEncoder[4] = p_amplifierBoard2->GetEncoderVelocity(0);
    m_rawVelocityEncoder[5] = p_amplifierBoard2->GetEncoderVelocity(1);
    m_rawVelocityEncoder[6] = p_amplifierBoard2->GetEncoderVelocity(2);
    m_rawVelocityEncoder[7] = p_amplifierBoard2->GetEncoderVelocity(3);
}

//===================================================================
void CDaVinciRobot::FetchMotorFeedbackAmp()
{
    m_rawMotorFeedbackTorque[0] = p_amplifierBoard1->GetMotorCurrent(0);
    m_rawMotorFeedbackTorque[1] = p_amplifierBoard1->GetMotorCurrent(1);
    m_rawMotorFeedbackTorque[2] = p_amplifierBoard1->GetMotorCurrent(2);
    m_rawMotorFeedbackTorque[3] = p_amplifierBoard1->GetMotorCurrent(3);

    m_rawMotorFeedbackTorque[4] = p_amplifierBoard2->GetMotorCurrent(4);
    m_rawMotorFeedbackTorque[5] = p_amplifierBoard2->GetMotorCurrent(5);
    m_rawMotorFeedbackTorque[6] = p_amplifierBoard2->GetMotorCurrent(6);
    m_rawMotorFeedbackTorque[7] = p_amplifierBoard2->GetMotorCurrent(7);
}

//===================================================================
void CDaVinciRobot::CalculateJointAnglesEncoder()
{
    for (int i = 0; i < 8; i++)
    {
        m_actuatorAnglesEncoder[i] = ((double)m_rawEncoder[i] - (double)m_ioInfo.m_encoderBitsOffset[i])
                                                                     * m_ioInfo.m_encoderBitsToAngleDegScale[i]
                                                                     + m_ioInfo.m_encoderBitsToAngleDegOffset[i];
    }

    // Convert the units from degrees and millimeters to radians and meters
    for (int i = 0; i < 8; i++)
    {
        if (m_ioInfo.m_jointType[i] == REVOLUTE)
        {
            m_actuatorAnglesEncoderSI[i] = m_actuatorAnglesEncoder[i] / 180 * M_PI;
        }
        else
        {
            m_actuatorAnglesEncoderSI[i] = m_actuatorAnglesEncoder[i] * 0.001;
        }
    }

    // Calculate the actual joint angle with the coupling
    for (int i = 0; i < 8; i++)
    {
        double t_temp = 0;

        for (int j = 0; j < 8; j++)
        {
            t_temp = t_temp + m_ioInfo.m_couplingAcutatorToJointPosition[i][j] * m_actuatorAnglesEncoder[j];
        }

        m_jointAnglesEncoder[i] = t_temp + m_ioInfo.m_encoderZeroOffset[i];
    }

    // Convert the units from degrees and millimeters to
    // radians and meters
    for (int i = 0; i < 7; i++)
    {
        if (m_ioInfo.m_jointType[i] == REVOLUTE)
        {
            m_jointAnglesEncoderSI[i] = m_jointAnglesEncoder[i] / 180 * M_PI;
        }
        else
        {
            m_jointAnglesEncoderSI[i] = m_jointAnglesEncoder[i] * 0.001;
        }
    }
}

//===================================================================
void CDaVinciRobot::CalculateJointAnglesPotentiometer()
{
    for (int i = 0; i < 8; i++)
    {
        m_actuatorAnglePotentiometer[i] = (m_rawPotentiometer[i]*m_ioInfo.m_potentiometerBitsToVoltageScale[i]
                                                + m_ioInfo.m_potentiometerBitsToVoltageOffset[i])
                                                * m_ioInfo.m_potentiometerVoltageToAngleDegScale[i]
                                                + m_ioInfo.m_potentiometerVoltageToAngleDegOffset[i];
    }

    // Convert the units from degrees and millimeters to
    // radians and meters
    for (int i = 0; i < 8; i++)
    {
        if (m_ioInfo.m_jointType[i] == REVOLUTE)
        {
            m_actuatorAnglePotentiometerSI[i] = m_actuatorAnglePotentiometer[i] / 180 * M_PI;
        }
        else
        {
            m_actuatorAnglePotentiometerSI[i] = m_actuatorAnglePotentiometer[i] * 0.001;
        }
    }

    // Calculate the actual joint angle with the coupling
    for (int i = 0; i < 8; i++)
    {
        /*
        double t_temp = 0;

        for (int j = 0; j < 8; j++)
        {
            t_temp = t_temp + m_ioInfo.m_couplingAcutatorToJointPosition[i][j] * m_actuatorAnglePotentiometer[j];
        }
        */

        m_jointAnglesPotentiometer[i] = m_actuatorAnglePotentiometer[i];
    }

    // Convert the units from degrees and millimeters to
    // radians and meters
    for (int i = 0; i < 8; i++)
    {
        if (m_ioInfo.m_jointType[i] == REVOLUTE)
        {
            m_jointAnglesPotentiometerSI[i] = m_jointAnglesPotentiometer[i] / 180 * M_PI;
        }
        else
        {
            m_jointAnglesPotentiometerSI[i] = m_jointAnglesPotentiometer[i] * 0.001;
        }
    }
}

//===================================================================
void CDaVinciRobot::CalculateJointVelocity()
{
    // m_rawVelocityEncoder gives the number of clock pulses per encoder ticks
    // The middle value of m_rawVelocityEncoder is 32767, with the value decreasing
    // to zero in the positive direction of rotation and increasing to 65536 in the
    // negative direction of rotation

    double t_jointVelocityTime[8];
    double t_jointVelocityEncoderNoCoupling[8];
    double t_temp;

    for (int i = 0; i < 8; i++)
    {
        if (m_rawVelocityEncoder[i] >= (32767+10))
        {
            t_jointVelocityTime[i] = -(65536.0 - (double)m_rawVelocityEncoder[i]) / 768000;
        }
        else if (m_rawVelocityEncoder[i] <= (32767-10))
        {
            t_jointVelocityTime[i] = ((double)m_rawVelocityEncoder[i]) / 768000;
        }
        else
        {
            t_jointVelocityTime[i] = 100000;
        }
    }

    // Check for joint velocity error
    for (int i = 0; i < 8; i++)
    {
        if (t_jointVelocityTime[i] == 0)
        {
            t_jointVelocityTime[i] = 1;
        }
    }

    for (int i = 0; i < 8; i++)
    {
        t_jointVelocityEncoderNoCoupling[i] = 1 * m_ioInfo.m_encoderBitsToAngleDegScale[i] / t_jointVelocityTime[i];
    }

    // Calculate the actual joint velocity with the coupling
    for (int i = 0; i < 8; i++)
    {
        t_temp = 0;

        for (int j = 0; j < 8; j++)
        {
            t_temp = t_temp + m_ioInfo.m_couplingAcutatorToJointPosition[i][j] * t_jointVelocityEncoderNoCoupling[j];
        }

        m_jointVelocityEncoder[i] = t_temp;
    }

    // Convert the units from degrees/s and millimeters/s to radians/s and meters/s
    for (int i = 0; i < 7; i++)
    {
        if (m_ioInfo.m_jointType[i] == REVOLUTE)
        {
            m_jointVelocityEncoderSI[i] = m_jointVelocityEncoder[i] / 180 * M_PI;
        }
        else
        {
            m_jointVelocityEncoderSI[i] = m_jointVelocityEncoder[i] * 0.001;
        }
    }
}

//===================================================================
void CDaVinciRobot::CalculateEECartesianPosRot()
{
    vctDynamicVector<double> t_vector;
    vctFrame4x4<double> t_FKResults;

    t_vector.SetSize(7);
    t_vector.Zeros();

    for (int i = 0; i < 7; i++)
    {
        t_vector[i] = m_jointAnglesEncoderSI[i];
    }

    t_FKResults = m_robot.ForwardKinematics(t_vector);

    // Update the cartesian position of the robot end effector
    m_eeCartesianPosition[0] = (t_FKResults.Translation())[0];
    m_eeCartesianPosition[1] = (t_FKResults.Translation())[1];
    m_eeCartesianPosition[2] = (t_FKResults.Translation())[2];

    // Update the rotation matrix of the robot end effector
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            m_eeOrientation[i][j] = (t_FKResults.Rotation())[i][j];
        }
    }
}


//===================================================================
void CDaVinciRobot::CalculateSpatialJacobian()
{
    vctDynamicVector<double> t_vector;

    t_vector.SetSize(7);
    t_vector.Zeros();
    for (int i = 0; i < 7; i++)
    {
        t_vector[i] = m_jointAnglesEncoderSI[i];
    }

    m_robot.JacobianBody(t_vector);
    m_robot.JacobianSpatial(t_vector);

}

//===================================================================
void CDaVinciRobot::GetActuatorAnglesEncoder(double *q)
{
    for (int i = 0; i < 7; i++)
    {
        q[i] = m_actuatorAnglesEncoderSI[i];
    }

}

//===================================================================
void CDaVinciRobot::GetActuatorAnglesPotentiometer(double *q)
{
    for (int i = 0; i < 7; i++)
    {
        q[i] = m_actuatorAnglePotentiometerSI[i];
    }
}

//===================================================================
void CDaVinciRobot::GetJointAnglesEncoder(double *q)
{
    for (int i = 0; i < 7; i++)
    {
        q[i] = m_jointAnglesEncoderSI[i];
    }

}

//===================================================================
void CDaVinciRobot::GetJointAnglesPotentiometer(double *q)
{
    for (int i = 0; i < 7; i++)
    {
        q[i] = m_jointAnglesPotentiometerSI[i];
    }
}

//===================================================================
void CDaVinciRobot::GetJointVelocityEncoder(double *dq)
{
    for (int i = 0; i < 7; i++)
    {
        dq[i] = m_jointVelocityEncoderSI[i];
    }
}

//===================================================================
bool CDaVinciRobot::GetDigitalInput(int ChannelNum)
{
    if (ChannelNum < 12)
    {
        return m_digitalInput[ChannelNum];
    }
}

//===================================================================
cVector3d CDaVinciRobot::GetEECartesianPosition()
{
    cVector3d t_position;

    t_position[0] = m_eeCartesianPosition[0];
    t_position[1] = m_eeCartesianPosition[1];
    t_position[2] = m_eeCartesianPosition[2];

    return t_position;
}

//===================================================================
cVector3d CDaVinciRobot::GetEECartesianLinearVelocity()
{
    cVector3d t_linearVelocity;

    t_linearVelocity[0] = m_eeCartesianVelocityLinear[0];
    t_linearVelocity[1] = m_eeCartesianVelocityLinear[1];
    t_linearVelocity[2] = m_eeCartesianVelocityLinear[2];

    return t_linearVelocity;
}

//===================================================================
cVector3d CDaVinciRobot::GetEECartesianAngularVelocity()
{
    cVector3d t_angularVelocity;

    t_angularVelocity[0] = m_eeCartesianVelocityRotary[0];
    t_angularVelocity[1] = m_eeCartesianVelocityRotary[1];
    t_angularVelocity[2] = m_eeCartesianVelocityRotary[2];

    return t_angularVelocity;
}

//===================================================================
cMatrix3d CDaVinciRobot::GetEEOrientation()
{
    /*
    cMatrix3d t_Orientation;
    t_Orientation.Element(0,0) = m_eeOrientation
    t_Orientation.setCol0(cVector3d(m_eeOrientation[0][0], m_eeOrientation[1][0], m_eeOrientation[2][0]));
    t_Orientation.setCol1(cVector3d(m_eeOrientation[0][1], m_eeOrientation[1][1], m_eeOrientation[2][1]));
    t_Orientation.setCol2(cVector3d(m_eeOrientation[0][2], m_eeOrientation[1][2], m_eeOrientation[2][2]));
    */

    return m_eeOrientation;
}

//===================================================================
void CDaVinciRobot::SetForceTorqueEnable(bool a_enable)
{
    p_amplifierBoard1->SetAmpEnable(0, a_enable);
    p_amplifierBoard1->SetAmpEnable(1, a_enable);
    p_amplifierBoard1->SetAmpEnable(2, a_enable);
    p_amplifierBoard1->SetAmpEnable(3, a_enable);
    p_amplifierBoard2->SetAmpEnable(0, a_enable);
    p_amplifierBoard2->SetAmpEnable(1, a_enable);
    p_amplifierBoard2->SetAmpEnable(2, a_enable);
    p_amplifierBoard2->SetAmpEnable(3, a_enable);
}

//===================================================================
void CDaVinciRobot::SetPowerEnable(bool a_enable)
{
    p_amplifierBoard1->SetPowerEnable(a_enable);
    p_amplifierBoard2->SetPowerEnable(a_enable);
}

//===================================================================
void CDaVinciRobot::SetEncoderFromPotentiometer()
{
    for (int i = 0; i < 8; i++)
    {
        m_ioInfo.m_encoderZeroOffset[i] = m_jointAnglesPotentiometer[i];
    }
}

// ===============================================================================
void CDaVinciRobot::SetGravityForceValue(cVector3d a_gravityForce)
{
    // Set the gravity compensation force value
    m_gravityForce = a_gravityForce;
}

// ===============================================================================
void CDaVinciRobot::SetSafetyRelay(bool a_enable)
{
    // Set the safety relay
    p_amplifierBoard1->SetSafetyRelay(a_enable);
    p_amplifierBoard2->SetSafetyRelay(a_enable);
}

// ===============================================================================
void CDaVinciRobot::SetJointLimitsEnable(bool a_enable)
{
    m_flagJointLimitEnable = a_enable;
}

// ===============================================================================
