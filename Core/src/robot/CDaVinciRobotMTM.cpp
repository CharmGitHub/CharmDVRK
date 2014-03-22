#include "robot/CDaVinciRobotMTM.h"
//===================================================================
CDaVinciRobotMTM::CDaVinciRobotMTM()
{

}

//===================================================================
CDaVinciRobotMTM::~CDaVinciRobotMTM()
{

}

//===================================================================
CDaVinciRobotMTM::CDaVinciRobotMTM(FirewirePort *a_port, int a_boardNum1, int a_boardNum2)
{
    p_fireWirePort = a_port;

    p_amplifierBoard1 = new AmpIO(a_boardNum1);
    p_amplifierBoard2 = new AmpIO(a_boardNum2);

    bool t_success1 = p_fireWirePort->AddBoard(p_amplifierBoard1);
    bool t_success2 = p_fireWirePort->AddBoard(p_amplifierBoard2);

    if ((t_success1 != true) || (t_success2 != true))
    {
        // Throw an exception
        throw std::runtime_error("Initialization of board failed!");
    }

    // Set the joint limit enable to off
    m_flagJointLimitEnable = false;
}

//===================================================================
void CDaVinciRobotMTM::SetupRobotParam(const std::string& a_kinematicFilename, const std::string& a_IOCharacterFilename)
{
    robManipulator::Errno t_errorNum = m_robot.LoadRobot(a_kinematicFilename);

    if (t_errorNum == robManipulator::EFAILURE)
    {
        // Throw an exception
        throw std::runtime_error("Initialization of robot kinematic failed!\n");
    }

    std::string t_robotType = xmlReaderGetRobotType(a_IOCharacterFilename.c_str());

    if (strcmp(t_robotType.c_str(), "MTM") != 0)
    {
        std::stringstream s;
        s << "Robot type " << t_robotType << "is not compatible with MTM type robot!\n";
        throw std::runtime_error(s.str());
    }
    else
    {
        xmlReaderGetRobotInfo(a_IOCharacterFilename.c_str(), m_ioInfo);
    }

    return;
}


//===================================================================
void CDaVinciRobotMTM::CalculateEECartesianVelocity()
{
    double t_manipulatorJacobian[6][7];
    vctDynamicVector<double> t_velVector;
    vctDynamicVector<double> t_linearVelocity;
    vctDynamicVector<double> t_angularVelocity;

    // Calculate the end effector manipulator Jacobian using the spatial jacobian
    // and the current end effector position
    for (int j = 0; j < 7; j++)
    {
        t_manipulatorJacobian[0][j] = m_robot.Js[j][0] + m_robot.Js[j][4] * m_eeCartesianPosition[2] - m_robot.Js[j][5] * m_eeCartesianPosition[1];
        t_manipulatorJacobian[1][j] = m_robot.Js[j][1] + m_robot.Js[j][5] * m_eeCartesianPosition[0] - m_robot.Js[j][3] * m_eeCartesianPosition[2];
        t_manipulatorJacobian[2][j] = m_robot.Js[j][2] + m_robot.Js[j][3] * m_eeCartesianPosition[1] - m_robot.Js[j][4] * m_eeCartesianPosition[0];
        t_manipulatorJacobian[3][j] = m_robot.Js[j][3];
        t_manipulatorJacobian[4][j] = m_robot.Js[j][4];
        t_manipulatorJacobian[5][j] = m_robot.Js[j][5];
    }

    t_velVector.SetSize(7);
    t_velVector.Zeros();
    for (int i = 0; i < 7; i++)
    {
        t_velVector[i] = m_jointVelocityEncoderSI[i];
    }

    // Calculate the linear velocity of the end effector
    t_linearVelocity.SetSize(3);
    t_linearVelocity.Zeros();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            t_linearVelocity[i] = t_linearVelocity[i] + t_manipulatorJacobian[i][j] * t_velVector[j];
        }
    }

    // Calculate the angular velocity of the end effector
    t_angularVelocity.SetSize(3);
    t_angularVelocity.Zeros();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            t_angularVelocity[i] = t_angularVelocity[i] + t_manipulatorJacobian[i+3][j] * t_velVector[j];
        }
    }

    m_eeCartesianVelocityLinear[0] = t_linearVelocity[0];
    m_eeCartesianVelocityLinear[1] = t_linearVelocity[1];
    m_eeCartesianVelocityLinear[2] = t_linearVelocity[2];

    m_eeCartesianVelocityRotary[0] = t_angularVelocity[0];
    m_eeCartesianVelocityRotary[1] = t_angularVelocity[1];
    m_eeCartesianVelocityRotary[2] = t_angularVelocity[2];
}

//===================================================================
void CDaVinciRobotMTM::CalculateJointFeedbackTorque()
{
    for (int i = 0; i < 7; i++)
    {
        m_motorFeedbackTorque[i] = m_ioInfo.m_driveBitsToFeedbackAmpsScale[i] * m_rawMotorFeedbackTorque[i] / m_ioInfo.m_driveTorqueNmToAmpsScale[i];
    }

    for (int i = 0; i < 7; i++)
    {
        m_jointFeedbackTorque[i] = 0;

        for (int j = 0; j < 7; i++)
        {
            m_jointFeedbackTorque[i] = m_ioInfo.m_couplingAcutatorToJointTorque[i][j] * m_motorFeedbackTorque[j];
        }
    }
}

//===================================================================
void CDaVinciRobotMTM::CalculateJointLimitTorques()
{
    for (int i = 0; i < 6; i++)
    {
        if (m_jointAnglesEncoderSI[i] >= m_jointLimitUpper[i])
        {
            m_jointLimitTorques[i] = -m_jointLimitStiffness[i] * (m_jointAnglesEncoderSI[i] - m_jointLimitUpper[i]) - m_jointLimitDamping[i] * m_jointVelocityEncoderSI[i];
        }
        else if (m_jointAnglesEncoderSI[i] <= m_jointLimitLower[i])
        {
            m_jointLimitTorques[i] = -m_jointLimitStiffness[i] * (m_jointAnglesEncoderSI[i] - m_jointLimitLower[i]) - m_jointLimitDamping[i] * m_jointVelocityEncoderSI[i];
        }
    }
}

//===================================================================
void CDaVinciRobotMTM::SetForceTorqueCartesian(cVector3d a_force, cVector3d a_torque)
{
    double t_manipulatorJacobian[6][7];
    vctDynamicVector<double> t_vectorJointTorque;
    vctDynamicVector<double> t_vectorMotorTorque;
    vctDynamicVector<double> t_vectorForceTorque;
    double m_ampMotorTorque[7];
    AmpIO_UInt32 m_rawMotorTorque[8];

    // Calculate the end effector manipulator Jacobian using the spatial jacobian
    // and the current end effector position
    for (int j = 0; j < 7; j++)
    {
        t_manipulatorJacobian[0][j] = m_robot.Js[j][0] + m_robot.Js[j][4] * m_eeCartesianPosition[2] - m_robot.Js[j][5] * m_eeCartesianPosition[1];
        t_manipulatorJacobian[1][j] = m_robot.Js[j][1] + m_robot.Js[j][5] * m_eeCartesianPosition[0] - m_robot.Js[j][3] * m_eeCartesianPosition[2];
        t_manipulatorJacobian[2][j] = m_robot.Js[j][2] + m_robot.Js[j][3] * m_eeCartesianPosition[1] - m_robot.Js[j][4] * m_eeCartesianPosition[0];
        t_manipulatorJacobian[3][j] = m_robot.Js[j][3];
        t_manipulatorJacobian[4][j] = m_robot.Js[j][4];
        t_manipulatorJacobian[5][j] = m_robot.Js[j][5];
    }

    t_vectorJointTorque.SetSize(7);
    t_vectorJointTorque.Zeros();
//===================================================================
    t_vectorMotorTorque.SetSize(7);
    t_vectorMotorTorque.Zeros();

    t_vectorForceTorque.SetSize(6);
    t_vectorForceTorque.Zeros();

    // Determine the motor torques to exert force (No torques)
    t_vectorForceTorque[0] = a_force[0] + m_gravityForce[0];
    t_vectorForceTorque[1] = a_force[1] + m_gravityForce[1];
    t_vectorForceTorque[2] = a_force[2] + m_gravityForce[2];
    t_vectorForceTorque[3] = a_torque[0];
    t_vectorForceTorque[4] = a_torque[1];
    t_vectorForceTorque[5] = a_torque[2];

    // Determine the joint torques
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            t_vectorJointTorque[i] = t_vectorJointTorque[i] +
                    t_manipulatorJacobian[j][i] * t_vectorForceTorque[j];
        }
    }

    // Add in the joint limit torques
    if (m_flagJointLimitEnable == true)
    {
        CalculateJointLimitTorques();
        printf("Joint limit torques: %6.4f\n", m_jointLimitTorques[0]);
    }

    // Determine the actuator torques
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            t_vectorMotorTorque[i] = t_vectorMotorTorque[i] +
                    m_ioInfo.m_couplingJointToActuatorTorque[i][j] * t_vectorJointTorque[j];
        }
    }

    // Convert the torque value to motor current
    for (int i = 0; i < 7; i++)
    {
        m_ampMotorTorque[i] = t_vectorMotorTorque[i]*m_ioInfo.m_driveTorqueNmToAmpsScale[i];

        // Check for current overflow
        if (m_ampMotorTorque[i] >= m_ioInfo.m_driveMaxAmpsValue[i])
        {
            m_ampMotorTorque[i] = m_ioInfo.m_driveMaxAmpsValue[i];
        }
        else if (m_ampMotorTorque[i] <= -m_ioInfo.m_driveMaxAmpsValue[i])
        {
            m_ampMotorTorque[i] = -m_ioInfo.m_driveMaxAmpsValue[i];
        }

        // Calculate the motor torques in bits
        m_rawMotorTorque[i] = (m_ampMotorTorque[i] * m_ioInfo.m_driveAmpsToBitsScale[i] + m_ioInfo.m_driveAmpsToBitsOffset[i]);
    }

    m_rawMotorTorque[7] = 0;

    // Send the data to the amplifier board
    p_amplifierBoard1->SetMotorCurrent(0, m_rawMotorTorque[0]);
    p_amplifierBoard1->SetMotorCurrent(1, m_rawMotorTorque[1]);
    p_amplifierBoard1->SetMotorCurrent(2, m_rawMotorTorque[2]);
    p_amplifierBoard1->SetMotorCurrent(3, m_rawMotorTorque[3]);

    p_amplifierBoard2->SetMotorCurrent(0, m_rawMotorTorque[4]);
    p_amplifierBoard2->SetMotorCurrent(1, m_rawMotorTorque[5]);
    p_amplifierBoard2->SetMotorCurrent(2, m_rawMotorTorque[6]);
    p_amplifierBoard2->SetMotorCurrent(3, m_rawMotorTorque[7]);
}

//===================================================================
void CDaVinciRobotMTM::SetForceTorqueJoint(double *a_torque)
{
    vctDynamicVector<double> t_vectorJointTorque;
    vctDynamicVector<double> t_vectorMotorTorque;
    double m_ampMotorTorque[7];
    AmpIO_UInt32 m_rawMotorTorque[8];

    t_vectorJointTorque.SetSize(7);
    t_vectorJointTorque.Zeros();

    t_vectorMotorTorque.SetSize(7);
    t_vectorMotorTorque.Zeros();

    // Determine the joint torques
    for (int i = 0; i < 7; i++)
    {
        t_vectorJointTorque[i] = a_torque[i];
    }

    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            t_vectorMotorTorque[i] = t_vectorMotorTorque[i] +
                                     m_ioInfo.m_couplingJointToActuatorTorque[i][j] * t_vectorJointTorque[j];
        }
    }

    // Convert the torque value to motor current
    for (int i = 0; i < 7; i++)
    {
        m_ampMotorTorque[i] = t_vectorMotorTorque[i]*m_ioInfo.m_driveTorqueNmToAmpsScale[i];

        // Check for current overflow
        if (m_ampMotorTorque[i] >= m_ioInfo.m_driveMaxAmpsValue[i])
        {
            m_ampMotorTorque[i] = m_ioInfo.m_driveMaxAmpsValue[i];
        }
        else if (m_ampMotorTorque[i] <= -m_ioInfo.m_driveMaxAmpsValue[i])
        {
            m_ampMotorTorque[i] = -m_ioInfo.m_driveMaxAmpsValue[i];
        }

        // Calculate the motor torques in bits
        m_rawMotorTorque[i] = (m_ampMotorTorque[i] * m_ioInfo.m_driveAmpsToBitsScale[i] + m_ioInfo.m_driveAmpsToBitsOffset[i]);
//        printf("%d\t",m_rawMotorTorque[i]);
    }
//    printf("\n");

    // Send the data to the amplifier board
    p_amplifierBoard1->SetMotorCurrent(0, m_rawMotorTorque[0]);
    p_amplifierBoard1->SetMotorCurrent(1, m_rawMotorTorque[1]);
    p_amplifierBoard1->SetMotorCurrent(2, m_rawMotorTorque[2]);
    p_amplifierBoard1->SetMotorCurrent(3, m_rawMotorTorque[3]);

    p_amplifierBoard2->SetMotorCurrent(0, m_rawMotorTorque[4]);
    p_amplifierBoard2->SetMotorCurrent(1, m_rawMotorTorque[5]);
    p_amplifierBoard2->SetMotorCurrent(2, m_rawMotorTorque[6]);
    p_amplifierBoard2->SetMotorCurrent(3, m_rawMotorTorque[7]);
}

// ===============================================================================
double CDaVinciRobotMTM::GetGripperAngle()
{
    return m_jointAnglesPotentiometerSI[7];
}

//===================================================================
bool CDaVinciRobotMTM::GetPedalClutch()
{
    if (m_digitalInput[0] == true)
    {
        return false;
    }
    else
    {
        return true;
    }
}

//===================================================================
void CDaVinciRobotMTM::SetJointLimitsUpper(double *a_qlimits)
{
    for (int i = 0; i < 6; i++)
    {
        m_jointLimitUpper[i] = a_qlimits[i];
    }
}

//===================================================================
void CDaVinciRobotMTM::SetJointLimitsLower(double *a_qlimits)
{
    for (int i = 0; i < 6; i++)
    {
        m_jointLimitLower[i] = a_qlimits[i];
    }
}

//===================================================================
void CDaVinciRobotMTM::SetJointLimitsStiffness(double *a_qlimitsK)
{
    for (int i = 0; i < 7; i++)
    {
        m_jointLimitStiffness[i] = a_qlimitsK[i];
    }
}

//===================================================================
void CDaVinciRobotMTM::SetJointLimitsDamping(double *a_qlimitsK)
{
    for (int i = 0; i < 7; i++)
    {
        m_jointLimitDamping[i] = a_qlimitsK[i];
    }
}

//===================================================================
void CDaVinciRobotMTM::GetFootPedalInputs(bool *footPedalInputs)
{
    for (int i = 0; i < 6; i++) {
        if (m_digitalInput[i]){
            footPedalInputs[i] = false;
        } else {
            footPedalInputs[i] = true;
        }
    }
}

//===================================================================
void CDaVinciRobotMTM::FetchDigitalInput()
{
    AmpIO_UInt32 t_digitalInputRead = p_amplifierBoard2->GetDigitalInput();

    for (int i = 0; i < 12; i++)
    {
        if ((t_digitalInputRead &  (0x01 << i)) == 0)
        {
            m_digitalInput[i] = false;
        }
        else
        {
            m_digitalInput[i] = true;
        }
    }
}
