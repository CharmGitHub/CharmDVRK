#include "thread_control.h"
//============================================================================
Thread_Control::Thread_Control(QObject *parent) :
    QThread(parent)
{
}

//============================================================================
Thread_Control::~Thread_Control()
{

}

//============================================================================
void Thread_Control::Init()
{
    // Set the rate display clock to 1 second
    m_rateDisplayClock.setTimeoutPeriodSeconds(1.0);
    m_rateDisplayClock.start(true);

    // Set the rate clock to 1 kHz
    m_rateClock.reset();
    m_rateClock.setTimeoutPeriodSeconds(0.001);
    m_rateClock.start(true);
}

//============================================================================
void Thread_Control::run()
{
    qDebug("Starting Control Thread...\n");

    while (p_sharedData->m_flagDisplayThreadEnded == false)
    {
        if (m_rateClock.timeoutOccurred())
        {
            m_rateClock.stop();

            // Perform the position and velocity acquisition of the master and slave robot
            p_sharedData->p_port->ReadAllBoards();

            this->GetRobotInfo();
            this->ProcessJointContrlCommands();
            this->ProcessCartesianControlCommands();
            this->SetRobotCommands();

            // ------ Send command to board ------
            p_sharedData->p_port->WriteAllBoards();

            // estimate the refresh rate
            m_rateDisplayCounter++;
            if (m_rateDisplayClock.timeoutOccurred())
            {
                m_rateDisplayClock.stop();
                p_sharedData->m_controlRate = m_rateDisplayCounter;
                m_rateDisplayCounter = 0;
                m_rateDisplayClock.start(true);
            }

            m_rateClock.start(true);
        }
    }

    // Set the thread ended flag to true
    p_sharedData->m_flagControlThreadEnd = true;

    // Print out the message
    qDebug("Control Thread ended!\n");
}

//============================================================================
void Thread_Control::GetRobotInfo()
{
    // ----------------------------------- MTM Update ---------------------------------------
    // Update the position and velocity into the shared data of Left MTM
    p_sharedData->m_daVinciMTMLPosition = p_sharedData->p_daVinciMTML->GetEECartesianPosition() - p_sharedData->m_mtmlPositionOffset;
    p_sharedData->m_daVinciMTMLVelocity = p_sharedData->p_daVinciMTML->GetEECartesianLinearVelocity();
    p_sharedData->m_daVinciMTMLRotation = p_sharedData->p_daVinciMTML->GetEEOrientation() * p_sharedData->m_mtmlRotationOffset.Transpose();
    p_sharedData->m_daVinciMTMLVelocityAngular = p_sharedData->p_daVinciMTML->GetEECartesianAngularVelocity();
    p_sharedData->m_daVinciMTMLGripAngle = p_sharedData->p_daVinciMTML->GetGripperAngle();

    p_sharedData->p_daVinciMTML->GetJointAnglesEncoder(p_sharedData->m_daVinciMTMLJointAngles);
    p_sharedData->p_daVinciMTML->GetJointVelocityEncoder(p_sharedData->m_daVinciMTMLJointVeloicities);

    // ----------------------------------- PSM Update ---------------------------------------
    // Update the position and velocity into the shared data of PSM 1
    p_sharedData->m_daVinciPSM1Position = p_sharedData->m_slaveToMasterRotation * (p_sharedData->p_daVinciPSM1->GetEECartesianPosition()) - p_sharedData->m_psm1PositionOffset;
    p_sharedData->m_daVinciPSM1Velocity = p_sharedData->m_slaveToMasterRotation * (p_sharedData->p_daVinciPSM1->GetEECartesianLinearVelocity());
    p_sharedData->m_daVinciPSM1Rotation = p_sharedData->m_slaveToMasterRotation * (p_sharedData->p_daVinciPSM1->GetEEOrientation()) * p_sharedData->m_psm1RotationOffset.Transpose();
    p_sharedData->m_daVinciPSM1VelocityAngular = p_sharedData->m_slaveToMasterRotation * (p_sharedData->p_daVinciPSM1->GetEECartesianAngularVelocity());
    p_sharedData->m_daVinciPSM1GripAngle = p_sharedData->p_daVinciPSM1->GetGripperAngle();
    p_sharedData->m_daVinciPSM1GripVelocity = p_sharedData->p_daVinciPSM1->GetGripperVelocity();
    p_sharedData->p_daVinciPSM1->GetJointAnglesEncoder(p_sharedData->m_daVinciPSM1JointAngles);
    p_sharedData->p_daVinciPSM1->GetJointVelocityEncoder(p_sharedData->m_daVinciPSM1JointVeloicities);
}

//============================================================================
void Thread_Control::ProcessJointContrlCommands()
{
    // ----------- Calculate torques based on joint errors -----------
    for (int i = 0; i < 7; i++)
    {
        // Calculate the error
        t_mtmlJointError[i] = p_sharedData->m_daVinciMTMLJointAnglesDesired[i] - p_sharedData->m_daVinciMTMLJointAngles[i];
        t_psm1JointError[i] = p_sharedData->m_daVinciPSM1JointAnglesDesired[i] - p_sharedData->m_daVinciPSM1JointAngles[i];
        t_mtmrJointError[i] = p_sharedData->m_daVinciMTMRJointAnglesDesired[i] - p_sharedData->m_daVinciMTMRJointAngles[i];
        t_psm2JointError[i] = p_sharedData->m_daVinciPSM2JointAnglesDesired[i] - p_sharedData->m_daVinciPSM2JointAngles[i];

        // Calculate the torque
        p_sharedData->m_daVinciMTMLJointForces[i] = p_sharedData->m_mtmJointKp[i] * t_mtmlJointError[i] - p_sharedData->m_mtmJointKd[i] * p_sharedData->m_daVinciMTMLJointVeloicities[i];
        p_sharedData->m_daVinciPSM1JointForces[i] = p_sharedData->m_psmJointKp[i] * t_psm1JointError[i] - p_sharedData->m_psmJointKd[i] * p_sharedData->m_daVinciPSM1JointVeloicities[i];
        p_sharedData->m_daVinciMTMRJointForces[i] = p_sharedData->m_mtmJointKp[i] * t_mtmrJointError[i] - p_sharedData->m_mtmJointKd[i] * p_sharedData->m_daVinciMTMRJointVeloicities[i];
        p_sharedData->m_daVinciPSM2JointForces[i] = p_sharedData->m_psmJointKp[i] * t_psm2JointError[i] - p_sharedData->m_psmJointKd[i] * p_sharedData->m_daVinciPSM2JointVeloicities[i];
    }
}

//============================================================================
void Thread_Control::ProcessCartesianControlCommands()
{
    // ----------- Calculate torques based on cartesian errors -----------
    for (int i = 0; i < 3; i++)
    {
        p_sharedData->m_daVinciMTMLForce[i] = p_sharedData->m_mtmCartesianKpForce[i] * (p_sharedData->m_daVinciPSM1Position[i] - p_sharedData->m_daVinciMTMLPosition[i] * 0.30) -
                                              p_sharedData->m_mtmCartesianKdForce[i] * (p_sharedData->m_daVinciMTMLVelocity[i]);

        p_sharedData->m_daVinciMTMRForce[i] = p_sharedData->m_mtmCartesianKpForce[i] * (p_sharedData->m_daVinciPSM2Position[i] - p_sharedData->m_daVinciMTMRPosition[i] * 0.30) -
                                              p_sharedData->m_mtmCartesianKdForce[i] * (p_sharedData->m_daVinciMTMRVelocity[i]);

        p_sharedData->m_daVinciPSM1Force[i] = p_sharedData->m_psmCartesianKpForce[i] * (p_sharedData->m_daVinciMTMLPosition[i] * 0.30 - p_sharedData->m_daVinciPSM1Position[i]) -
                                              p_sharedData->m_psmCartesianKdForce[i] * (p_sharedData->m_daVinciPSM1Velocity[i]);

        p_sharedData->m_daVinciPSM2Force[i] = p_sharedData->m_psmCartesianKpForce[i] * (p_sharedData->m_daVinciMTMRPosition[i] * 0.30 - p_sharedData->m_daVinciPSM2Position[i]) -
                                              p_sharedData->m_psmCartesianKdForce[i] * (p_sharedData->m_daVinciPSM2Velocity[i]);
    }

    t_mtmlDesiredR = p_sharedData->m_daVinciPSM1Rotation;
    t_mtmlCurrentR = p_sharedData->m_daVinciMTMLRotation;

    t_mtmlDelPhi = CalculateDelPhi(t_mtmlCurrentR, t_mtmlDesiredR);

    t_mtmrDesiredR = p_sharedData->m_daVinciPSM2Rotation;
    t_mtmrCurrentR = p_sharedData->m_daVinciMTMRRotation;
    t_mtmrDelPhi = CalculateDelPhi(t_mtmrCurrentR, t_mtmrDesiredR);

    t_psm1DesiredR = p_sharedData->m_daVinciMTMLRotation;
    t_psm1CurrentR = p_sharedData->m_daVinciPSM1Rotation;
    t_psm1DelPhi = CalculateDelPhi(t_psm1CurrentR, t_psm1DesiredR);

    t_psm2DesiredR = p_sharedData->m_daVinciMTMRRotation;
    t_psm2CurrentR = p_sharedData->m_daVinciPSM2Rotation;
    t_psm2DelPhi = CalculateDelPhi(t_psm2CurrentR, t_psm2DesiredR);

    for (int i = 0; i < 3; i++)
    {
        p_sharedData->m_daVinciMTMLTorque[i] = -p_sharedData->m_mtmCartesianKpTorque[i] * t_mtmlDelPhi[i]
                                               -p_sharedData->m_mtmCartesianKdTorque[i] * p_sharedData->m_daVinciMTMLVelocityAngular[i];

        p_sharedData->m_daVinciPSM1Torque[i] = -p_sharedData->m_psmCartesianKpTorque[i] * t_psm1DelPhi[i]
                                               -p_sharedData->m_psmCartesianKdTorque[i] * p_sharedData->m_daVinciPSM1VelocityAngular[i];

        p_sharedData->m_daVinciMTMRTorque[i] = -p_sharedData->m_mtmCartesianKpTorque[i] * t_mtmrDelPhi[i]
                                               -p_sharedData->m_mtmCartesianKdTorque[i] * p_sharedData->m_daVinciMTMRVelocityAngular[i];

        p_sharedData->m_daVinciPSM2Torque[i] = -p_sharedData->m_psmCartesianKpTorque[i] * t_psm2DelPhi[i]
                                               -p_sharedData->m_psmCartesianKdTorque[i] * p_sharedData->m_daVinciPSM2VelocityAngular[i];
    }

    p_sharedData->m_daVinciPSM1GripperForce = p_sharedData->m_gripperKp * (p_sharedData->m_daVinciMTMLGripAngle - p_sharedData->m_daVinciPSM1GripAngle)
                                            - p_sharedData->m_gripperKd * p_sharedData->m_daVinciPSM1GripVelocity;

    // Transform the forces and torques back to the slave frame
    p_sharedData->m_daVinciPSM1Force = p_sharedData->m_masterToSlaveRotation * p_sharedData->m_daVinciPSM1Force;
    p_sharedData->m_daVinciPSM1Torque = p_sharedData->m_masterToSlaveRotation * p_sharedData->m_daVinciPSM1Torque;

    p_sharedData->m_daVinciPSM2GripperForce = p_sharedData->m_gripperKp * (p_sharedData->m_daVinciMTMRGripAngle - p_sharedData->m_daVinciPSM2GripAngle)
                                            - p_sharedData->m_gripperKd * p_sharedData->m_daVinciPSM2GripVelocity;

    // Transform the forces and torques back to the slave frame
    p_sharedData->m_daVinciPSM2Force = p_sharedData->m_masterToSlaveRotation * p_sharedData->m_daVinciPSM2Force;
    p_sharedData->m_daVinciPSM2Torque = p_sharedData->m_masterToSlaveRotation * p_sharedData->m_daVinciPSM2Torque;
}

//============================================================================
void Thread_Control::SetRobotCommands()
{
    // ----- MTML control ------
    if (p_sharedData->m_flagMTMLControlType == JointCtrlWithGC)
    {
        // Joint control --- Set the joint torques to MTML
        p_sharedData->p_daVinciMTML->SetForceTorqueJoint(p_sharedData->m_daVinciMTMLJointForces);
    }
    else if (p_sharedData->m_flagMTMLControlType == CartesianCtrlWithGC)
    {
        // Cartesian control --- Set the forces and torques to MTML
        p_sharedData->p_daVinciMTML->SetForceTorqueCartesian(p_sharedData->m_daVinciMTMLForce, p_sharedData->m_daVinciMTMLTorque);
    }
    else
    {
        p_sharedData->m_daVinciMTMLForce.Zeros();
        p_sharedData->m_daVinciMTMLTorque.Zeros();

        p_sharedData->p_daVinciMTML->SetForceTorqueCartesian(p_sharedData->m_daVinciMTMLForce, p_sharedData->m_daVinciMTMLTorque);
    }

}

//============================================================================
cVector3d Thread_Control::CalculateDelPhi(cMatrix3d a_R1, cMatrix3d a_R2)
{
    cVector3d t_returnVal;

    t_returnVal[0] = a_R1[1][0] * a_R2[2][0] - a_R1[2][0] * a_R2[1][0] +
                     a_R1[1][1] * a_R2[2][1] - a_R1[2][1] * a_R2[1][1] +
                     a_R1[1][2] * a_R2[2][2] - a_R1[2][2] * a_R2[1][2] ;

    t_returnVal[1] = a_R1[2][0] * a_R2[0][0] - a_R1[0][0] * a_R2[2][0] +
                     a_R1[2][1] * a_R2[0][1] - a_R1[0][1] * a_R2[2][1] +
                     a_R1[2][2] * a_R2[0][2] - a_R1[0][2] * a_R2[2][2] ;

    t_returnVal[2] = a_R1[0][0] * a_R2[1][0] - a_R1[1][0] * a_R2[0][0] +
                     a_R1[0][1] * a_R2[1][1] - a_R1[1][1] * a_R2[0][1] +
                     a_R1[0][2] * a_R2[1][2] - a_R1[1][2] * a_R2[0][2] ;

    t_returnVal[0] = t_returnVal[0] * 0.5 * -1;
    t_returnVal[1] = t_returnVal[1] * 0.5 * -1;
    t_returnVal[2] = t_returnVal[2] * 0.5 * -1;

    return t_returnVal;
}
