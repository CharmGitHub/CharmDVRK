#include <QtGui/QApplication>
#include "form_mainwindow.h"

#include "shared_data.h"
#include "thread_control.h"
#include "thread_statemachine.h"

//============================================================================
sd_SharedData g_sharedData;
Thread_Control g_threadControl;
Thread_StateMachine g_threadStateMachine;

//============================================================================
int main(int argc, char *argv[])
{    
    // Initialize firewire port to port 0
    g_sharedData.p_port = new FirewirePort(0);

    // ---------------------------- MTML - PSM1 Initialization --------------------
    // Create Left MTM object
    g_sharedData.p_daVinciMTML = new CDaVinciRobotMTM(g_sharedData.p_port, 0, 1);
    sleep(1);

    // Create PSM 1 object
    g_sharedData.p_daVinciPSM1 = new CDaVinciRobotPSM(g_sharedData.p_port, 4, 5);
    sleep(1);

    g_sharedData.p_daVinciMTML->SetupRobotParam("robotparamfiles/dvmtm.rob", "robotparamfiles/sawRobotIO1394-MTML-29932.xml");
    g_sharedData.p_daVinciPSM1->SetupRobotParam("robotparamfiles/dvpsm.rob", "robotparamfiles/sawRobotIO1394-PSM1-19798.xml");
    g_sharedData.p_port->ReadAllBoards();
    g_sharedData.p_daVinciMTML->Init();
    g_sharedData.p_daVinciMTML->SetGravityForceValue(cVector3d(0,0,0));
    g_sharedData.p_daVinciPSM1->Init();
    g_sharedData.p_daVinciPSM1->SetGravityForceValue(cVector3d(0,0,0));

    // Set the joint limits for the PSM1 robot
    double t_psm1JointUpperLimits[8];
    t_psm1JointUpperLimits[0] = cDegToRad(30);
    t_psm1JointUpperLimits[1] = cDegToRad(30);
    t_psm1JointUpperLimits[2] = 0;
    t_psm1JointUpperLimits[3] = cDegToRad(180);
    t_psm1JointUpperLimits[4] = cDegToRad(30);
    t_psm1JointUpperLimits[5] = cDegToRad(45);

    double t_psm1JointLowerLimits[8];
    t_psm1JointLowerLimits[0] = cDegToRad(-30);
    t_psm1JointLowerLimits[1] = cDegToRad(-30);
    t_psm1JointLowerLimits[2] = 0;
    t_psm1JointLowerLimits[3] = cDegToRad(-180);
    t_psm1JointLowerLimits[4] = cDegToRad(-30);
    t_psm1JointLowerLimits[5] = cDegToRad(-45);

    double t_psm1JointLimitStiffness[8];
    t_psm1JointLimitStiffness[0] = 0;   //20
    t_psm1JointLimitStiffness[1] = 0;   //20
    t_psm1JointLimitStiffness[2] = 0;
    t_psm1JointLimitStiffness[3] = 0.0; //0.5
    t_psm1JointLimitStiffness[4] = 0.0; //0.5
    t_psm1JointLimitStiffness[5] = 0;

    double t_psm1JointLimitDamping[8];
    t_psm1JointLimitDamping[0] = 0;
    t_psm1JointLimitDamping[1] = 0;
    t_psm1JointLimitDamping[2] = 0;
    t_psm1JointLimitDamping[3] = 0.01;
    t_psm1JointLimitDamping[4] = 0;
    t_psm1JointLimitDamping[5] = 0;

    g_sharedData.p_daVinciPSM1->SetJointLimitsStiffness(t_psm1JointLimitStiffness);
    g_sharedData.p_daVinciPSM1->SetJointLimitsDamping(t_psm1JointLimitDamping);
    g_sharedData.p_daVinciPSM1->SetJointLimitsLower(t_psm1JointLowerLimits);
    g_sharedData.p_daVinciPSM1->SetJointLimitsUpper(t_psm1JointUpperLimits);

    // ---------------------------- MTMR - PSM2 Initialization --------------------
    // Create right MTM object
    g_sharedData.p_daVinciMTMR = new CDaVinciRobotMTM(g_sharedData.p_port, 2, 3);
    sleep(1);

    // Create PSM 2 object
    g_sharedData.p_daVinciPSM2 = new CDaVinciRobotPSM(g_sharedData.p_port, 6, 7);
    sleep(1);

    g_sharedData.p_daVinciMTMR->SetupRobotParam("robotparamfiles/dvmtm.rob", "robotparamfiles/sawRobotIO1394-MTMR-25348.xml");
    g_sharedData.p_daVinciPSM2->SetupRobotParam("robotparamfiles/dvpsm.rob", "robotparamfiles/sawRobotIO1394-PSM2-30831.xml");
    g_sharedData.p_port->ReadAllBoards();
    g_sharedData.p_daVinciMTMR->Init();
    g_sharedData.p_daVinciMTMR->SetGravityForceValue(cVector3d(0,0,0));
    g_sharedData.p_daVinciPSM2->Init();
    g_sharedData.p_daVinciPSM2->SetGravityForceValue(cVector3d(0,0,0));

    // Set the joint limits for the PSM1 robot
    double t_psm2JointUpperLimits[8];
    t_psm2JointUpperLimits[0] = cDegToRad(30);
    t_psm2JointUpperLimits[1] = cDegToRad(30);
    t_psm2JointUpperLimits[2] = 0;
    t_psm2JointUpperLimits[3] = cDegToRad(180);
    t_psm2JointUpperLimits[4] = cDegToRad(30);
    t_psm2JointUpperLimits[5] = cDegToRad(45);

    double t_psm2JointLowerLimits[8];
    t_psm2JointLowerLimits[0] = cDegToRad(-30);
    t_psm2JointLowerLimits[1] = cDegToRad(-30);
    t_psm2JointLowerLimits[2] = 0;
    t_psm2JointLowerLimits[3] = cDegToRad(-180);
    t_psm2JointLowerLimits[4] = cDegToRad(-30);
    t_psm2JointLowerLimits[5] = cDegToRad(-45);

    double t_psm2JointLimitStiffness[8];
    t_psm2JointLimitStiffness[0] = 0;   //20
    t_psm2JointLimitStiffness[1] = 0;   //20
    t_psm2JointLimitStiffness[2] = 0;
    t_psm2JointLimitStiffness[3] = 0.0; //0.5
    t_psm2JointLimitStiffness[4] = 0.0; //0.5
    t_psm2JointLimitStiffness[5] = 0;

    double t_psm2JointLimitDamping[8];
    t_psm2JointLimitDamping[0] = 0;
    t_psm2JointLimitDamping[1] = 0;
    t_psm2JointLimitDamping[2] = 0;
    t_psm2JointLimitDamping[3] = 0.01;
    t_psm2JointLimitDamping[4] = 0;
    t_psm2JointLimitDamping[5] = 0;

    g_sharedData.p_daVinciPSM2->SetJointLimitsStiffness(t_psm2JointLimitStiffness);
    g_sharedData.p_daVinciPSM2->SetJointLimitsDamping(t_psm2JointLimitDamping);
    g_sharedData.p_daVinciPSM2->SetJointLimitsLower(t_psm2JointLowerLimits);
    g_sharedData.p_daVinciPSM2->SetJointLimitsUpper(t_psm2JointUpperLimits);


    g_sharedData.m_slaveToMasterRotation[0][0] = -1;
    g_sharedData.m_slaveToMasterRotation[0][1] = 0;
    g_sharedData.m_slaveToMasterRotation[0][2] = 0;
    g_sharedData.m_slaveToMasterRotation[1][0] = 0;
    g_sharedData.m_slaveToMasterRotation[1][1] = -1;
    g_sharedData.m_slaveToMasterRotation[1][2] = 0;
    g_sharedData.m_slaveToMasterRotation[2][0] = 0;
    g_sharedData.m_slaveToMasterRotation[2][1] = 0;
    g_sharedData.m_slaveToMasterRotation[2][2] = 1;

    g_sharedData.m_masterToSlaveRotation = g_sharedData.m_slaveToMasterRotation.Transpose();

    // --------------------- Set flags in the shared data ---------------------------
    // Flags for MTML control type
    g_sharedData.m_flagMTMLControlType = NoCtrlWithGC;
    g_sharedData.m_flagMTMRControlType = NoCtrlWithGC;
    g_sharedData.m_flagPSM1ControlType = NoCtrlWithGC;
    g_sharedData.m_flagPSM2ControlType = NoCtrlWithGC;

    // Flags for power on and relay on
    g_sharedData.m_flagEnableSafetyRelay = false;
    g_sharedData.m_flagEnablePower = false;

    // Flags for state machine
    g_sharedData.m_flagHomeRobot = false;
    g_sharedData.m_flagStartTeleoperation = false;

    // Flags for threads
    g_sharedData.m_flagControlThreadEnd = false;
    g_sharedData.m_flagStateMachineThreadEnd = false;
    g_sharedData.m_flagDisplayThreadEnded = false;

    // Flags for head presence
    g_sharedData.m_flagHeadPresent = false;

    QApplication a(argc, argv);
    Form_MainWindow g_threadDisplay;

    // Setup the pointers for the control thread
    g_threadControl.p_sharedData = &g_sharedData;
    g_threadControl.Init();

    // Setup the pointers for the state thread
    g_threadStateMachine.p_sharedData = &g_sharedData;
    g_threadStateMachine.Init();

    // Setup the pointers for the display thread
    g_threadDisplay.p_sharedData = &g_sharedData;
    g_threadDisplay.Init();

    g_threadDisplay.show();
    
    return a.exec();
}

//============================================================================
