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

    g_sharedData.p_daVinciMTML->SetupRobotParam("./dvmtm.rob", "./sawRobotIO1394-MTML-29932.xml");
    g_sharedData.p_daVinciPSM1->SetupRobotParam("./dvpsm.rob", "./sawRobotIO1394-PSM1-19798.xml");
    g_sharedData.p_port->ReadAllBoards();
    g_sharedData.p_daVinciMTML->Init();
    g_sharedData.p_daVinciMTML->SetGravityForceValue(cVector3d(0,0,0));
    g_sharedData.p_daVinciPSM1->Init();
    g_sharedData.p_daVinciPSM1->SetGravityForceValue(cVector3d(0,0,0));

    // ---------------------------- MTMR - PSM2 Initialization --------------------
    // Create right MTM object
    g_sharedData.p_daVinciMTMR = new CDaVinciRobotMTM(g_sharedData.p_port, 2, 3);
    sleep(1);

    // Create PSM 2 object
    g_sharedData.p_daVinciPSM2 = new CDaVinciRobotPSM(g_sharedData.p_port, 6, 7);
    sleep(1);

    g_sharedData.p_daVinciMTMR->SetupRobotParam("./dvmtm.rob", "./sawRobotIO1394-MTMR-25348.xml");
    g_sharedData.p_daVinciPSM2->SetupRobotParam("./dvpsm.rob", "./sawRobotIO1394-PSM2-30831.xml");
    g_sharedData.p_port->ReadAllBoards();
    g_sharedData.p_daVinciMTMR->Init();
    g_sharedData.p_daVinciMTMR->SetGravityForceValue(cVector3d(0,0,0));
    g_sharedData.p_daVinciPSM2->Init();
    g_sharedData.p_daVinciPSM2->SetGravityForceValue(cVector3d(0,0,0));



    QApplication a(argc, argv);
    Form_MainWindow g_displayThread;
    g_displayThread.show();
    
    return a.exec();
}

//============================================================================
