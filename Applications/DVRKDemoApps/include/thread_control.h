#ifndef THREAD_CONTROL_H
#define THREAD_CONTROL_H
//============================================================================
#include <QThread>
#include "shared_data.h"
//============================================================================
class Thread_Control : public QThread
{
    Q_OBJECT
public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    // Constructor
    explicit Thread_Control(QObject *parent = 0);

    // Destructor
    ~Thread_Control();

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
    // Initialization function
    void Init();

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
    sd_SharedData* p_sharedData;

    // A clock to ensure a certain control rate
    cPrecisionClock m_rateClock;

    // a clock to estimate the haptic simulation loop update rate
    cPrecisionClock m_rateDisplayClock;
    int m_rateDisplayCounter;

protected:
    void run();

private:
    void GetRobotInfo();
    void ProcessJointContrlCommands();
    void ProcessCartesianControlCommands();
    void SetRobotCommands();
    cVector3d CalculateDelPhi(cMatrix3d a_R1, cMatrix3d a_R2);


    double t_mtmlJointError[8];
    double t_psm1JointError[8];

    double t_mtmrJointError[8];
    double t_psm2JointError[8];

    cMatrix3d t_mtmlDesiredR;
    cMatrix3d t_mtmlCurrentR;
    cVector3d t_mtmlDelPhi;

    cMatrix3d t_psm1DesiredR;
    cMatrix3d t_psm1CurrentR;
    cVector3d t_psm1DelPhi;

    cMatrix3d t_mtmrDesiredR;
    cMatrix3d t_mtmrCurrentR;
    cVector3d t_mtmrDelPhi;

    cMatrix3d t_psm2DesiredR;
    cMatrix3d t_psm2CurrentR;
    cVector3d t_psm2DelPhi;
    
signals:
    
public slots:
    
};
//============================================================================
#endif // THREAD_CONTROL_H
