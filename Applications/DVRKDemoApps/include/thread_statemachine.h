#ifndef THREAD_STATEMACHINE_H
#define THREAD_STATEMACHINE_H
//============================================================================
#include <QThread>
#include "shared_data.h"
//============================================================================
class Thread_StateMachine : public QThread
{
    Q_OBJECT
public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    // Constructor
    explicit Thread_StateMachine(QObject *parent = 0);

    // Destructor
    ~Thread_StateMachine();

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
    // Initialization function
    void Init();

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
    sd_SharedData* p_sharedData;
    
protected:
    void run();

signals:
    
public slots:
    
};
//============================================================================
#endif // THREAD_STATEMACHINE_H
