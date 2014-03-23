#ifndef THREAD_STATEMACHINE_H
#define THREAD_STATEMACHINE_H
//============================================================================
#include <QThread>
//============================================================================
class Thread_StateMachine : public QThread
{
    Q_OBJECT
public:
    explicit Thread_StateMachine(QObject *parent = 0);
    
signals:
    
public slots:
    
};
//============================================================================
#endif // THREAD_STATEMACHINE_H
