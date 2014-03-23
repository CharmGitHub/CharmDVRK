#ifndef THREAD_CONTROL_H
#define THREAD_CONTROL_H
//============================================================================
#include <QThread>
//============================================================================
class Thread_Control : public QThread
{
    Q_OBJECT
public:
    explicit Thread_Control(QObject *parent = 0);
    
signals:
    
public slots:
    
};
//============================================================================
#endif // THREAD_CONTROL_H
