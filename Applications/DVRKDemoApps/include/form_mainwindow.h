#ifndef FORM_MAINWINDOW_H
#define FORM_MAINWINDOW_H
//============================================================================
#include <QMainWindow>
#include "shared_data.h"
//============================================================================
namespace Ui {
class Form_MainWindow;
}
//============================================================================
class Form_MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    // Constructor
    explicit Form_MainWindow(QWidget *parent = 0);

    // Destructor
    ~Form_MainWindow();

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
    // Initialization function
    void Init();

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
    sd_SharedData* p_sharedData;
    
private:
    Ui::Form_MainWindow *ui;
};
//============================================================================
#endif // FORM_MAINWINDOW_H
