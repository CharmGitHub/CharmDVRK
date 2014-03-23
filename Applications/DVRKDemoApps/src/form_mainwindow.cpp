#include "form_mainwindow.h"
#include "ui_form_mainwindow.h"
//===================================================================
Form_MainWindow::Form_MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Form_MainWindow)
{
    ui->setupUi(this);
}

//===================================================================
Form_MainWindow::~Form_MainWindow()
{
    // Set the display thread end to true
    p_sharedData->m_flagDisplayThreadEnded = true;

    // Wait for other threads to end
    while((p_sharedData->m_flagControlThreadEnd == false) || (p_sharedData->m_flagStateMachineThreadEnd == false))
    {

    }

    delete ui;
}

//===================================================================
void Form_MainWindow::Init()
{

}

//===================================================================
