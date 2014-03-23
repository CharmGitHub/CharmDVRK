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
    delete ui;
}

//===================================================================
