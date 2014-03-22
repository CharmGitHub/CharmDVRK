#include <QtGui/QApplication>
#include "form_mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Form_MainWindow w;
    w.show();
    
    return a.exec();
}
