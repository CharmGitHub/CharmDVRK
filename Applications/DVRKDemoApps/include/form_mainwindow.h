#ifndef FORM_MAINWINDOW_H
#define FORM_MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class Form_MainWindow;
}

class Form_MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit Form_MainWindow(QWidget *parent = 0);
    ~Form_MainWindow();
    
private:
    Ui::Form_MainWindow *ui;
};

#endif // FORM_MAINWINDOW_H
