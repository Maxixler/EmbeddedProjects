#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include "circulargauge.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateSimulation();
    void readSerialData();
    void toggleConnection();
    void populatePorts();

private:
    CircularGauge *motorGauges[4];
    QTimer *simTimer;
    int simStep;

    QSerialPort *serialPort;
    QComboBox *portSelector;
    QPushButton *connectButton;
    QLabel *statusLabel;
    QByteArray serialBuffer;
};

#endif // MAINWINDOW_H
