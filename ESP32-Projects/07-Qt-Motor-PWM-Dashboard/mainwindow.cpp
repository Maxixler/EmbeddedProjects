#include "mainwindow.h"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QPalette>
#include <QStringList>
#include <QtCore/qmath.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), simStep(0)
{
    setWindowTitle("ESP32 Motor PWM Dashboard");
    resize(800, 700);

    // Dark Theme Background
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(25, 25, 30));
    pal.setColor(QPalette::WindowText, Qt::white);
    setPalette(pal);
    setAutoFillBackground(true);

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    
    QLabel *headerLabel = new QLabel("ESP32 MOTOR PWM CONTROL", this);
    QFont headerFont = headerLabel->font();
    headerFont.setPointSize(24);
    headerFont.setBold(true);
    headerLabel->setFont(headerFont);
    headerLabel->setAlignment(Qt::AlignCenter);
    headerLabel->setStyleSheet("color: #00FFC8; margin-top: 20px; margin-bottom: 20px;");
    
    mainLayout->addWidget(headerLabel);

    // -------------- SERIAL PORT UI ----------------
    QHBoxLayout *serialLayout = new QHBoxLayout();
    
    portSelector = new QComboBox(this);
    portSelector->setMinimumWidth(150);
    portSelector->setStyleSheet("background-color: #303035; color: white; padding: 5px; font-size: 14px;");
    populatePorts();

    QPushButton *refreshButton = new QPushButton("Refresh Ports", this);
    refreshButton->setStyleSheet("background-color: #444; color: white; padding: 5px 15px; font-size: 14px;");
    connect(refreshButton, &QPushButton::clicked, this, &MainWindow::populatePorts);

    connectButton = new QPushButton("Connect", this);
    connectButton->setStyleSheet("background-color: #2e8b57; color: white; font-weight: bold; padding: 5px 20px; font-size: 14px;");
    connect(connectButton, &QPushButton::clicked, this, &MainWindow::toggleConnection);

    statusLabel = new QLabel("Simulation Mode Active", this);
    statusLabel->setStyleSheet("color: #FFB300; font-size: 14px; font-weight: bold; margin-left: 20px;");

    serialLayout->addWidget(new QLabel("Port: ", this));
    serialLayout->addWidget(portSelector);
    serialLayout->addWidget(refreshButton);
    serialLayout->addWidget(connectButton);
    serialLayout->addWidget(statusLabel);
    serialLayout->addStretch();
    
    mainLayout->addLayout(serialLayout);
    // ----------------------------------------------

    QGridLayout *gridLayout = new QGridLayout();
    
    for (int i = 0; i < 4; ++i) {
        motorGauges[i] = new CircularGauge(this);
        motorGauges[i]->setTitle(QString("Motor %1").arg(i + 1));
        motorGauges[i]->setValue(0);
        
        int row = i / 2;
        int col = i % 2;
        gridLayout->addWidget(motorGauges[i], row, col, Qt::AlignCenter);
    }
    
    mainLayout->addLayout(gridLayout);

    // Setup Serial Port
    serialPort = new QSerialPort(this);
    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    // Setup simulation
    simTimer = new QTimer(this);
    connect(simTimer, &QTimer::timeout, this, &MainWindow::updateSimulation);
    simTimer->start(100); // Start simulation by default
}

MainWindow::~MainWindow() {
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void MainWindow::populatePorts() {
    portSelector->clear();
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        portSelector->addItem(info.portName() + " (" + info.description() + ")", info.portName());
    }
}

void MainWindow::toggleConnection() {
    if (serialPort->isOpen()) {
        serialPort->close();
        connectButton->setText("Connect");
        connectButton->setStyleSheet("background-color: #2e8b57; color: white; font-weight: bold; padding: 5px 20px; font-size: 14px;");
        statusLabel->setText("Disconnected - Simulation resuming");
        statusLabel->setStyleSheet("color: #FFB300; font-size: 14px; font-weight: bold; margin-left: 20px;");
        simTimer->start(100);
    } else {
        QString portName = portSelector->currentData().toString();
        if (portName.isEmpty()) return;

        serialPort->setPortName(portName);
        serialPort->setBaudRate(QSerialPort::Baud115200);
        
        if (serialPort->open(QIODevice::ReadOnly)) {
            connectButton->setText("Disconnect");
            connectButton->setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; padding: 5px 20px; font-size: 14px;");
            statusLabel->setText("Connected to " + portName);
            statusLabel->setStyleSheet("color: #00FFC8; font-size: 14px; font-weight: bold; margin-left: 20px;");
            simTimer->stop(); // Stop simulation
        } else {
            statusLabel->setText("Failed to open " + portName);
            statusLabel->setStyleSheet("color: red; font-size: 14px; font-weight: bold; margin-left: 20px;");
        }
    }
}

void MainWindow::readSerialData() {
    serialBuffer.append(serialPort->readAll());
    
    // Process line by line
    while (serialBuffer.contains('\n')) {
        int newlineIndex = serialBuffer.indexOf('\n');
        QString line = QString::fromLatin1(serialBuffer.left(newlineIndex)).trimmed();
        serialBuffer.remove(0, newlineIndex + 1);

        // Expected format: PWM:25,50,75,100
        if (line.startsWith("PWM:")) {
            QString data = line.mid(4);
            QStringList values = data.split(',');
            if (values.size() == 4) {
                for (int i = 0; i < 4; ++i) {
                    bool ok;
                    int pwmValue = values[i].toInt(&ok);
                    if (ok) {
                        motorGauges[i]->setValue(pwmValue);
                    }
                }
            }
        }
    }
}

void MainWindow::updateSimulation() {
    simStep++;
    // Create some fake varying PWM values using sine waves
    for (int i = 0; i < 4; ++i) {
        double offset = i * (M_PI / 2.0); // phase shift
        int val = (int)((qSin(simStep * 0.1 + offset) + 1.0) * 50.0);
        motorGauges[i]->setValue(val);
    }
}
