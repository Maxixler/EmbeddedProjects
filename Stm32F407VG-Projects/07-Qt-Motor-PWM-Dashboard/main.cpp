#include "mainwindow.h"
#include <QApplication>
#include <QTimer>
#include <QPixmap>
#include <QDir>
#include <QCommandLineParser>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    
    QCommandLineParser parser;
    parser.setApplicationDescription("STM32 Motor PWM Dashboard");
    parser.addHelpOption();
    
    QCommandLineOption testModeOption(QStringList() << "t" << "test", "Run in test mode to generate screenshot and exit.");
    parser.addOption(testModeOption);
    parser.process(a);

    MainWindow w;
    w.show();

    if (parser.isSet(testModeOption)) {
        // Wait 1 second for UI to update simulation, then capture screenshot
        QTimer::singleShot(1000, [&]() {
            QPixmap pixmap = w.grab();
            QString screenshotPath = QDir::currentPath() + "/screenshot.png";
            pixmap.save(screenshotPath);
            QApplication::quit();
        });
    }

    return a.exec();
}
