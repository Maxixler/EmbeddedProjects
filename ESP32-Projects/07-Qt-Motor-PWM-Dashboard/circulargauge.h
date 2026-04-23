#ifndef CIRCULARGAUGE_H
#define CIRCULARGAUGE_H

#include <QWidget>
#include <QColor>

class CircularGauge : public QWidget {
    Q_OBJECT
    Q_PROPERTY(int value READ value WRITE setValue NOTIFY valueChanged)

public:
    explicit CircularGauge(QWidget *parent = nullptr);

    int value() const;
    void setValue(int value);

    QString title() const;
    void setTitle(const QString &title);

protected:
    void paintEvent(QPaintEvent *event) override;

signals:
    void valueChanged(int value);

private:
    int m_value;          // PWM value 0-100
    QString m_title;      // e.g. "Motor 1"
    QColor m_ringColor;
    QColor m_bgColor;
    QColor m_textColor;
};

#endif // CIRCULARGAUGE_H
