#include "circulargauge.h"
#include <QPainter>
#include <QConicalGradient>
#include <QRect>
#include <QPen>
#include <QFontDatabase>

CircularGauge::CircularGauge(QWidget *parent)
    : QWidget(parent), m_value(0), m_title("Motor"),
      m_ringColor(QColor(0, 255, 200)), // Neon Cyan/Green
      m_bgColor(QColor(40, 40, 45)),
      m_textColor(Qt::white)
{
    setMinimumSize(150, 150);
}

int CircularGauge::value() const { return m_value; }

void CircularGauge::setValue(int value) {
    if (value < 0) value = 0;
    if (value > 100) value = 100;
    if (m_value != value) {
        m_value = value;
        emit valueChanged(m_value);
        update();
    }
}

QString CircularGauge::title() const { return m_title; }

void CircularGauge::setTitle(const QString &title) {
    if (m_title != title) {
        m_title = title;
        update();
    }
}

void CircularGauge::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int width = this->width();
    int height = this->height();
    int size = qMin(width, height) - 20;
    
    QRectF rect((width - size) / 2.0, (height - size) / 2.0, size, size);

    // Draw Background Ring
    QPen bgPen(m_bgColor, 12, Qt::SolidLine, Qt::RoundCap);
    painter.setPen(bgPen);
    painter.drawArc(rect, -45 * 16, 270 * 16); // 270 degree arc

    // Draw active Ring
    if (m_value > 0) {
        qreal spanAngle = -(m_value / 100.0) * 270.0;
        
        QConicalGradient gradient(rect.center(), 225);
        gradient.setColorAt(0.0, QColor(0, 255, 200));
        gradient.setColorAt(1.0, QColor(0, 150, 255));
        
        QPen activePen(QBrush(gradient), 12, Qt::SolidLine, Qt::RoundCap);
        painter.setPen(activePen);
        painter.drawArc(rect, 225 * 16, spanAngle * 16);
    }

    // Draw Text Value
    painter.setPen(m_textColor);
    QFont valFont = painter.font();
    valFont.setPointSize(size / 5);
    valFont.setBold(true);
    painter.setFont(valFont);
    painter.drawText(rect, Qt::AlignCenter, QString::number(m_value) + "%");

    // Draw Title
    QFont titleFont = painter.font();
    titleFont.setPointSize(size / 12);
    titleFont.setBold(false);
    painter.setFont(titleFont);
    QRectF titleRect(rect.x(), rect.y() + size * 0.75, rect.width(), size * 0.25);
    painter.drawText(titleRect, Qt::AlignCenter, m_title);
}
