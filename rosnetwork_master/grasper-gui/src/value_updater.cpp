#include "value_updater.hpp"

#include <QDebug>
#include <QPainter>
#include <QRect>

ValueUpdater::ValueUpdater(QQuickItem *parent) : m_parent(parent), m_value("0") {}

void ValueUpdater::setValue(QString value)
{
    if (value == m_value) return;
    m_value = value;
    update();
    emit valueChanged();
}

void drawText(
    QPainter &painter,
    qreal x,
    qreal y,
    Qt::Alignment flags,
    const QString &text,
    QRectF *boundingRect = 0)
{
    const qreal size = 32767.0;
    QPointF corner(x, y - size);
    if (flags & Qt::AlignHCenter)
        corner.rx() -= size / 2.0;
    else if (flags & Qt::AlignRight)
        corner.rx() -= size;
    if (flags & Qt::AlignVCenter)
        corner.ry() += size / 2.0;
    else if (flags & Qt::AlignTop)
        corner.ry() += size;
    else
        flags |= Qt::AlignBottom;
    QRectF rect{corner.x(), corner.y(), size, size};
    painter.drawText(rect, flags, text, boundingRect);
}

void ValueUpdater::paint(QPainter *painter)
{
    // Unhardcode all the things
    QFont font = painter->font();
    int pixel_size = 20;
    font.setPixelSize(pixel_size);
    painter->setFont(font);
    painter->setPen(QColor("black"));  // TODO unhardcode
    QRectF boundingRect;
    int width = 100;
    int height = 50;
    drawText(*painter, width / 2, height / 2, Qt::AlignCenter, m_value, &boundingRect);
    //    QRect rectangle = QRect(0, 0, height, width);
    //    painter->drawText(rectangle, Qt::AlignCenter, m_value, &boundingRect);
}
