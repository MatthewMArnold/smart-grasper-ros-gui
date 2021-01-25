#include "value_updater.hpp"

#include <QDebug>
#include <QPainter>
#include <QRect>

ValueUpdater::ValueUpdater(QQuickItem *parent) : m_parent(parent), m_value("0")
{
}

void ValueUpdater::setValue(QString value)
{
    if (value == m_value) return;
    m_value = value;
    update();
    emit valueChanged();
}

void ValueUpdater::paint(QPainter *painter)
{
    // TODO fix
    QColor color("black");
    QRect rect(0, 0, 50, 20);
    for (auto ch : m_value)
    {
        //        QImage canvas(rect.width(), rect.height(),
        //        QImage::Format_RGBA8888); canvas.fill(QColor("transparent"));

        QFont font = painter->font();
        // font.setPixelSize(20);
        font.setPixelSize(rect.width());
        font.setBold(true);
        painter->setFont(font);
        painter->setPen(color);

        QRect bounding = QRect(0, 0, rect.width(), rect.height());
        painter->drawText(
            0,
            0,
            rect.width(),
            rect.height(),
            Qt::AlignCenter,
            ch,
            &bounding);

        //        QSGTexture *texture =
        //        this->window()->createTextureFromImage(canvas);
        //        m_textureList[ch] = texture;
    }
    //    qDebug() << "call to paint";
}
