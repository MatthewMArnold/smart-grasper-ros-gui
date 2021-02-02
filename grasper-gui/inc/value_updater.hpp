#ifndef SENSOR_READING_SUB_HPP_
#define SENSOR_READING_SUB_HPP_

#include <QQuickPaintedItem>

class ValueUpdater : public QQuickPaintedItem
{
    Q_OBJECT
public:
    explicit ValueUpdater(QQuickItem *parent = 0);

    Q_PROPERTY(QString value READ value WRITE setValue NOTIFY valueChanged);
    QString value() const { return m_value; }

    void paint(QPainter *painter) override;

public slots:
    void setValue(QString value);

signals:
    void valueChanged();

private:
    QQuickItem *m_parent;
    QString m_value;
};

#endif  // SENSOR_READING_SUB_HPP_
