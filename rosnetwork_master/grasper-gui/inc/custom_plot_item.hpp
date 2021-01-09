#ifndef CUSTOMPLOTITEM_HPP
#define CUSTOMPLOTITEM_HPP

#include <QObject>
#include <QtQuick/QQuickPaintedItem>

class QCustomPlot;
class QCPAbstractPlottable;
class PulseoxWorker;

class CustomPlotItem : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(int timeToDisplay MEMBER m_timeToDisplay)
    Q_PROPERTY(QString xAxisLabel MEMBER m_xAxisLabel)
    Q_PROPERTY(QString yAxisLabel MEMBER m_yAxisLabel)

public:
    CustomPlotItem(QQuickItem* parent = 0);
    virtual ~CustomPlotItem();

    void paint(QPainter* painter);

    Q_INVOKABLE void initCustomPlot();

    void setTimeToDisplay(int time) { m_timeToDisplay = time; }
    void setXAxisLabel(QString label) { m_xAxisLabel = label; }
    void setYAxisLabel(QString label) { m_yAxisLabel = label; }

    int timeToDisplay() const { return m_timeToDisplay; }
    QString xAxisLabel() const { return m_xAxisLabel; }
    QString yAxisLabel() const { return m_yAxisLabel; }

public slots:
    void graphData(double data, double time);

protected:
    void routeMouseEvents(QMouseEvent* event);

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mouseDoubleClickEvent(QMouseEvent* event);

    void setupPulseoxData(QCustomPlot* customPlot);

private:
    QCustomPlot* m_CustomPlot;

    double m_prevTime;
    int m_timeToDisplay;
    QString m_xAxisLabel;
    QString m_yAxisLabel;

private slots:
    void graphClicked(QCPAbstractPlottable* plottable);
    void onCustomReplot();
    void updateCustomPlotSize();
};

#endif  // CUSTOMPLOTITEM_HPP
