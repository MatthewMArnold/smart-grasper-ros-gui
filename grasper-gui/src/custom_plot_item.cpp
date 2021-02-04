#include "custom_plot_item.hpp"

#include <QDebug>
#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickPaintedItem>

#include "qcustomplot.h"

CustomPlotItem::CustomPlotItem(QQuickItem* parent)
    : QQuickPaintedItem(parent),
      m_CustomPlot(nullptr)
{
    setFlag(QQuickItem::ItemHasContents, true);
    // setRenderTarget(QQuickPaintedItem::FramebufferObject);
    // setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::AllButtons);

    connect(this, &QQuickPaintedItem::widthChanged, this, &CustomPlotItem::updateCustomPlotSize);
    connect(this, &QQuickPaintedItem::heightChanged, this, &CustomPlotItem::updateCustomPlotSize);
}

CustomPlotItem::~CustomPlotItem()
{
    delete m_CustomPlot;
    m_CustomPlot = nullptr;
}

void CustomPlotItem::initCustomPlot()
{
    m_CustomPlot = new QCustomPlot();

    updateCustomPlotSize();

    setupPulseoxData(m_CustomPlot);

    connect(m_CustomPlot, &QCustomPlot::afterReplot, this, &CustomPlotItem::onCustomReplot);

    m_CustomPlot->replot();
}

void CustomPlotItem::paint(QPainter* painter)
{
    if (m_CustomPlot)
    {
        QPixmap picture(boundingRect().size().toSize());
        QCPPainter qcpPainter(&picture);

        // m_CustomPlot->replot();
        m_CustomPlot->toPainter(&qcpPainter);

        painter->drawPixmap(QPoint(), picture);
    }
}

void CustomPlotItem::mousePressEvent(QMouseEvent* event) { routeMouseEvents(event); }

void CustomPlotItem::mouseReleaseEvent(QMouseEvent* event) { routeMouseEvents(event); }

void CustomPlotItem::mouseMoveEvent(QMouseEvent* event) { routeMouseEvents(event); }

void CustomPlotItem::mouseDoubleClickEvent(QMouseEvent* event) { routeMouseEvents(event); }

void CustomPlotItem::graphClicked(QCPAbstractPlottable* plottable)
{
    qDebug() << Q_FUNC_INFO << QString("Clicked on graph '%1 ").arg(plottable->name());
}

void CustomPlotItem::routeMouseEvents(QMouseEvent* event)
{
    if (m_CustomPlot)
    {
        QMouseEvent* newEvent = new QMouseEvent(
            event->type(),
            event->localPos(),
            event->button(),
            event->buttons(),
            event->modifiers());
        QCoreApplication::postEvent(m_CustomPlot, newEvent);
    }
}

void CustomPlotItem::updateCustomPlotSize()
{
    if (m_CustomPlot)
    {
        m_CustomPlot->setGeometry(0, 0, width(), height());
    }
}

void CustomPlotItem::onCustomReplot() { update(); }

void CustomPlotItem::setupPulseoxData(QCustomPlot* customPlot)
{
    customPlot->addGraph();
    customPlot->xAxis->setLabel(m_xAxisLabel);
    customPlot->yAxis->setLabel(m_yAxisLabel);
    customPlot->xAxis->setRange(0, 1);
    customPlot->yAxis->setRange(0, 1);
}

void CustomPlotItem::graphData(double data, double time)
{
    if (time - m_prevTime > 2)
    {
        m_CustomPlot->graph(0)->addData(time, data);
        m_prevTime = time;
    }

    m_CustomPlot->graph(0)->rescaleValueAxis();
    m_CustomPlot->xAxis->setRange(time, m_timeToDisplay, Qt::AlignRight);
    m_CustomPlot->replot();
}

void CustomPlotItem::setXAxisLabel(QString label)
{
    m_xAxisLabel = label;
    m_CustomPlot->xAxis->setLabel(label);
    m_CustomPlot->replot();
}

void CustomPlotItem::setYAxisLabel(QString label)
{
    m_yAxisLabel = label;
    m_CustomPlot->yAxis->setLabel(label);
    m_CustomPlot->replot();
}
