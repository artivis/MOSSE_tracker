#include <QPixmap>
#include <QRubberBand>
#include <QMouseEvent>
#include <QPoint>
#include <QRect>
#include <QSize>
#include <QWidget>
#include <QDebug>

#include "FrameLabel.h"

FrameLabel::FrameLabel(QWidget *parent)
    : QLabel(parent)
    , ROI(QRect())
    , selectingROI(false)
{
}

void FrameLabel::mouseMoveEvent(QMouseEvent *ev)
{
    rubber->setGeometry(QRect(origen, ev->pos()).normalized());
}

void FrameLabel::mousePressEvent(QMouseEvent *ev)
{
    if (selectingROI)
    {
        origen = ev->pos();
        rubber = new QRubberBand(QRubberBand::Rectangle, this);
        QPalette pal;
        pal.setBrush(QPalette::Highlight, QBrush(Qt::red));
        rubber->setPalette(pal);
        rubber->setGeometry(QRect(origen, QSize()));
        rubber->show();
    }
}

void FrameLabel::mouseReleaseEvent(QMouseEvent *ev)
{
    if (selectingROI)
    {
        rubber->hide();
        selectingROI = false;
        emit roiSelected(rubber->contentsRect(), origen);
    }
}
