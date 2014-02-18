#ifndef FRAMELABEL_H
#define FRAMELABEL_H

#include <QLabel>
#include <QRubberBand>
#include <QPoint>

class FrameLabel : public QLabel
{
    Q_OBJECT
public:
    explicit FrameLabel(QWidget *parent = 0);
    
protected:
    void mouseMoveEvent(QMouseEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);

public slots:
    void selectROI() { selectingROI = true; }

signals:
    void roiSelected(QRect, QPoint);

private:
    QRect ROI;
    QRubberBand * rubber;
    bool selectingROI;
    QPoint origen;
};

#endif // FRAMELABEL_H
