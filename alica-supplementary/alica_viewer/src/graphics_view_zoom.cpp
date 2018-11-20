#include "alica_viewer/graphics_view_zoom.h"
#include <QApplication>
#include <QMouseEvent>
#include <QScrollBar>
#include <qmath.h>

namespace alica
{

GraphicsViewZoom::GraphicsViewZoom(QGraphicsView* view)
    : QObject(view)
    , _view(view)
{
    _view->viewport()->installEventFilter(this);
    _view->setMouseTracking(true);
    _modifiers = Qt::ControlModifier;
    _zoomFactorBase = 1.0015;
}

void GraphicsViewZoom::gentleZoom(double factor)
{
    _view->scale(factor, factor);
    _view->centerOn(_targetScenePos);
    QPointF deltaViewportPos = _targetViewportPos - QPointF(_view->viewport()->width() / 2.0, _view->viewport()->height() / 2.0);
    QPointF viewportCenter = _view->mapFromScene(_targetScenePos) - deltaViewportPos;
    _view->centerOn(_view->mapToScene(viewportCenter.toPoint()));
    emit zoomed();
}

void GraphicsViewZoom::setModifiers(Qt::KeyboardModifiers modifiers)
{
    _modifiers = modifiers;
}

void GraphicsViewZoom::setZoomFactorBase(double value)
{
    _zoomFactorBase = value;
}

bool GraphicsViewZoom::eventFilter(QObject* object, QEvent* event)
{
    if (event->type() == QEvent::MouseMove) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        QPointF delta = _targetViewportPos - mouseEvent->pos();
        if (qAbs(delta.x()) > 5 || qAbs(delta.y()) > 5) {
            _targetViewportPos = mouseEvent->pos();
            _targetScenePos = _view->mapToScene(mouseEvent->pos());
        }
    } else if (event->type() == QEvent::Wheel) {
        QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);
        if (QApplication::keyboardModifiers() == _modifiers) {
            if (wheelEvent->orientation() == Qt::Vertical) {
                double angle = wheelEvent->angleDelta().y();
                double factor = qPow(_zoomFactorBase, angle);
                gentleZoom(factor);
                return true;
            }
        }
    }
    Q_UNUSED(object)
    return false;
}

} // namespace alica