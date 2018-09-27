#include <QGraphicsView>
#include <QObject>

/*!
 * This class adds ability to zoom QGraphicsView using mouse wheel. The point under cursor
 * remains motionless while it's possible.
 *
 * Note that it becomes not possible when the scene's
 * size is not large enough comparing to the viewport size. QGraphicsView centers the picture
 * when it's smaller than the view. And QGraphicsView's scrolls boundaries don't allow to
 * put any picture point at any viewport position.
 *
 * When the user starts scrolling, this class remembers original scene position and
 * keeps it until scrolling is completed. It's better than getting original scene position at
 * each scrolling step because that approach leads to position errors due to before-mentioned
 * positioning restrictions.
 *
 * When zommed using scroll, this class emits zoomed() signal.
 *
 * Usage:
 *
 *   new GraphicsViewZoom(view);
 *
 * The object will be deleted automatically when the view is deleted.
 *
 * You can set keyboard modifiers used for zooming using set_modified(). Zooming will be
 * performed only on exact match of modifiers combination. The default modifier is Ctrl.
 *
 * You can change zoom velocity by calling set_zoom_factor_base().
 * Zoom coefficient is calculated as zoom_factor_base^angle_delta
 * (see QWheelEvent::angleDelta).
 * The default zoom factor base is 1.0015.
 */

namespace alica
{

class GraphicsViewZoom : public QObject
{
    Q_OBJECT
  public:
    GraphicsViewZoom(QGraphicsView* view);
    void gentleZoom(double factor);
    void setModifiers(Qt::KeyboardModifiers modifiers);
    void setZoomFactorBase(double value);

  private:
    bool eventFilter(QObject* object, QEvent* event);

    QGraphicsView* _view;
    Qt::KeyboardModifiers _modifiers;
    double _zoomFactorBase;
    QPointF _targetScenePos;
    QPointF _targetViewportPos;

  Q_SIGNALS:
    void zoomed();
};

} // namespace alica