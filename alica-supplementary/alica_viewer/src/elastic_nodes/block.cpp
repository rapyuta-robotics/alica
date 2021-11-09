#include <QPainter>

#include <elastic_nodes/block.h>
#include <elastic_nodes/node.h>

namespace elastic_nodes
{

QColor colours[] = {QColor("red"),   QColor("darkRed"),   QColor("darkCyan"), QColor("darkMagenta"),
                    QColor("green"), QColor("darkGreen"), QColor("yellow"),   QColor("blue")};

int Block::count = -1;

void Block::reset()
{
    count = -1;
}

Block::Block(Node* sourceNode, Node* destNode)
{
    count = (count + 1) % ((sizeof(colours) / sizeof(*colours)));
    setAcceptedMouseButtons(0);
    _source = sourceNode;
    _dest = destNode;
    QPointF startPos = mapFromItem(_source, Node::nodeStartPos);
    QPointF endPos = mapFromItem(_dest, Node::nodeEndPos);
    if (startPos.x() > endPos.x()) {
        startPos = mapFromItem(_dest, Node::nodeStartPos);
        endPos = mapFromItem(_source, Node::nodeEndPos);
        qreal tmp = startPos.y();
        startPos.setY(endPos.y());
        endPos.setY(tmp);
    }
    _boundary = QRectF(startPos, endPos);
    qreal buffer = 50;
    _boundary.adjust(-buffer, -buffer, buffer, buffer);
}

QRectF Block::boundingRect() const
{
    qreal margin = 2;
    return _boundary.adjusted(-margin, -margin, margin, margin);
}

void Block::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
    // Draw boundary
    qreal buffer = 1;
    painter->setPen(QPen(colours[count], 1, Qt::DashLine));
    painter->drawRect(_boundary.adjusted(-buffer, -buffer, buffer, buffer));
    // Add Text
    QString planName = _source->getPlanName();
    float factor = _boundary.width() / painter->fontMetrics().width(planName);
    if ((factor > 0.5 && factor < 1)) {
        QFont f = painter->font();
        f.setPointSizeF(f.pointSizeF() * factor);
        painter->setFont(f);
    }
    painter->drawText(_boundary, Qt::AlignTop | Qt::AlignLeft | Qt::TextDontClip | Qt::TextWordWrap, planName);
}

} // namespace elastic_nodes