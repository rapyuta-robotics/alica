
/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in _source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of _source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QPainter>
#include <qmath.h>

#include <elastic_nodes/edge.h>
#include <elastic_nodes/node.h>

namespace elastic_nodes
{

Edge::Edge(Node* sourceNode, Node* destNode, Direction direction)
    : _arrowSize(10)
    , _direction(direction)
{
    setAcceptedMouseButtons(0);
    _source = sourceNode;
    _dest = destNode;
    if (_source && _dest) {
        _source->addEdge(this);
        _dest->addEdge(this);
        adjust();
    }
}

void Edge::adjust()
{
    if (!_source || !_dest)
        return;

    QLineF line;
    if (_direction == Direction::DOWN) {
        line = QLineF(mapFromItem(_source, Node::ellipseBottomCenter), mapFromItem(_dest, Node::ellipseTopCenter));
    } else {
        line = QLineF(mapFromItem(_source, Node::ellipseRightCenter), mapFromItem(_dest, Node::ellipseLeftCenter));
    }
    qreal length = line.length();

    prepareGeometryChange();

    if (length > qreal(20.)) {
        QPointF edgeOffset((line.dx() * 10) / length, (line.dy() * 10) / length);
        _sourcePoint = line.p1() + edgeOffset;
        _destPoint = line.p2() - edgeOffset;
    } else {
        _sourcePoint = _destPoint = line.p1();
    }
}

QRectF Edge::boundingRect() const
{
    if (!_source || !_dest)
        return QRectF();

    qreal penWidth = 1;
    qreal extra = (penWidth + _arrowSize) / 2.0;

    return QRectF(_sourcePoint, QSizeF(_destPoint.x() - _sourcePoint.x(), _destPoint.y() - _sourcePoint.y()))
        .normalized()
        .adjusted(-extra, -extra, extra, extra);
}

void Edge::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
    if (!_source || !_dest)
        return;

    QLineF line(_sourcePoint, _destPoint);
    if (qFuzzyCompare(line.length(), qreal(0.)))
        return;

    // Draw the line itself
    painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->drawLine(line);

    // Draw the arrows
    double angle = std::atan2(-line.dy(), line.dx());

    QPointF destArrowP1 = _destPoint + QPointF(sin(angle - M_PI / 3) * _arrowSize, cos(angle - M_PI / 3) * _arrowSize);
    QPointF destArrowP2 = _destPoint + QPointF(sin(angle - M_PI + M_PI / 3) * _arrowSize, cos(angle - M_PI + M_PI / 3) * _arrowSize);

    painter->setBrush(Qt::black);
    painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
}

} // namespace elastic_nodes