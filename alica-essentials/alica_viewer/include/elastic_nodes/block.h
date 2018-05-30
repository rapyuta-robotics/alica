#ifndef ELASTIC_NODES_BLOCK_H
#define ELASTIC_NODES_BLOCK_H

#include <QGraphicsItem>

namespace elastic_nodes
{

class Node;

class Block : public QGraphicsItem
{
  public:
    static int count;
    static void reset();
    Block(Node* sourceNode, Node* destNode);
    enum
    {
        Type = UserType + 3
    };
    int type() const override { return Type; }

  protected:
    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  protected:
    Node* _source;
    Node* _dest;
    QRectF _boundary;
};

} // namespace elastic_nodes

#endif // ELASTIC_NODES_BLOCK_H