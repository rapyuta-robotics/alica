#ifndef ALICA_VIEWER_ALICA_VIEWER_MAIN_WINDOW_H
#define ALICA_VIEWER_ALICA_VIEWER_MAIN_WINDOW_H

#include <QMainWindow>

#include "alica_viewer/alica_plan_parser.h"
#include "alica_viewer/alica_viewer_ros_interface.h"
#include "alica_viewer/graphics_view_zoom.h"
#include "ui_alica_viewer_main_window.h"

namespace elastic_nodes
{

class Node;
class Edge;
}

namespace alica
{

typedef AlicaViewerRosInterface AlicaViewerInterface;

class AlicaViewerMainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit AlicaViewerMainWindow(int argc, char* argv[], QWidget* parent = 0);
    ~AlicaViewerMainWindow();

  public Q_SLOTS:
    void alicaEngineInfoUpdate(const alica::AlicaEngineInfo& msg);
    void alicaPlanInfoUpdate(const alica::PlanTreeInfo& msg);

  private:
    elastic_nodes::Node* addStateToScene(const PlanTree* planTreeNode);
    void updateNodes();

    Ui::AlicaViewerMainWindow _ui;
    QGraphicsScene* _scene;
    GraphicsViewZoom* _zoom;
    AlicaViewerInterface _interfaceNode;
    AlicaPlan _alicaPlan;
    AgentGrp _agentIdVector;
    int _offset;
};

} // namespace alica

#endif // ALICA_VIEWER_ALICA_VIEWER_MAIN_WINDOW_H
