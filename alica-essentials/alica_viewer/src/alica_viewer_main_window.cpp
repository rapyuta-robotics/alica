#include "alica_viewer/alica_viewer_main_window.h"

#include "elastic_nodes/block.h"
#include "elastic_nodes/edge.h"
#include "elastic_nodes/node.h"

namespace alica
{

AlicaViewerMainWindow::AlicaViewerMainWindow(int argc, char* argv[], QWidget* parent)
    : QMainWindow(parent)
    , _interfaceNode(argc, argv)
    , _alicaPlan(argc, argv)
    , _offset(0)
{
    _ui.setupUi(this);

    _scene = new QGraphicsScene(this);
    _scene->setItemIndexMethod(QGraphicsScene::NoIndex);

    _zoom = new GraphicsViewZoom(_ui.graphicsView);
    _zoom->setModifiers(Qt::NoModifier);

    _ui.graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    _ui.graphicsView->setScene(_scene);
    _ui.graphicsView->fitInView(_scene->sceneRect());

    // fill combo box
    const AgentInfoMap& aiMap = _alicaPlan.getAgentInfoMap();
    for (const auto& agentInfo : aiMap) {
        _agentIdVector.push_back(agentInfo.first);
    }
    std::sort(_agentIdVector.begin(), _agentIdVector.end(), supplementary::AgentIDComparator());

    _ui.agentIdComboBox->addItem("Combined");
    _ui.agentIdComboBox->addItem("All");
    for (const supplementary::AgentID* agentId : _agentIdVector) {
        const AgentInfo* ai = _alicaPlan.getAgentInfo(agentId);
        if (ai) {
            _ui.agentIdComboBox->addItem(QString::fromStdString(ai->name));
        }
    }

    // Connect the signals and slots between interface to main window
    QObject::connect(&_interfaceNode, &AlicaViewerInterface::shutdown, this, &AlicaViewerMainWindow::close);
    QObject::connect(&_interfaceNode, &AlicaViewerInterface::alicaEngineInfoUpdate, this, &AlicaViewerMainWindow::alicaEngineInfoUpdate);
    QObject::connect(&_interfaceNode, &AlicaViewerInterface::alicaPlanInfoUpdate, this, &AlicaViewerMainWindow::alicaPlanInfoUpdate);
}

elastic_nodes::Node* AlicaViewerMainWindow::addStateToScene(const PlanTree* planTreeNode)
{
    if (planTreeNode == nullptr) {
        return nullptr;
    }

    elastic_nodes::Node* parentNode = nullptr;

    if (planTreeNode->isValid()) {
        AgentGrp robotIds;
        planTreeNode->getRobotsSorted(robotIds);
        std::string robotIdList = "[ ";
        for (const supplementary::AgentID* robotId : robotIds) {
            robotIdList += _alicaPlan.getAgentInfo(robotId)->name + std::string(", ");
        }
        robotIdList.erase(robotIdList.end() - 2, robotIdList.end());
        robotIdList += std::string(" ]");

        std::string stateName = planTreeNode->getState()->getName();
        std::string entrypointName = planTreeNode->getEntryPoint()->getTask()->getName();
        std::string planName = planTreeNode->getEntryPoint()->getPlan()->getName();

        parentNode = new elastic_nodes::Node(stateName, entrypointName, planName, robotIdList);
        _scene->addItem(parentNode); // addItem() takes ownership of the arguement
        int x = (planTreeNode->getX() + _offset) * 300;
        int y = planTreeNode->getY() * 300;
        parentNode->setPos(x, y);

        if (planTreeNode->getParent() == nullptr) {
            // Draw a bounding rectangle for every plan
            elastic_nodes::Block* block = new elastic_nodes::Block(parentNode, parentNode);
            _scene->addItem(block);
        }
    }

    const PlanTreeVectorMap& children = planTreeNode->getChildren();
    for (const auto& ptvMapPair : children) {
        elastic_nodes::Node* startBlockNode = nullptr;
        elastic_nodes::Node* childNode = nullptr;
        for (int i = 0; i < static_cast<int>(ptvMapPair.second.size()); ++i) {
            childNode = addStateToScene(ptvMapPair.second[i]);
            _scene->addItem(new elastic_nodes::Edge(parentNode, childNode)); // addItem() takes ownership of the arguement
            if (i == 0) {
                startBlockNode = childNode;
            }
        }
        // Draw a bounding rectangle for every plan
        _scene->addItem(new elastic_nodes::Block(startBlockNode, childNode));
    }

    return parentNode;
}

void AlicaViewerMainWindow::updateNodes()
{
    _scene->clear();
    int indexSelected = _ui.agentIdComboBox->currentIndex();
    const PlanTreeMap& ptMap = _alicaPlan.getPlanTrees();
    elastic_nodes::Block::reset();
    _offset = 0;
    if (indexSelected == 0) { // Combined
        PlanTree combinedPlanTree;
        _alicaPlan.getCombinedPlanTree(combinedPlanTree);
        addStateToScene(&combinedPlanTree);
    } else if (indexSelected == 1) { // All
        for (const auto& ptMapPair : ptMap) {
            elastic_nodes::Block::reset();
            addStateToScene(ptMapPair.second);
            _offset += 1; // To display each graph separately
        }
    } else { // individual agents
        PlanTreeMap::const_iterator planTreeEntry = ptMap.find(_agentIdVector[indexSelected - 2]);
        if (planTreeEntry != ptMap.end()) {
            addStateToScene(planTreeEntry->second);
        }
    }
}

AlicaViewerMainWindow::~AlicaViewerMainWindow()
{
    delete _scene;
}

void AlicaViewerMainWindow::alicaEngineInfoUpdate(const AlicaEngineInfo& msg) {}

void AlicaViewerMainWindow::alicaPlanInfoUpdate(const PlanTreeInfo& msg)
{
    _alicaPlan.handlePlanTreeInfo(msg);
    updateNodes();
}

} // namespace alica