#include "alica_client/AlicaClient.h"
#include "alica/AlicaWidget.h"
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/node_handle.h>

namespace alica_client
{

AlicaClient::AlicaClient()
    : rqt_gui_cpp::Plugin()
    , widget(0)
    , alicaWidget(0)
{
    setObjectName("AlicaClient");
    rosNode = new ros::NodeHandle();
}

void AlicaClient::initPlugin(qt_gui_cpp::PluginContext& context)
{
    alicaWidget = new alica::AlicaWidget();

    if (context.serialNumber() > 1) {
        alicaWidget->qframe->setWindowTitle(alicaWidget->qframe->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    // TODO: create management widget, which takes alicaWidgets
    this->widget = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout();
    layout->insertWidget(0, this->alicaWidget->qframe, 0, Qt::AlignmentFlag::AlignLeft);
    this->widget->setLayout(layout);
    context.addWidget(widget);

    aliceClientSubscriber = rosNode->subscribe("/AlicaEngine/AlicaEngineInfo", 10, &alica::AlicaWidget::handleAlicaEngineInfo, this->alicaWidget);
}

void AlicaClient::shutdownPlugin() {}

void AlicaClient::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {}

void AlicaClient::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {}

} // namespace alica_client

PLUGINLIB_EXPORT_CLASS(alica_client::AlicaClient, rqt_gui_cpp::Plugin)
