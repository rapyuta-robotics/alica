#ifndef alica_client__AlicaClient_H
#define alica_client__AlicaClient_H

#include <rqt_gui_cpp/plugin.h>

#include <ros/macros.h>
#include <ros/subscriber.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

namespace alica {
class AlicaWidget;
}

namespace alica_client {
using namespace std;

class AlicaClient : public rqt_gui_cpp::Plugin {
    Q_OBJECT

public:
    AlicaClient();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

    virtual void restoreSettings(
            const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    QWidget* widget;
    alica::AlicaWidget* alicaWidget;
    ros::NodeHandle* rosNode;
    ros::Subscriber aliceClientSubscriber;
};

}  // namespace alica_client

#endif  // alica_client__AlicaClient_H
