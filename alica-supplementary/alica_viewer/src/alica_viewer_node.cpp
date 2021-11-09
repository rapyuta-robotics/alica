#include "alica_viewer/alica_viewer_main_window.h"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    std::cout << "Started Alica Viewer Application.\n";

    alica::AlicaViewerMainWindow window(argc, argv);
    window.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int ret = app.exec();
    return ret;
}
