#include <QApplication>
#include "flexiv/teleoperation/gui/mainwindow.hpp"

using namespace flexiv::teleoperation;

int main(int argc, char* argv[])
{
  // Start GUI
  QApplication app(argc, argv);
  MainWindow app_gui;
  app_gui.show();

  return app.exec();
}