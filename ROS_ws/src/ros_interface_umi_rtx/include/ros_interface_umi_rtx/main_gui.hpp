#ifndef __GUI__
#define __GUI__

#include <QApplication>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QObject>
#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QPalette>
#include <QDockWidget>
#include <QProcess>
#include <QVBoxLayout>
#include <QCheckBox>

#include "ros_interface_umi_rtx/node_commands.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include "rviz_common/display.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include <rviz_common/config.hpp>
#include <rviz_common/yaml_config_reader.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

using namespace std;

namespace rviz_common
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class MainGUI : public QMainWindow, public rviz_common::WindowManagerInterface {
public:
    MainGUI(QApplication * app,
            const std::shared_ptr<Objective_node>&  ros2_node, 
            rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, 
            QWidget* parent = nullptr);
    ~MainGUI() override;

    double x,y,z,pitch=0.,roll=0.;

    bool manual_on = true;

    QWidget * getParentWindow() override;
    rviz_common::PanelDockWidget * addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating) override;
    void setStatus(const QString & message) override;

private:
    const shared_ptr<Objective_node> ros2_node;

    QApplication* app_;
    QWidget* main_widget;
    QDockWidget* RightDockWidget;
    QDockWidget* TopDockWidget;

    rviz_common::RenderPanel * render_panel_;
    rviz_common::Display *TF_, *Model_;
    rviz_common::VisualizationManager * manager_;

    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;

    void initializeRViz();

    void launchRViz();
    void toggleRViz(int state);

private slots:
    void closeEvent(QCloseEvent *event);

};

#endif