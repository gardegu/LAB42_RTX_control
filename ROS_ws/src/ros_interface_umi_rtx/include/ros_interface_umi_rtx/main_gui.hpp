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

#include <ros_interface_umi_rtx/node_commands.hpp>

#include <iostream>

using namespace std;

class MainGUI : public QMainWindow {
public:
    explicit MainGUI(const std::shared_ptr<Objective_node>&  ros2_node, QWidget* parent = nullptr);
    ~MainGUI() override;

    double x,y,z;

    bool manual_on = false;

private:
    const shared_ptr<Objective_node> ros2_node;

    QWidget* main_widget;

};

#endif