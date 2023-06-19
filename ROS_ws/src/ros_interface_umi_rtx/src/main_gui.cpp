#include "ros_interface_umi_rtx/main_gui.hpp"

MainGUI::MainGUI(QApplication * app,
                 const std::shared_ptr<Objective_node>& ros2_node, 
                 rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, 
                 QWidget* parent)
  : app_(app), ros2_node(ros2_node), rviz_ros_node_(rviz_ros_node), QMainWindow(parent)
{
    main_widget = new QWidget(this);

    QGridLayout* main_layout = new QGridLayout;
    main_layout->setSpacing(10);
    main_layout->setMargin(10);

    QLabel *Title = new QLabel("UMI-RTX Interface");
    Title->setAlignment(Qt::AlignHCenter);
    main_layout->addWidget(Title, 0,1);

    /// Add sliders
    QLabel *label_x = new QLabel("X coordinates :");
    label_x->setAlignment(Qt::AlignRight);
    main_layout->addWidget(label_x,1,0);
    // Création du slider
    QSlider* slider_x = new QSlider(Qt::Horizontal);
    slider_x->setRange(-30, 30); // Plage de valeurs du slider
    slider_x->setSingleStep(1); // Pas d'incrémentation du slider
    main_layout->addWidget(slider_x,1,1);
    // Création du spin box pour afficher la valeur flottante x
    QDoubleSpinBox* spinBox_x = new QDoubleSpinBox;
    spinBox_x->setMaximum(30);
    spinBox_x->setMinimum(-30);
    spinBox_x->setSingleStep(1);
    main_layout->addWidget(spinBox_x,1,2);
    // Lier le slider et le spin box
    QObject::connect(slider_x, &QSlider::valueChanged, spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_x, &QSlider::setValue);


    QLabel *label_y = new QLabel("Y coordinates :");
    label_y->setAlignment(Qt::AlignRight);
    main_layout->addWidget(label_y,2,0);
    // Création du slider
    QSlider* slider_y = new QSlider(Qt::Horizontal);
    slider_y->setRange(20, 70); // Plage de valeurs du slider
    slider_y->setSingleStep(1); // Pas d'incrémentation du slider
    main_layout->addWidget(slider_y,2,1);
    // Création du spin box pour afficher la valeur flottante x
    QDoubleSpinBox* spinBox_y = new QDoubleSpinBox;
    spinBox_y->setMaximum(70);
    spinBox_y->setMinimum(20);
    main_layout->addWidget(spinBox_y,2,2);
    // Lier le slider et le spin box
    QObject::connect(slider_y, &QSlider::valueChanged, spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_y, &QSlider::setValue);


    QLabel *label_z = new QLabel("Z coordinates :");
    label_z->setAlignment(Qt::AlignRight);
    main_layout->addWidget(label_z,3,0);
    // Création du slider
    QSlider* slider_z = new QSlider(Qt::Horizontal);
    slider_z->setRange(10, 70); // Plage de valeurs du slider
    slider_z->setSingleStep(1); // Pas d'incrémentation du slider
    main_layout->addWidget(slider_z,3,1);
    // Création du spin box pour afficher la valeur flottante x
    QDoubleSpinBox* spinBox_z = new QDoubleSpinBox;
    spinBox_z->setMaximum(70);
    spinBox_z->setMinimum(10);
    main_layout->addWidget(spinBox_z,3,2);
    // Lier le slider et le spin box
    QObject::connect(slider_z, &QSlider::valueChanged, spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_z, &QSlider::setValue);



    QLabel *label_pitch = new QLabel("Pitch :");
    label_pitch->setAlignment(Qt::AlignRight);
    main_layout->addWidget(label_pitch,4,0);
    QSlider* slider_pitch = new QSlider(Qt::Horizontal);
    slider_pitch->setRange(0, 90);
    slider_pitch->setSingleStep(1);
    main_layout->addWidget(slider_pitch,4,1);
    QDoubleSpinBox* spinBox_pitch = new QDoubleSpinBox;
    spinBox_pitch->setMaximum(90);
    spinBox_pitch->setMinimum(0);
    spinBox_pitch->setSingleStep(1);
    main_layout->addWidget(spinBox_pitch,4,2);
    QObject::connect(slider_pitch, &QSlider::valueChanged, spinBox_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_pitch, &QSlider::setValue);

    QLabel *label_roll = new QLabel("Roll :");
    label_roll->setAlignment(Qt::AlignRight);
    main_layout->addWidget(label_roll,5,0);
    QSlider* slider_roll = new QSlider(Qt::Horizontal);
    slider_roll->setRange(0, 90);
    slider_roll->setSingleStep(1);
    main_layout->addWidget(slider_roll,5,1);
    QDoubleSpinBox* spinBox_roll = new QDoubleSpinBox;
    spinBox_roll->setMaximum(90);
    spinBox_roll->setMinimum(0);
    spinBox_roll->setSingleStep(1);
    main_layout->addWidget(spinBox_roll,5,2);
    QObject::connect(slider_roll, &QSlider::valueChanged, spinBox_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_roll, &QSlider::setValue);


    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        x = newValue/100;
    });
    QObject::connect(spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        y = newValue/100;
    });
    QObject::connect(spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        z = newValue/100;
    });
    QObject::connect(spinBox_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        pitch = newValue;
    });
    QObject::connect(spinBox_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        roll = newValue;
    });

    slider_x->setValue(0);
    slider_y->setValue(60);
    slider_z->setValue(60);

    // Création du bouton pour le switch
    QPushButton* switchButton = new QPushButton(this);
    switchButton->setCheckable(true);  // Permet de sélectionner/désélectionner le bouton
    switchButton->setChecked(true);  // Définit l'état initial du switch (désélectionné)
    // Personnalisation de l'apparence du switch
    switchButton->setFixedSize(120, 50);  // Définit la taille du bouton
    switchButton->setText("Manual mode");
    switchButton->setStyleSheet("QPushButton {"
                                "border: none;"
                                "background-color: #ccc;"
                                "border-radius: 10px"
                                "}"
                                "QPushButton:checked {"
                                    "background-color: #6c6;"
                                "}"
                                "QPushButton:!checked {"
                                    "background-color: #ccc;"
                                "}");

    // Connexion du signal clicked au slot correspondant pour réagir aux clics sur le switch
    connect(switchButton, &QPushButton::clicked, this, [=]() {
        manual_on = switchButton->isChecked();
        ros2_node->manual_control = switchButton->isChecked();
    });
    main_layout->addWidget(switchButton,0,3);

    // Print RVIZ in the window
    initializeRViz();

    QVBoxLayout* rviz_layout = new QVBoxLayout;
    rviz_layout->addWidget(render_panel_);

    videoLabel = new QLabel("");
    capture.open(0);
    timer = new QTimer;
    connect(timer, SIGNAL(timeout()), this, SLOT(updateFrame()));
    timer->start(33);
    rviz_layout->addWidget(videoLabel);

    QHBoxLayout* glob_layout = new QHBoxLayout;
    glob_layout->addLayout(rviz_layout);
    glob_layout->addLayout(main_layout);

    main_widget->setLayout(glob_layout);
    setCentralWidget(main_widget);
    setStyleSheet("background-color: #e0e8bd;");
}

MainGUI::~MainGUI()
{
    // capture.release();
}


void MainGUI::initializeRViz()
{
    app_->processEvents();
    render_panel_ = new rviz_common::RenderPanel(RightDockWidget);
    app_->processEvents();
    render_panel_->getRenderWindow()->initialize();
    auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
    manager_ = new rviz_common::VisualizationManager(render_panel_, rviz_ros_node_, this, clock);
    render_panel_->initialize(manager_);
    app_->processEvents();

    QString config_file = QString::fromStdString(ament_index_cpp::get_package_share_directory("ros_interface_umi_rtx")+"/rviz/rviz_basic_settings.rviz");

    rviz_common::YamlConfigReader config_reader;
    rviz_common::Config config;
    config_reader.readFile(config, config_file);
    
    manager_->load(config);

    TF_ = manager_->createDisplay("rviz_default_plugins/TF","TF",true);
    Model_ = manager_->createDisplay("rviz_default_plugins/RobotModel","RobotModel",true);
    assert(TF_ != NULL);
    assert(Model_ != NULL);

    Model_->setTopic(QString::fromStdString("/robot_description"),QString::fromStdString("std_msgs/msg/String"));

    manager_->getToolManager()->addTool(QString::fromStdString("rviz_default_plugins/MoveCamera"));

    manager_->initialize();
    manager_->startUpdate();
}

void MainGUI::updateFrame()
{
    capture.read(*frame); // Capture d'une image du flux vidéo

    // Convertir l'image OpenCV en QImage
    *image = QImage(frame->data, frame->cols, frame->rows, frame->step, QImage::Format_RGB888).rgbSwapped();

    // Afficher l'image dans le QLabel
    videoLabel->setPixmap(QPixmap::fromImage(*image));
    videoLabel->adjustSize(); // Ajuster la taille du QLabel pour correspondre à l'image
}

QWidget *
MainGUI::getParentWindow()
{
  return this;
}

rviz_common::PanelDockWidget *
MainGUI::addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating)
{
  return nullptr;
}

void
MainGUI::setStatus(const QString & message)
{
  // TODO(mjeronimo)
}

void MainGUI::closeEvent(QCloseEvent * event)
{
  QMainWindow::closeEvent(event);
  rclcpp::shutdown();
}

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}


int main(int argc, char* argv[])
{   
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);
    auto ros_node_abs = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    auto ros2_node = std::make_shared<Objective_node>();
    auto gui_app = std::make_shared<MainGUI>(&app,ros2_node,ros_node_abs);

    app.processEvents();
    gui_app->showMaximized();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);

    while (rclcpp::ok())
    {   
        if (ros2_node->manual_control){
            ros2_node->update_state(gui_app->x,gui_app->y,gui_app->z,gui_app->roll,gui_app->pitch);
        }
        // ros2_node->update_state(gui_app->x,gui_app->y,gui_app->z,gui_app->roll,gui_app->pitch);
        exec.spin_some();
        app.processEvents();
    }
    signal(SIGINT, siginthandler);

    exec.remove_node(ros2_node);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
