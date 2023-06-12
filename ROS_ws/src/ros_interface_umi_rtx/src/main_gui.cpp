#include "ros_interface_umi_rtx/main_gui.hpp"


MainGUI::MainGUI(const std::shared_ptr<Objective_node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
{
    main_widget = new QWidget(this);
    main_widget->setStyleSheet("background-color: #e0e8bd;");

    QGridLayout* main_layout = new QGridLayout;
    main_layout->setSpacing(20);
    main_layout->setMargin(20);

    /// Add sliders
    QLabel *label_x = new QLabel("X coordinates :");
    main_layout->addWidget(label_x,0,0);
    // Création du slider
    QSlider* slider_x = new QSlider(Qt::Horizontal);
    slider_x->setRange(-60, 60); // Plage de valeurs du slider
    slider_x->setSingleStep(1); // Pas d'incrémentation du slider
    main_layout->addWidget(slider_x,0,1);
    // Création du spin box pour afficher la valeur flottante x
    QDoubleSpinBox* spinBox_x = new QDoubleSpinBox;
    spinBox_x->setMaximum(60);
    spinBox_x->setMinimum(-60);
    spinBox_x->setSingleStep(1);
    main_layout->addWidget(spinBox_x,0,2);
    // Lier le slider et le spin box
    QObject::connect(slider_x, &QSlider::valueChanged, spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_x, &QSlider::setValue);


    QLabel *label_y = new QLabel("Y coordinates :");
    main_layout->addWidget(label_y,1,0);
    // Création du slider
    QSlider* slider_y = new QSlider(Qt::Horizontal);
    slider_y->setRange(20, 70); // Plage de valeurs du slider
    slider_y->setSingleStep(1); // Pas d'incrémentation du slider
    main_layout->addWidget(slider_y,1,1);
    // Création du spin box pour afficher la valeur flottante x
    QDoubleSpinBox* spinBox_y = new QDoubleSpinBox;
    spinBox_y->setMaximum(70);
    spinBox_y->setMinimum(20);
    main_layout->addWidget(spinBox_y,1,2);
    // Lier le slider et le spin box
    QObject::connect(slider_y, &QSlider::valueChanged, spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_y, &QSlider::setValue);


    QLabel *label_z = new QLabel("Z coordinates :");
    main_layout->addWidget(label_z,2,0);
    // Création du slider
    QSlider* slider_z = new QSlider(Qt::Horizontal);
    slider_z->setRange(10, 70); // Plage de valeurs du slider
    slider_z->setSingleStep(1); // Pas d'incrémentation du slider
    main_layout->addWidget(slider_z,2,1);
    // Création du spin box pour afficher la valeur flottante x
    QDoubleSpinBox* spinBox_z = new QDoubleSpinBox;
    spinBox_z->setMaximum(70);
    spinBox_z->setMinimum(10);
    main_layout->addWidget(spinBox_z,2,2);
    // Lier le slider et le spin box
    QObject::connect(slider_z, &QSlider::valueChanged, spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_z, &QSlider::setValue);

    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        x = newValue/100;
    });
    QObject::connect(spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        y = newValue/100;
    });
    QObject::connect(spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        z = newValue/100;
    });

    slider_x->setValue(20);
    slider_y->setValue(30);
    slider_z->setValue(60);

    // Création du bouton pour le switch
    QPushButton* switchButton = new QPushButton(this);
    switchButton->setCheckable(true);  // Permet de sélectionner/désélectionner le bouton
    switchButton->setChecked(false);  // Définit l'état initial du switch (désélectionné)

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

    main_layout->addWidget(switchButton,1,3);

    main_widget->setLayout(main_layout);
    setCentralWidget(main_widget);
}

MainGUI::~MainGUI()
{
}

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}


int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto ros2_node = std::make_shared<Objective_node>();
    auto gui_app = std::make_shared<MainGUI>(ros2_node);

    app.processEvents();
    gui_app->show();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);

    while (rclcpp::ok())
    {   
        if (ros2_node->manual_control){
            ros2_node->update_coords(gui_app->x,gui_app->y,gui_app->z);
        }
        exec.spin_some();
        app.processEvents();
    }
    signal(SIGINT, siginthandler);

    exec.remove_node(ros2_node);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
