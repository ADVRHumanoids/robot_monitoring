#include "joint_state_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QComboBox>
#include <QPushButton>

void joint_state_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {


QWidget * LoadUiFile(QWidget * parent)
{
    joint_state_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/joint_state_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}


JointStateWidget::JointStateWidget(QWidget * parent):
    QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);
    setMinimumWidth(400);

    posref = findChild<QDoubleSpinBox *>("posref");
    motopos = findChild<QDoubleSpinBox *>("motopos");
    linkpos = findChild<QDoubleSpinBox *>("linkpos");

    velref = findChild<QDoubleSpinBox *>("velref");
    motovel = findChild<QDoubleSpinBox *>("motovel");
    linkvel = findChild<QDoubleSpinBox *>("linkvel");

    torref = findChild<QDoubleSpinBox *>("torref");
    torref_imp = findChild<QDoubleSpinBox *>("torref_imp");
    tor = findChild<QDoubleSpinBox *>("tor");

    current = findChild<QDoubleSpinBox *>("current");

    mototemp = findChild<QDoubleSpinBox *>("mototemp");
    drivertemp = findChild<QDoubleSpinBox *>("drivertemp");

    stiffness = findChild<QDoubleSpinBox *>("stiffness");
    damping = findChild<QDoubleSpinBox *>("damping");

    group = findChild<QGroupBox *>("JointStateGroup");

    _fault = findChild<QLabel *>("faulttext");

    posref      ->setRange(-1e9, 1e9);
    motopos     ->setRange(-1e9, 1e9);
    linkpos     ->setRange(-1e9, 1e9);
    velref      ->setRange(-1e9, 1e9);
    motovel     ->setRange(-1e9, 1e9);
    linkvel     ->setRange(-1e9, 1e9);
    torref      ->setRange(-1e9, 1e9);
    tor         ->setRange(-1e9, 1e9);
    torref_imp  ->setRange(-1e9, 1e9);
    current     ->setRange(-1e9, 1e9);
    mototemp    ->setRange(-1e9, 1e9);
    drivertemp  ->setRange(-1e9, 1e9);
    stiffness   ->setRange(-1e9, 1e9);
    damping     ->setRange(-1e9, 1e9);

    auto plot_link_pos = findChild<QPushButton *>("plotLinkPos");
    connect(plot_link_pos, &QPushButton::released,
            [this](){ emit plotAdded("link_pos");});

    auto plot_ref_pos = findChild<QPushButton *>("plotPosRef");
    connect(plot_ref_pos, &QPushButton::released,
            [this](){ emit plotAdded("pos_ref");});

    auto plot_moto_pos = findChild<QPushButton *>("plotMotorPos");
    connect(plot_moto_pos, &QPushButton::released,
            [this](){ emit plotAdded("motor_pos");});

    auto plot_link_vel = findChild<QPushButton *>("plotLinkVel");
    connect(plot_link_vel, &QPushButton::released,
            [this](){ emit plotAdded("link_vel");});

    auto plot_ref_vel = findChild<QPushButton *>("plotVelRef");
    connect(plot_ref_vel, &QPushButton::released,
            [this](){ emit plotAdded("vel_ref");});

    auto plot_moto_vel = findChild<QPushButton *>("plotMotorVel");
    connect(plot_moto_vel, &QPushButton::released,
            [this](){ emit plotAdded("motor_vel");});

    auto plot_tau_imp = findChild<QPushButton *>("plotTauImp");
    connect(plot_tau_imp, &QPushButton::released,
            [this](){ emit plotAdded("torque_imp");});

    auto plot_tau_ffwd = findChild<QPushButton *>("plotTauFfwd");
    connect(plot_tau_ffwd, &QPushButton::released,
            [this](){ emit plotAdded("torque_ffwd");});

    auto plot_tau = findChild<QPushButton *>("plotTau");
    connect(plot_tau, &QPushButton::released,
            [this](){ emit plotAdded("torque");});

    auto plot_temp_motor = findChild<QPushButton *>("plotMotorTemp");
    connect(plot_temp_motor, &QPushButton::released,
            [this](){ emit plotAdded("motor_temp");});

    auto plot_driver_motor = findChild<QPushButton *>("plotDriverTemp");
    connect(plot_driver_motor, &QPushButton::released,
            [this](){ emit plotAdded("driver_temp");});




}

void JointStateWidget::setJointName(QString jname, int jid)
{
    group->setTitle(QString("%1  (ID: %2)").arg(jname).arg(jid));
    _jname = jname;
}

void JointStateWidget::setStatus(std::string status)
{
    _fault->setText(QString::fromStdString(status));

    if(status != "Ok")
    {
        _fault->setStyleSheet("color: black");
        _last_fault_time = std::chrono::high_resolution_clock::now();
    }

}

void JointStateWidget::updateStatus()
{
    using namespace std::chrono;

    const auto fault_timeout = seconds(1);

    if(_last_fault_time.time_since_epoch().count() > 0 &&
        high_resolution_clock::now() > _last_fault_time + fault_timeout)
    {
        _fault->setStyleSheet("color: grey");

        _last_fault_time = high_resolution_clock::time_point(nanoseconds(0));
    }
}
