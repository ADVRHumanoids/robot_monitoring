#include "joint_state_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QComboBox>

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

    posref = findChild<QDoubleSpinBox *>("posref");
    motopos = findChild<QDoubleSpinBox *>("motopos");
    linkpos = findChild<QDoubleSpinBox *>("linkpos");

    velref = findChild<QDoubleSpinBox *>("velref");
    motovel = findChild<QDoubleSpinBox *>("motovel");
    linkvel = findChild<QDoubleSpinBox *>("linkvel");

    torref = findChild<QDoubleSpinBox *>("torref");
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
    current     ->setRange(-1e9, 1e9);
    mototemp    ->setRange(-1e9, 1e9);
    drivertemp  ->setRange(-1e9, 1e9);
    stiffness   ->setRange(-1e9, 1e9);
    damping     ->setRange(-1e9, 1e9);

}

void JointStateWidget::setJointName(QString jname, int jid)
{
    group->setTitle(QString("%1  (ID: %2)").arg(jname).arg(jid));
    _jname = jname;
}

void JointStateWidget::setStatus(std::string status)
{
    _fault->setText(QString::fromStdString(status));
}
