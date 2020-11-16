#include "rviz_widget.h"

#include <QHBoxLayout>
#include <QContextMenuEvent>
#include <QMenu>
#include <QInputDialog>
#include <QLabel>

RvizWidget::RvizWidget(QWidget* parent)
{
    _render_panel = new rviz::RenderPanel;

    auto label = new QLabel;
    label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,
                                     QSizePolicy::Fixed));
    label->setText("Right click to show menu");

    auto l = new QVBoxLayout;
    l->addWidget(label);
    l->addWidget(_render_panel);
    setLayout(l);

    _manager = new rviz::VisualizationManager(_render_panel);
    _render_panel->initialize(_manager->getSceneManager(),
                              _manager);
    _manager->initialize();
    _manager->startUpdate();

    _robot_model = _manager->createDisplay("rviz/RobotModel",
                                           "XBot Robot",
                                           true);

    _robot_model->subProp("Robot Description")->
        setValue("/xbotcore/robot_description");

    Ogre::ColourValue colour(1, 1, 1, 0.9);
    _render_panel->setBackgroundColor(colour);

    _manager->createDisplay("rviz/Grid",
                            "Grid",
                            true);

}


bool RvizWidget::init(Args& args)
{
    return CustomQtWidget::init(args);
}

void RvizWidget::update()
{
}

QString RvizWidget::name()
{
    return "RViz";
}


void RvizWidget::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);

    // set fixed frame
    auto * set_fixed_frame = new QAction("Set fixed frame", this);

    connect(set_fixed_frame, &QAction::triggered,
            [this, event]()
            {
                bool ok;
                QString text = QInputDialog::getText(
                    this,
                    "RViz fixed frame selection",
                    "Fixed frame:",
                    QLineEdit::Normal,
                    _manager->getFixedFrame(),
                    &ok);

                if(ok && !text.isEmpty())
                {
                    _manager->setFixedFrame(text);
                }
            });

    menu.addAction(set_fixed_frame);

    // set tf prefix
    auto * set_tf_prefix = new QAction("Set tf prefix", this);

    connect(set_tf_prefix, &QAction::triggered,
            [this, event]()
            {
                bool ok;
                auto tfp = _robot_model->subProp("TF Prefix")->getValue().toString();
                QString text = QInputDialog::getText(
                    this,
                    "Robot model TF prefix selection",
                    "TF prefix:",
                    QLineEdit::Normal,
                    tfp,
                    &ok);

                if(ok)
                {
                    _robot_model->subProp("TF Prefix")->setValue(text);
                }
            });

    menu.addAction(set_tf_prefix);

    // set param name
    auto * set_param_name = new QAction("Set robot description", this);

    connect(set_param_name, &QAction::triggered,
            [this, event]()
            {
                bool ok;
                auto rd = _robot_model->subProp("Robot Description")->
                           getValue().toString();
                QString text = QInputDialog::getText(
                    this,
                    "Robot model description selection",
                    "Robot description:",
                    QLineEdit::Normal,
                    rd,
                    &ok);

                if(ok && !text.isEmpty())
                {
                    _robot_model->subProp("Robot Description")->
                        setValue(text);
                }
            });

    menu.addAction(set_param_name);

    menu.exec(event->globalPos());
}
