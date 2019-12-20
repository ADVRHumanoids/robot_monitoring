#include "joint_bar_widget.h"
#include "circle_widget.h"

#include <iostream>

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>

void joint_bar_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {



QWidget * LoadUiFile(QWidget * parent)
{

    joint_bar_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/joint_bar_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}

JointBarWidget::JointBarWidget(const QString& jname, QWidget *parent) : QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    layout->setMargin(3);
    setLayout(layout);

    _bar = findChild<QProgressBar *>("ValueBar");
    _bar->installEventFilter(this);
    _jname = findChild<QLabel *>("JointLabel");
    _jname->installEventFilter(this);
    _jname->setText(jname);

    _on_double_click = [](){};
}

void JointBarWidget::setOnBarDoubleClick(std::function<void ()> func)
{
    _on_double_click = func;
}

void JointBarWidget::setRange(double min, double max)
{
    _bar->setRange(min, max);
}

void JointBarWidget::setValue(double x)
{
    _bar->setValue(x);
    _bar->setFormat(QString("%1").arg(x, 5,'f',1));
}

void JointBarWidget::setValue(double xbar, double xtext)
{
    _bar->setValue(xbar);
    _bar->setFormat(QString("%1").arg(xtext, 5,'f',1));
}

void JointBarWidget::setStatus(QString status)
{
//    _status->setText(status);
}

void JointBarWidget::setSafe()
{   auto frame = findChild<QFrame *>("StatusFrame");
    QPalette pal;
    pal.setColor(QPalette::Background, Qt::green);
    frame->setAutoFillBackground(true);
    frame->setPalette(pal);
}

void JointBarWidget::setDanger()
{
    auto frame = findChild<QFrame *>("StatusFrame");
    QPalette pal;
    pal.setColor(QPalette::Background, Qt::red);
    frame->setAutoFillBackground(true);
    frame->setPalette(pal);

}

void JointBarWidget::setActive()
{
    _jname->setStyleSheet("font-weight: bold");
}

void JointBarWidget::setInactive()
{
    _jname->setStyleSheet("font-weight: normal");
}

void JointBarWidget::handleMouseDoubleClickEvent(QMouseEvent * ev)
{
    _on_double_click();
}

bool JointBarWidget::eventFilter(QObject * obj, QEvent * event)
{
    if (event->type() == QEvent::MouseButtonDblClick)
    {
        QMouseEvent * mouse_event = static_cast<QMouseEvent *>(event);
        if(mouse_event->button() == Qt::MouseButton::LeftButton)
        {
            handleMouseDoubleClickEvent(mouse_event);
        }

        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}
