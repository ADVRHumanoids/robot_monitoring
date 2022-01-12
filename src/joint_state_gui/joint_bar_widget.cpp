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

using namespace std::chrono_literals;

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

JointBarWidget::JointBarWidget(const QString& jname, QWidget *parent) :
    QWidget(parent),
    _blinker(this),
    _state(0)
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

    setColor(Qt::green);

    setToolTip("Double left-click to show this joint inside the single joint view \n"
               "Double right-click to plot this signal inside the chart");
}

void JointBarWidget::setRange(double min, double max)
{
    _bar->setRange(min*1000, max*1000);
}

void JointBarWidget::setValue(double x)
{
    _bar->setValue(x*1000);
    _bar->setFormat(QString("%1").arg(x, 5,'f',1));
    _value_timeout = time_point::clock::now() + 300ms;
}

void JointBarWidget::setValue(double xbar, double xtext)
{
    _bar->setValue(xbar*1000);
    _bar->setFormat(QString("%1").arg(xtext, 5,'f',1));
    _value_timeout = time_point::clock::now() + 300ms;
}

void JointBarWidget::setStatus(QString status)
{
    auto frame = findChild<QFrame *>("StatusFrame");
    frame->setToolTip(status);
}

void JointBarWidget::setSafe(bool force)
{
    if(_blinker.blinking()) return;

    if(_state != 0)
    {
        _blinker.blink(10);
    }

    _state = 0;

    setColor(Qt::green);
}

void JointBarWidget::setDanger(bool force)
{
    _blinker.stop();

    setColor(Qt::red);

    _state = 1;

    _last_fault_time = std::chrono::high_resolution_clock::now();

}

void JointBarWidget::updateStatus()
{
    using namespace std::chrono;

    const auto fault_timeout = seconds(1);

    if(_last_fault_time.time_since_epoch().count() > 0 &&
        high_resolution_clock::now() > _last_fault_time + fault_timeout)
    {
        setSafe();

        _last_fault_time = high_resolution_clock::time_point(nanoseconds(0));
    }

    if(_value_timeout < time_point::clock::now())
    {
        _bar->setEnabled(false);
    }
    else
    {
        _bar->setEnabled(true);
    }
}

void JointBarWidget::setActive()
{
    _jname->setStyleSheet("font-weight: bold");
}

void JointBarWidget::setInactive()
{
    _jname->setStyleSheet("font-weight: normal");
}

QString JointBarWidget::getJointName() const
{
    return _jname->text();
}

void JointBarWidget::setColor(Qt::GlobalColor color)
{
    auto frame = findChild<QFrame *>("StatusFrame");
    QPalette pal;
    pal.setColor(QPalette::Background, color);
    frame->setAutoFillBackground(true);
    frame->setPalette(pal);
}

bool JointBarWidget::eventFilter(QObject * obj, QEvent * event)
{
    if (event->type() == QEvent::MouseButtonDblClick)
    {
        QMouseEvent * mouse_event = static_cast<QMouseEvent *>(event);
        if(mouse_event->button() == Qt::MouseButton::LeftButton)
        {
            emit doubleLeftClicked();
        }
        if(mouse_event->button() == Qt::MouseButton::RightButton)
        {
            emit doubleRightClicked();
        }

        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

Blinker::Blinker(JointBarWidget * parent):
    _blinks(0),
    _state(0),
    _parent(parent)
{
    _timer.setInterval(500);
    _timer.setSingleShot(false);

    connect(&_timer, &QTimer::timeout,
            this, &Blinker::on_timeout);

}

void Blinker::stop()
{
    _blinks = 0;
    _parent->setColor(Qt::green);
    _timer.stop();
}

void Blinker::on_timeout()
{
    if(_blinks == 0)
    {
        stop();
        return;
    }

    if(_state == 0)
    {
        _parent->setColor(Qt::green);
        _state = 1;
    }
    else {
        _parent->setColor(Qt::red);
        _state = 0;
    }

    _blinks--;

}

void Blinker::blink(int nblinks)
{
    if(_blinks > 0) return;

    _blinks = nblinks;
    _state = 1;
    _timer.start();
}

bool Blinker::blinking() const
{
    return _blinks > 0;
}
