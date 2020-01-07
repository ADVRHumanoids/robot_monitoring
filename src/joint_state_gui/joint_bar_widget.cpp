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

void JointBarWidget::setSafe(bool force)
{
    if(_blinker.blinking()) return;

    if(_state != 0)
    {
        printf("Started blinker..\n");
        _blinker.blink(10);
    }

    _state = 0;

    setColor(Qt::green);
}

void JointBarWidget::setDanger(bool force)
{
    if(_state == 0)
    {
        _blinker.stop();
    }

    setColor(Qt::red);

    _state = 1;

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

JointBarWidget::Blinker::Blinker(JointBarWidget * parent):
    _blinks(0),
    _state(0),
    _parent(parent)
{
    _timer.setInterval(500);
    _timer.setSingleShot(false);

    connect(&_timer, &QTimer::timeout,
            this, &Blinker::on_timeout);

}

void JointBarWidget::Blinker::stop()
{
    _blinks = 0;
    _timer.stop();
}

void JointBarWidget::Blinker::on_timeout()
{
    if(_blinks == 0)
    {
        _parent->setColor(Qt::green);
        _timer.stop();
    }

    printf("Blinking..\n");

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

void JointBarWidget::Blinker::blink(int nblinks)
{
    if(_blinks > 0) return;

    _blinks = nblinks;
    _state = 1;
    _timer.start();
}

bool JointBarWidget::Blinker::blinking() const
{
    return _blinks > 0;
}
