#include "xbot2_plugin_wid.h"


#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>

void xbot2_plugin_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{

    xbot2_plugin_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/xbot2_plugin.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}

XBot2PluginWidget::XBot2PluginWidget(QString name,
                                     QWidget * parent) :
    QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto lay = new QVBoxLayout(this);
    lay->addWidget(ui);
    lay->setMargin(0);
    setLayout(lay);

    _pluginLabel = findChild<QLabel*>("pluginLabel");
    _startStopBtn = findChild<QPushButton*>("startStopBtn");
    _abortBtn = findChild<QPushButton*>("abortBtn");
    _loadBar = findChild<QProgressBar*>("loadBar");
    _statusLine = findChild<QLineEdit*>("statusLine");

    _pluginLabel->setText(name);

    _btnStarts = true;

    connect(_startStopBtn, &QPushButton::released,
            [this]()
            {
                emit startStopPressed(_btnStarts);
            });

    connect(_abortBtn, &QPushButton::released,
            [this]()
            {
                emit abortPressed();
            });
}

void XBot2PluginWidget::setLoad(double load)
{
    _loadBar->setValue(load*100);
}

void XBot2PluginWidget::setStatus(QString status)
{
    _statusLine->setText(status);

    if(status == "Initialized")
    {
        _btnStarts = true;
    }

    if(status == "Running")
    {
        _btnStarts = false;
        _pluginLabel->setStyleSheet(
            "font-size: 12pt;"
            "font-weight: bold; "
            "border-radius: 4px;"
            "padding-left: 2px;"
            "background-color: #99ff99;");
    }
    else if(status == "Aborted")
    {
        _pluginLabel->setStyleSheet(
            "font-size: 12pt;"
            "border-radius: 4px;"
            "font-weight: bold; "
            "border-radius: 4px;"
            "padding-left: 2px;"
            "background-color: #ff9999;");

        _startStopBtn->setEnabled(false);
    }
    else
    {
        _pluginLabel->setStyleSheet(
            "font-size: 12pt;"
            "border-radius: 4px;"
            "padding-left: 2px;"
            "background-color: #dddddd;");
    }

    if(_btnStarts)
    {
        _startStopBtn->setText("Start");
    }
    else {
        _startStopBtn->setText("Stop");
    }
}
