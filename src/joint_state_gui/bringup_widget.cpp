#include "bringup_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>
#include <QDir>
#include <QPushButton>
#include <QLabel>
#include <QCoreApplication>
#include <QTimer>
#include <QComboBox>

#include <iostream>

#include <std_srvs/Trigger.h>
#include <ec_srvs/GetSlaveInfo.h>
#include <xbot_msgs/StartProcess.h>
#include <xbot_msgs/StopProcess.h>
#include <xbot_msgs/GetPluginList.h>
#include <rosgraph_msgs/Log.h>

#include <yaml-cpp/yaml.h>

void bringup_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{

    bringup_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/bringup.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;

}

QString hw_type_cache;

}


BringupWidget::BringupWidget(QString hw, QWidget * parent):
    QDialog(parent),
    _worker(nullptr),
    _worker_success(false),
    _hw(hw_type_cache.isEmpty() ? hw : hw_type_cache)
{
    auto wid = LoadUiFile(this);
    auto l = new QHBoxLayout;
    l->addWidget(wid);
    setLayout(l);
    wid->setMinimumWidth(400);

    setWindowTitle("Robot bringup procedure");

    // text
    _text = findChild<QTextEdit*>("textEdit");

    // hw selection
    QList<QString> items;

    auto cli = _nh.serviceClient<xbot_msgs::GetPluginList>("xbotcore/d/get_hw_types");

    xbot_msgs::GetPluginList srv;
    writeText(">> waiting for get_hw_types service..");
    if(!cli.waitForExistence(ros::Duration(1.0)))
    {
        writeText("failed! \nget_hw_types service is offline, make sure "
                  "xbot2-launcher daemon is up and running \n");
    }
    else if(!cli.call(srv))
    {
        writeText("failed! \nget_hw_types service failed, make sure "
                  "xbot2-launcher daemon is up and running \n");
    }
    else
    {
        writeText(".ok \n");
    }

    items.append("auto");
    std::transform(srv.response.plugins.begin(),
                   srv.response.plugins.end(),
                   std::back_inserter(items),
                   QString::fromStdString);

    // add hw types to combobox
    auto hwCombo = findChild<QComboBox*>("hwCombo");
    hwCombo->insertItems(0, items);

    if(!hw_type_cache.isEmpty())
    {
        hwCombo->setCurrentText(hw_type_cache);
    }

    // close button
    auto okBtn = findChild<QPushButton*>("okBtn");
    auto okBtnClicked = [this]()
    {
        stop_worker();
        if(_worker_success)
        {
            accept();
        }
        else
        {
            reject();
        }

    };
    connect(okBtn, &QPushButton::released, okBtnClicked);

    // start button
    _startBtn = findChild<QPushButton*>("startBtn");
    connect(_startBtn, &QPushButton::released, this,
            [this]()
    {
        if(_startBtn->text() == "Start")
        {
            start_worker();
            _startBtn->setText("Abort");
        }
        else
        {
            stop_worker();
            _startBtn->setText("Start");
        }
    });

    // hw type
    auto xbot2Label = findChild<QLabel*>("xbot2Label");
    auto xbot2LabelText = xbot2Label->text();
    if(_hw.isEmpty())
    {
        xbot2Label->setText(xbot2Label->text().arg("unspecified"));
    }
    else
    {
        xbot2Label->setText(xbot2Label->text().arg("'" + _hw + "'"));
    }

    connect(hwCombo, &QComboBox::currentTextChanged,
            [this, xbot2Label, xbot2LabelText](const QString& hwtype)
    {
        _hw = hwtype;
        xbot2Label->setText(xbot2LabelText.arg("'" + _hw + "'"));
        hw_type_cache = hwtype;
    });

    // ros sub
    auto stderr_cb = [this](rosgraph_msgs::LogConstPtr msg)
    {
        writeText(">! ");
        writeText(QString::fromStdString(msg->msg));
        writeText("\n");
    };

    _nh.setCallbackQueue(&_cbq);
    _stderr_sub = _nh.subscribe<rosgraph_msgs::Log>("xbotcore/d/stderr", 100,
                                                    stderr_cb);

    // ros sub timer
    _timer = new QTimer(this);

    connect(_timer, &QTimer::timeout,
            [this]()
    {
        _cbq.callAvailable();
    });

    _timer->start(10);
}

void BringupWidget::writeText(QString text)
{
    _text->moveCursor(QTextCursor::End);
    _text->insertPlainText(text);
    _text->moveCursor(QTextCursor::End);
    std::cout << text.toStdString();
    QCoreApplication::processEvents();
}


void BringupWidget::start_worker()
{
    // worker thread
    _worker = new BringupThread;
    _worker->hw = _hw;
    _worker_success = false;

    connect(_worker, &BringupThread::finished,
            [this]()
    {
        _startBtn->setText("Start");
        _startBtn->setEnabled(false);
    });

    connect(_worker, &BringupThread::writeText,
            this, &BringupWidget::writeText);

    connect(_worker, &BringupThread::labelOk,
            this, &BringupWidget::labelOk);

    connect(_worker, &BringupThread::labelWarn,
            this, &BringupWidget::labelWarn);

    connect(_worker, &BringupThread::labelNok,
            this, &BringupWidget::labelNok);

    connect(_worker, &BringupThread::labelText,
            this, &BringupWidget::labelText);

    connect(_worker, &QThread::finished,
            _worker, &QThread::deleteLater);

    connect(_worker, &BringupThread::resultReady,
            [this](bool success)
    {
        _worker_success = success;
    });

    _worker->start();
}

void BringupWidget::stop_worker()
{
    if(_worker)
    {
        _worker->ok = false;
    }
}

void BringupWidget::labelOk(QString name)
{
    auto label = findChild<QLabel*>(name);
    label->setStyleSheet(
                "color: green;");
}

void BringupWidget::labelWarn(QString name)
{
    auto label = findChild<QLabel*>(name);
    label->setStyleSheet(
                "color: orange;");
}


void BringupWidget::labelNok(QString name)
{
    auto label = findChild<QLabel*>(name);
    label->setStyleSheet(
                "color: red; "
                "font-weight: bold");
}

void BringupWidget::labelText(QString name, QString text)
{
    auto label = findChild<QLabel*>(name);
    label->setText(text);
}


void BringupThread::bringup()
{
    bool ec = is_ecat();

    // services existance (ecat related)
    if(ec)
    {
        if(!wait_service(_ecat_start) ||
                !wait_service(_ecat_status) ||
                !wait_service(_ecat_get_slaves))
        {
            labelText("servicesOk", "Failed");
            labelNok("servicesLabel");
            labelNok("servicesOk");
            return;
        }
    }

    // xbot2 related
    if(!wait_service(_xbot_start) ||
            !wait_service(_xbot_status))
    {
        labelText("servicesOk", "Failed");
        labelNok("servicesLabel");
        labelNok("servicesOk");
        return;
    }

    labelText("servicesOk", "Ok");
    labelOk("servicesLabel");
    labelOk("servicesOk");

    // check currently running
    if(!check_status())
    {
        labelText("checkOk", "Failed");
        labelNok("checkLabel");
        labelNok("checkOk");
        return;
    }

    labelText("checkOk", "Ok");
    labelOk("checkLabel");
    labelOk("checkOk");

    // start ecat
    if(ec && !start_ecat())
    {
        labelText("ecatOk", "Failed");
        labelNok("ecatLabel");
        labelNok("ecatOk");
        return;
    }

    if(ec)
    {
        labelText("ecatOk", "Ok");
        labelOk("ecatLabel");
        labelOk("ecatOk");
    }

    // slaves descr
    int nslaves = -1;
    if(ec && !wait_slaves(nslaves))
    {
        labelText("slaveOk", "Failed");
        labelNok("slaveLabel");
        labelNok("slaveOk");
        return;
    }

    if(ec)
    {
        labelText("slaveOk", QString("Ok (%1 slaves)").arg(nslaves));

        if(nslaves < 3)
        {
            labelWarn("slaveLabel");
            labelWarn("slaveOk");
        }
        else
        {
            labelOk("slaveLabel");
            labelOk("slaveOk");
        }
    }

    // xbot2
    if(!start_xbot())
    {
        labelText("xbot2Ok", "Failed");
        labelNok("xbot2Label");
        labelNok("xbot2Ok");
        return;
    }

    labelText("xbot2Ok", "Ok");
    labelOk("xbot2Label");
    labelOk("xbot2Ok");

    resultReady(true);

    writeText(">> bringup finished successfully, press close to load the gui..");
    return;
}

bool BringupThread::wait_service(ros::ServiceClient& s)
{
    if(!ok)
    {
        writeText(">> canceled by user");
        return false;
    }

    writeText(QString(">> waiting for service '%1'..").arg(s.getService().c_str()));
    if(s.waitForExistence(ros::Duration(1.0)))
    {
        writeText("..ok \n");
    }
    else
    {
        writeText("..failed, check connection with robot pc \n");
        return false;
    }

    return true;
}

bool BringupThread::check_status()
{
    std_srvs::Trigger srv;

    if(!ok)
    {
        writeText(">> canceled by user");
        return false;
    }

    writeText(">> checking xbot2 is not running..");

    if(!_xbot_status.call(srv))
    {
        writeText("..service failed \n");
        return false;
    }

    if(srv.response.success)
    {
        writeText("..xbot2 is running, stop it first \n");
        return false;
    }

    writeText("..ok \n");

    return true;
}

bool BringupThread::start_ecat()
{
    if(!ok)
    {
        writeText(">> canceled by user");
        return false;
    }

    std_srvs::Trigger srv;
    bool ecat_running = false;

    if(is_ecat())
    {
        writeText(">> checking ecat master status..");

        if(!_ecat_status.call(srv))
        {
            writeText("..service failed \n");
            return false;
        }

        if(srv.response.success)
        {
            writeText("..got 'running' \n");
            ecat_running = true;
        }
        else
        {
            writeText("..got 'not running' \n");
        }

    }

    if(!ecat_running)
    {
        writeText(">> starting ecat master..");

        if(!_ecat_start.call(srv))
        {
            writeText("..service failed \n");
            return false;
        }

        if(!srv.response.success)
        {
            writeText("..failed \n");
            return false;
        }

        writeText("..ok \n");
    }

    return true;

}

bool BringupThread::wait_slaves(int& nslaves)
{
    ec_srvs::GetSlaveInfo srv;

    int attempts = 10;
    bool descr_ok = false;

    while(!descr_ok && attempts--)
    {
        writeText(">> querying slave description..");

        if(!_ecat_get_slaves.call(srv))
        {
            writeText("..service failed \n");
            return false;
        }

        if(!ok)
        {
            writeText("..canceled by user \n");
            return false;
        }

        if(srv.response.cmd_info.status != "FAULT")
        {
            descr_ok = true;
        }
        else
        {
            sleep(3);
            writeText(" answered: '");
            writeText(QString::fromStdString(srv.response.cmd_info.fault_info) + "' \n");
        }

    }

    if(!descr_ok)
    {
        writeText("..failed after n = 30 attempts \n");
        return false;
    }

    try
    {
        auto y = YAML::Load(srv.response.cmd_info.msg);
        nslaves = y.size();
    }
    catch(YAML::Exception& e)
    {
        writeText("..parsing failed \n");
        return false;
    }

    writeText("..ok \n");

    return true;
}

bool BringupThread::start_xbot()
{
    if(!ok)
    {
        writeText(">> canceled by user");
        return false;
    }

    xbot_msgs::StartProcess srv;

    if(!hw.isEmpty())
    {
        srv.request.args.push_back("--hw");
        srv.request.args.push_back(hw.toStdString());
    }

    if(hw == "sim" || hw == "gz")
    {
        srv.request.args.push_back("--simtime");
    }


    writeText(">> starting xbot2-core ");
    for(auto a : srv.request.args)
    {
        writeText(a.c_str());
        writeText(" ");
    }

    if(!_xbot_start.call(srv))
    {
        writeText("..service failed \n");
        return false;
    }

    if(!srv.response.success)
    {
        writeText("..failed \n");
        writeText(">> xbot2-core startup failed; see log file"
                  " at /tmp/xbot2-output \n");
        return false;
    }

    writeText("..ok \n");

    return true;
}

bool BringupThread::is_ecat() const
{
    // is 'ec' or starts with 'ec_'
    return hw == "ec" ||
            (hw.length() >= 3 && hw.startsWith("ec_"));
}

BringupThread::BringupThread()
{

    // ros services
    _ecat_start = _nh.serviceClient<std_srvs::Trigger>("ecat/d/start");
    _ecat_status = _nh.serviceClient<std_srvs::Trigger>("ecat/d/status");
    _ecat_get_slaves = _nh.serviceClient<ec_srvs::GetSlaveInfo>("ec_client/get_slaves_description");
    _xbot_start = _nh.serviceClient<xbot_msgs::StartProcess>("xbotcore/d/start");
    _xbot_status = _nh.serviceClient<std_srvs::Trigger>("xbotcore/d/status");

}

void BringupThread::run()
{
    ok = true;
    bringup();
}