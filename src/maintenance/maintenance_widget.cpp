#include "maintenance_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <xbot_msgs/SetAuxFields.h>

#include <unistd.h>
#include <csignal>


using namespace XBot::Ui;

void maintenance_widget_qrc_init()
{
    Q_INIT_RESOURCE(maintenance_ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    maintenance_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/maintenance.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}



MaintenanceWidget::MaintenanceWidget():
    user_host("user@10.10.10.10")
{

}

bool MaintenanceWidget::init(CustomQtWidget::Args& args)
{
    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    auto motionComboBox = findChild<QComboBox*>("motionComboBox");
    auto consoleText = findChild<QTextEdit*>("consoleText");
    auto startStopBtn = findChild<QPushButton*>("startStopBtn");
    auto clearBtn = findChild<QPushButton*>("clearBtn");
    auto configBtn = findChild<QPushButton*>("configBtn");

    motionComboBox->addItem("Stress test (lower body)");
    motionComboBox->addItem("Stress test (upper body)");
    motionComboBox->addItem("Poses v2 demo");

    QStringList cmd_list;
    cmd_list.append("rosrun centauro_test stress_test_lb.py");
    cmd_list.append("rosrun centauro_test stress_test_ub.py");
    cmd_list.append("rosrun centauro_cartesio poses2.py");

    QStringList descr_list;
    descr_list.append("warning: make sure the robot is in homing position, "
                      "and it has enough clearance to fully stretch the legs; \n");
    descr_list.append("warning: make sure the robot is in homing position, "
                      "and it has enough clearance to fully stretch the legs; \n");
    descr_list.append("warning: make sure the robot is in homing position, "
                      "and it is fully disconnected from the crane; \n");

    consoleText->insertPlainText("select a motion, then press Start \n");
    consoleText->insertPlainText(descr_list[0]);

    connect(startStopBtn, &QPushButton::released,
            [this, motionComboBox, consoleText, startStopBtn, cmd_list]()
    {
        if(startStopBtn->text() == "Start")
        {
            consoleText->insertPlainText(
                        QString("will ssh into %1, press Configure to change remote \n")
                        .arg(user_host));

            QStringList args;
            args.append("-tt");
            args.append(user_host);
            args.append(cmd_list[motionComboBox->currentIndex()]);

            ssh.start("/usr/bin/ssh", args);

            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText(
                        QString("spawned new process: ssh %1 %2 %3 \n").arg(
                            args[0], args[1], args[2]));
        }
        else if(startStopBtn->text() == "Stop")
        {
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("sending SIGINT over ssh \n");
            consoleText->insertPlainText("press Abort to send SIGQUIT \n");

            ssh.putChar('\x03');
            ssh.putChar('\n');

            startStopBtn->setText("Abort");
        }
        else if(startStopBtn->text() == "Abort")
        {
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("sending SIGQUIT over ssh \n");
            consoleText->insertPlainText("press Close SSH to terminate the ssh process \n");

            ssh.putChar('\x1c');
            ssh.putChar('\n');

            startStopBtn->setText("Close SSH");
        }
        else if(startStopBtn->text() == "Close SSH")
        {
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("terminating ssh process \n");

            ssh.terminate();
        }
    });

    // connect ssh process
    connect(&ssh, &QProcess::stateChanged,
            [this, motionComboBox, consoleText, startStopBtn]
            (QProcess::ProcessState state)
    {
        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText("process switched to new state: ");

        switch(state)
        {
        case QProcess::ProcessState::Starting:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("Starting \n");
            startStopBtn->setText("Stop");
            break;

        case QProcess::ProcessState::Running:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("Running \n");
            startStopBtn->setText("Stop");
            break;

        case QProcess::ProcessState::NotRunning:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("NotRunning \n");
            startStopBtn->setText("Start");
            break;
        }

    });

    connect(&ssh, &QProcess::readyRead,
            [this, motionComboBox, consoleText, startStopBtn]()
    {
        auto output = ssh.readAll();

        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText(
                    QString(output).replace(QRegularExpression("\033\\[\\d+m"), ""));

        auto err = ssh.readAllStandardError();
        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText(
                    QString(err).replace(QRegularExpression("\033\\[\\d+m"), ""));

    });

    connect(&ssh, &QProcess::errorOccurred,
            [this, motionComboBox, consoleText, startStopBtn]
            (QProcess::ProcessError error)
    {
        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText("process error: ");

        switch(error)
        {
        case QProcess::ProcessError::Crashed:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("Crashed \n");
            break;

        case QProcess::ProcessError::FailedToStart:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("FailedToStart \n");
            break;

        case QProcess::ProcessError::ReadError:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("ReadError \n");
            break;

        case QProcess::ProcessError::Timedout:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("Timedout \n");
            break;

        case QProcess::ProcessError::UnknownError:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("UnknownError \n");
            break;

        case QProcess::ProcessError::WriteError:
            consoleText->moveCursor(QTextCursor::End);
            consoleText->insertPlainText("WriteError \n");
            break;
        }

    });

    connect(&ssh, (void (QProcess::*)(int,QProcess::ExitStatus))&QProcess::finished,
            [this, motionComboBox, consoleText, startStopBtn]
            (int exitcode, QProcess::ExitStatus status)
    {
        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText(
                    QString("process exited with code: %1 \n").arg(exitcode));

        auto output = ssh.readAll();

        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText(
                    QString(output).replace(QRegularExpression("\033\\[\\d+m"), ""));

        auto err = ssh.readAllStandardError();
        consoleText->moveCursor(QTextCursor::End);
        consoleText->insertPlainText(
                    QString(err).replace(QRegularExpression("\033\\[\\d+m"), ""));
    });

    // connect clear
    connect(clearBtn, &QPushButton::released,
            [consoleText, descr_list, motionComboBox]()
    {
        consoleText->clear();
        consoleText->insertPlainText("select a motion, then press Start \n");
        consoleText->insertPlainText(descr_list[motionComboBox->currentIndex()]);
    });

    // connect config
    connect(configBtn, &QPushButton::released,
            [this, consoleText]()
    {
        bool ok;
        auto txt = QInputDialog::getText(this, "SSH configuration",
                                         "Remote username@hostname:",
                                         QLineEdit::Normal,
                                         user_host,
                                         &ok,
                                         Qt::Sheet);
        if(ok)
        {
            user_host = txt;

            consoleText->insertPlainText(
                        QString("will ssh into %1 \n")
                        .arg(user_host));
        }
    });

    // connect combo
    connect(motionComboBox, qOverload<int>(&QComboBox::currentIndexChanged),
            [consoleText, descr_list](int i)
    {
        consoleText->insertPlainText(descr_list[i]);
    });

    return true;
}

void MaintenanceWidget::update()
{
}

MaintenanceWidget::~MaintenanceWidget()
{

}

QString MaintenanceWidget::name()
{
    return "Maintenance";
}


bool XBot::Ui::MaintenanceWidget::loadConfig(const YAML::Node &cfg)
{

    if(auto node = cfg["user_host"])
    {
        user_host = QString::fromStdString(node.as<std::string>());
    }

    return true;
}

bool XBot::Ui::MaintenanceWidget::saveConfig(YAML::Node &cfg)
{
    cfg["user_host"] = user_host.toStdString();
    return true;
}
