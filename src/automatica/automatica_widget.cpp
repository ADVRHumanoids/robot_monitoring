#include "automatica_widget.h"

#include <QLabel>

#include <QVBoxLayout>
#include <QtUiTools>

#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Dense>
#include <tf2/buffer_core.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace XBot::Ui;

bool AutomaticaWidget::init(CustomQtWidget::Args& args)
{
    std::cout << "Init Automatica GUI" << std::endl;
    isFollowingMe = isGravity = isConda = isPayloadCompensationOn = isPayloadEstimationOn = 1;

    condaBtn = new QPushButton("Condah");
    clientMap["Conda"] = nh.serviceClient<std_srvs::SetBool>("/xbotcore/cartesio_imp/switch");
    gravityBtn = new QPushButton("Gravity");
    clientMap["Gravity"] = nh.serviceClient<std_srvs::SetBool>("/cartesio/gcomp_switch");
    followMeBtn = new QPushButton("Follow Me");
    clientMap["FollowMe"] = nh.serviceClient<std_srvs::SetBool>("/xbotcore/force_follow_me/switch");
    payloadCompensationBtn = new QPushButton("Payload Compensation");
    clientMap["payloadCompensation"] = nh.serviceClient<std_srvs::SetBool>("/cartesio/payload_switch");

    payloadEstimationBtn = new QPushButton("Payload Estimation");
    clientMap["payloadEstimation"] = nh.serviceClient<std_srvs::SetBool>("/cartesio/payload_acquire");

    auto l = new QVBoxLayout;

    l->addWidget(condaBtn);
    l->addWidget(gravityBtn);
    l->addWidget(followMeBtn);
    l->addWidget(payloadCompensationBtn);
    l->addWidget(payloadEstimationBtn);
    


    setLayout(l);

    connect(condaBtn, SIGNAL( clicked() ), this, SLOT(condaClb()) );
    connect(gravityBtn, SIGNAL( clicked() ), this, SLOT(gravityClb()) );
    connect(followMeBtn, SIGNAL( clicked() ), this, SLOT(followMeClb()) );
    connect(payloadCompensationBtn, SIGNAL( clicked() ), this, SLOT(payloadCompensationClb()) );
    connect(payloadEstimationBtn, SIGNAL( clicked() ), this, SLOT(payloadEstimationClb()) );


    return true;
}

QString AutomaticaWidget::name()
{
    return "Automatica";
}

void AutomaticaWidget::update()
{
    CustomQtWidget::update();
}

bool AutomaticaWidget::loadConfig(const YAML::Node &cfg)
{
    return true;
}

bool AutomaticaWidget::saveConfig(YAML::Node &cfg)
{
    return true;
}

AutomaticaWidget::~AutomaticaWidget()
{

}

void AutomaticaWidget::condaClb()
{
    if (isConda == 1 )
    {
        isConda*=-1;
        condaBtn-> setStyleSheet(QString("QPushButton {background-color: green;}"));
        std_srvs::SetBool flag;
        flag.request.data = true;
        clientMap["Conda"].call(flag);
    }
    else
    {
        isConda*=-1;
        condaBtn-> setStyleSheet("");
        
        std_srvs::SetBool flag;
        flag.request.data = false;
        clientMap["Conda"].call(flag);
    }
}

void AutomaticaWidget::gravityClb()
{
    if (isGravity == 1)
    {
        isGravity*=-1;
        gravityBtn-> setStyleSheet(QString("QPushButton {background-color: green;}"));
        std_srvs::SetBool flag;
        flag.request.data = true;
        clientMap["Gravity"].call(flag);
    }
    else
    {
        isGravity*=-1;
        gravityBtn-> setStyleSheet("");
        
        std_srvs::SetBool flag;
        flag.request.data = false;
        clientMap["Gravity"].call(flag);
    }
}

void AutomaticaWidget::followMeClb()
{
    if (isFollowingMe == 1)
    {
        isFollowingMe*=-1;
        followMeBtn-> setStyleSheet(QString("QPushButton {background-color: green;}"));
        std_srvs::SetBool flag;
        flag.request.data = true;
        clientMap["FollowMe"].call(flag);
    }
    else
    {
        isFollowingMe*=-1;
        followMeBtn-> setStyleSheet("");
        
        std_srvs::SetBool flag;
        flag.request.data = false;
        clientMap["FollowMe"].call(flag);
    }
}


void AutomaticaWidget::payloadCompensationClb()
{
    if (isPayloadCompensationOn == 1)
    {
        isPayloadCompensationOn*=-1;
        payloadCompensationBtn-> setStyleSheet(QString("QPushButton {background-color: green;}"));
        std_srvs::SetBool flag;
        flag.request.data = true;
        clientMap["payloadCompensation"].call(flag);
    }
    else
    {
        isPayloadCompensationOn*=-1;
        payloadCompensationBtn-> setStyleSheet("");
        
        std_srvs::SetBool flag;
        flag.request.data = false;
        clientMap["payloadCompensation"].call(flag);
    }
}

void AutomaticaWidget::payloadEstimationClb()
{
    if (isPayloadEstimationOn == 1)
    {
        isPayloadEstimationOn*=-1;
        payloadEstimationBtn-> setStyleSheet(QString("QPushButton {background-color: green;}"));
        std_srvs::SetBool flag;
        flag.request.data = true;
        clientMap["payloadEstimation"].call(flag);
    }
    else
    {
        isPayloadEstimationOn*=-1;
        payloadEstimationBtn-> setStyleSheet("");
        
        std_srvs::SetBool flag;
        flag.request.data = false;
        clientMap["payloadEstimation"].call(flag);
    }
}
