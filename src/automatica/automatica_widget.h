#ifndef FORCE_FOLLOW_ME_WIDGET_H
#define FORCE_FOLLOW_ME_WIDGET_H

#include "robot_monitoring/custom_qt_widget.h"

#include <std_srvs/SetBool.h>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QComboBox>

#include <QTimer>
#include <QString>

#include <ros/ros.h>


class QPushButton;


namespace XBot { namespace Ui {


class AutomaticaWidget : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "automatica.CustomQtWidget.1.0.0" FILE "automatica.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

    public:

        bool init(Args&) override;

        QString name() override;

        void update() override;

        bool loadConfig(const YAML::Node &cfg);

        bool saveConfig(YAML::Node &cfg);

        ~AutomaticaWidget() override;

    protected Q_SLOTS:

        void condaClb();
        void gravityClb();
        void followMeClb();
        void payloadCompensationClb();
        void payloadEstimationClb();

    private:

        // static void setProgressBarValue(QProgressBar * pb, double value, double max);

        // ros::Publisher getCommandPublisher();

        int isGravity, isConda, isFollowingMe, isPayloadCompensationOn, isPayloadEstimationOn;

        QPushButton *followMeBtn;
        QPushButton *gravityBtn;
        QPushButton *condaBtn;

        QPushButton *payloadCompensationBtn;
        QPushButton *payloadEstimationBtn;
        



        ros::NodeHandle nh;
        std::vector<ros::Subscriber> subs;
        std::map<std::string, ros::ServiceClient> clientMap;
        QTimer * pubTimer;


};

}}

#endif // FORCE_FOLLOW_ME_WIDGET_H
