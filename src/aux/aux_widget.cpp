#include "aux_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <xbot_msgs/SetAuxFields.h>

using namespace XBot::Ui;

void aux_widget_qrc_init()
{
    Q_INIT_RESOURCE(aux_ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    aux_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/aux.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}



AuxWidget::AuxWidget()
{

}

bool AuxWidget::init(CustomQtWidget::Args& args)
{
    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    auto auxFieldsLayout = findChild<QVBoxLayout*>("auxFieldsLayout");
    auto applyAuxFieldsBtn = findChild<QPushButton*>("applyAuxFieldsBtn");

    // fill aux fields (for now, hardcoded)
    std::vector<QString> aux_fields = {
        "rtt",
        "pos_ref_fb",
        "iq_ref_fb",
        "iq_out_fb",
        "id_ref_fb",
        "id_out_fb",
        "torque_no_average",
        "torque_no_calibrated",
        "board_temp_fb",
        "motor_temp_fb",
        "i_batt_fb",
        "motor_vel_filt",
        "motor_encoder",
        "link_encoder",
        "deflection_encoder",
        "position_ref_filtered",
        "motor_vel_no_filt",
        "motor_enc_warn",
        "motor_enc_err",
        "link_enc_warn",
        "link_enc_err",
        "defl_enc_warn",
        "defl_enc_err"
    };

    // create checkboxes
    for(auto af : aux_fields)
    {
        auto cb = new QCheckBox;
        cb->setText(af);
        auxFieldsLayout->addWidget(cb);
        _aux_check_box.push_back(cb);
    }

    // apply service
    connect(applyAuxFieldsBtn, &QPushButton::released,
            this, &AuxWidget::on_btn_pressed);

    // connect ros service
    _set_aux_srv = _nh.serviceClient<xbot_msgs::SetAuxFields>("xbotcore/set_aux_fields");

    return true;
}

void AuxWidget::update()
{
}

AuxWidget::~AuxWidget()
{

}

void AuxWidget::on_btn_pressed()
{
    bool exists = _set_aux_srv.waitForExistence(ros::Duration(1.0));

    auto qt_srv_name = QString::fromStdString(
                _nh.resolveName(_set_aux_srv.getService())
                );

    if(!exists)
    {
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Failed to contact service '" + qt_srv_name + "'");
        messageBox.setFixedSize(500, 200);
        messageBox.exec();
        return;
    }

    xbot_msgs::SetAuxFields srv;

    for(auto cb : _aux_check_box)
    {
        if(cb->isChecked())
        {
            srv.request.aux_types.push_back(cb->text().toStdString());
        }
    }

    if(!_set_aux_srv.call(srv) || !srv.response.success)
    {
        auto qt_response_msg = QString::fromStdString(srv.response.message);
        QMessageBox messageBox;
        messageBox.critical(0,
                            "Error",
                            "Failed to contact service '" + qt_srv_name + "' \n" +
                            "Service answered '" + qt_response_msg + "'");
        messageBox.setFixedSize(500, 200);
        messageBox.exec();
        return;
    }
}


QString AuxWidget::name()
{
    return "EcatAux";
}
