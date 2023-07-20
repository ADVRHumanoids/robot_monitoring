#include "bar_plot_widget.h"
#include "joint_bar_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QComboBox>

void bar_plot_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {


QWidget * LoadUiFile(QWidget * parent)
{
    bar_plot_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/bar_plot_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}

BarPlotWidget::BarPlotWidget(std::vector<std::string> jnames,
                             QWidget *parent) :
    QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);

    auto bars_layout_left = findChild<QVBoxLayout *>("BarsLayoutLeft");
    auto bars_layout_center = findChild<QVBoxLayout *>("BarsLayoutCenter");
    auto bars_layout_right = findChild<QVBoxLayout *>("BarsLayoutRight");

    bars_layout_left->setSpacing(2);
    bars_layout_center->setSpacing(2);
    bars_layout_right->setSpacing(2);

    for(int i = 0; i < jnames.size(); i++)
    {

        auto wid = new JointBarWidget(QString::fromStdString(jnames[i]),
                                      this);

        if(i < jnames.size() / 3 || jnames.size() < 8)
        {
            bars_layout_left->addWidget(wid);
        }
        else if(i < jnames.size() / 3 * 2)
        {
            bars_layout_center->addWidget(wid);
        }
        else
        {
            bars_layout_right->addWidget(wid);
        }

        wid_map[jnames[i]] = wid;

    }


    _fieldtype_combobox = findChild<QComboBox *>("SelectFieldComboBox");
    _fieldtype_combobox->addItem("link position");
    _fieldtype_combobox->addItem("motor position");
    _fieldtype_combobox->addItem("link velocity");
    _fieldtype_combobox->addItem("motor velocity");
    _fieldtype_combobox->addItem("torque");
    _fieldtype_combobox->addItem("stiffness");
    _fieldtype_combobox->addItem("damping");
    _fieldtype_combobox->addItem("temperature");
    _fieldtype_combobox->addItem("torque tracking error");
    _fieldtype_combobox->addItem("position tracking error");


}

void BarPlotWidget::setOnJointClicked(std::function<void (std::string)> f)
{
    for(auto pair : wid_map)
    {
//        pair.second->setOnBarDoubleClick(std::bind(f, pair.first));
    }
}

std::string BarPlotWidget::getFieldType() const
{
    return _fieldtype_combobox->currentText().toStdString();
}

std::string BarPlotWidget::getFieldShortType() const
{
    auto type = getFieldType();

    // aux case
    if(type.length() > 4 &&
            type.substr(0, 4) == "aux/")
    {
        return type;
    }

    // non-aux fields
    std::map<std::string, std::string> type_to_short_type;
    type_to_short_type["link position" ] = "link_pos";
    type_to_short_type["motor position"] = "motor_pos";
    type_to_short_type["link velocity" ] = "link_vel";
    type_to_short_type["motor velocity"] = "motor_vel";
    type_to_short_type["torque"        ] = "torque";
    type_to_short_type["stiffness"     ] = "k";
    type_to_short_type["damping"       ] = "d";

    if(type_to_short_type.count(type) > 0)
    {
        return type_to_short_type.at(type);
    }

    return "unsupported";
}

void BarPlotWidget::addAuxType(std::string aux_type)
{
    QString qt_aux_type = "aux/" + QString::fromStdString(aux_type);

    if(_fieldtype_combobox->findText(qt_aux_type) == -1)
    {
        int nitems = _fieldtype_combobox->count();
        _fieldtype_combobox->insertItem(nitems, qt_aux_type);
    }
}
