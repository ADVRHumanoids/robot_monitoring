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

BarPlotWidget::BarPlotWidget(std::vector<std::string> jnames, QWidget *parent) : QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);

    auto bars_layout_left = findChild<QVBoxLayout *>("BarsLayoutLeft");
    auto bars_layout_right = findChild<QVBoxLayout *>("BarsLayoutRight");

    for(int i = 0; i < jnames.size(); i++)
    {

        auto wid = new JointBarWidget(QString::fromStdString(jnames[i]),
                                      this);

        if(i < jnames.size() / 2)
        {
            bars_layout_left->addWidget(wid);
        }
        else
        {
            bars_layout_right->addWidget(wid);
        }

        wid_map[jnames[i]] = wid;

    }


    _fieldtype_combobox = findChild<QComboBox *>("SelectFieldComboBox");
    _fieldtype_combobox->addItem("Link position");
    _fieldtype_combobox->addItem("Motor position");
    _fieldtype_combobox->addItem("Link velocity");
    _fieldtype_combobox->addItem("Motor velocity");
    _fieldtype_combobox->addItem("Torque");
    _fieldtype_combobox->addItem("Stiffness");
    _fieldtype_combobox->addItem("Damping");
    _fieldtype_combobox->addItem("Temperature");
    _fieldtype_combobox->addItem("Current");
    _fieldtype_combobox->addItem("Torque tracking error");
}

void BarPlotWidget::setOnJointClicked(std::function<void (std::string)> f)
{
    for(auto pair : wid_map)
    {
        pair.second->setOnBarDoubleClick(std::bind(f, pair.first));
    }
}

std::string BarPlotWidget::getFieldType() const
{
    return _fieldtype_combobox->currentText().toStdString();
}
