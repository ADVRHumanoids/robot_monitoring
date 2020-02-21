#ifndef BAR_PLOT_WIDGET_H
#define BAR_PLOT_WIDGET_H

#include <QWidget>
#include <QComboBox>
#include <unordered_map>

#include "joint_bar_widget.h"

class BarPlotWidget : public QWidget
{

public:

    explicit BarPlotWidget(std::vector<std::string> jnames, QWidget * parent = nullptr);

    void setOnJointClicked(std::function<void(std::string)> f);

    std::string getFieldType() const;
    std::string getFieldShortType() const;

    std::unordered_map<std::string, JointBarWidget *> wid_map;

private:

    QComboBox * _fieldtype_combobox;


};

#endif // BAR_PLOT_WIDGET_H
