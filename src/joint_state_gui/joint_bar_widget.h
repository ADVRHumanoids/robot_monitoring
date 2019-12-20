#ifndef JOINT_BAR_WIDGET_H
#define JOINT_BAR_WIDGET_H

#include <QWidget>
#include <functional>
#include <QProgressBar>
#include <QLabel>

class JointBarWidget : public QWidget
{

public:

    explicit JointBarWidget(const QString& jname, QWidget * parent = nullptr);

    void setOnBarDoubleClick(std::function<void()> func);

    void setRange(double min, double max);
    void setValue(double x);
    void setValue(double xbar, double xtext);
    void setStatus(QString status);
    void setSafe();
    void setDanger();
    void setActive();
    void setInactive();

private:

    void handleMouseDoubleClickEvent(QMouseEvent * ev);

    bool eventFilter(QObject * obj, QEvent * ev) override;

    std::function<void(void)> _on_double_click;

    QProgressBar * _bar;
    QLabel * _jname;
    QLabel * _status;



};

#endif // JOINT_BAR_WIDGET_H
