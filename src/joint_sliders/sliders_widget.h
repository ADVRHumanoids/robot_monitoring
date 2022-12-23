#ifndef IMPEDANCEWIDGET_H
#define IMPEDANCEWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <functional>

namespace cartesio_gui
{

class SlidersWidget : public QWidget
{


public:

    typedef std::function<void(std::string,
                               double)> SliderCallbackType;

    typedef std::function<void(std::vector<std::string>,
                               std::vector<double>)> SendCallbackType;


    explicit SlidersWidget (std::string group_name,
                             std::vector<std::string> joint_names,
                             QWidget * parent = 0);

    void setRange(std::vector<double> min,
                  std::vector<double> max);

    void setInitialValue(std::vector<double> x_0);

    void setBindEnabled(bool enabled);

    void setSliderCallback(SliderCallbackType f);

    void setSendCallback(SendCallbackType f);

    void enableSend();

    ~SlidersWidget();

private:

    struct JointWidgets
    {
        QCheckBox * lock;
        QLabel * label;
        QSlider * slider;
        QDoubleSpinBox * spinbox;

        double min, max;

        double locked_value;


        JointWidgets(QWidget * ui, int j_id);
    };

    void on_max_stiffness_changed();
    void on_slider_changed(int value, int i);
    void on_spinbox_changed(int i);
    void on_lock_checked(int state, int i);
    void on_lockall_pressed();
    void on_unlockall_pressed();

    void handle_locked_joints(int i, double value);

    QDoubleSpinBox * _max_stiffness_spinbox;
    QGroupBox * _binding_groupbox;
    std::vector<JointWidgets> _widget_vec;
    QCheckBox *_continuous_checkbox;
    QPushButton *_send_btn, *_abort_btn;
    QLineEdit *_info_text;

    std::map<std::string, double> _moved_joints;

    SliderCallbackType _sli_callback;
    SendCallbackType _send_callback;
    bool _callback_enabled;

};

}

#endif // IMPEDANCEWIDGET_H
