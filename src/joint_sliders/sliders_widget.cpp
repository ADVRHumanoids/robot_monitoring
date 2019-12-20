#include <robot_monitoring_tools/joint_sliders/sliders_widget.h>

#include <functional>
#include <iostream>

#include <RobotInterfaceROS/ConfigFromParam.h>

inline void initSlidersResource()
{
    Q_INIT_RESOURCE(sliders_widget_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    
    initSlidersResource();
    
    QUiLoader loader;

    QFile file(":/ui/impedance_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}

int value_to_perc(double val, double min, double max)
{
    return int( (val - min)/(max - min) * 100. );
}

double perc_to_value(int perc, double min, double max)
{
    return min + (max - min)/100.0 * perc;
}

double sign(double x)
{
    return (x > 0.) - (x < 0.);
}

}

namespace cartesio_gui
{


SlidersWidget::SlidersWidget (std::string group_name, 
                             std::vector<std::string>  joint_names,
                             QWidget *parent) :
    QWidget(parent),
    _callback_enabled(true)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    /* Initialize and connect max stiffness spinbox */
    double max_stiffness = 8000.0;
    _max_stiffness_spinbox = findChild<QDoubleSpinBox *>("maxStiffnessSpinbox");
    _max_stiffness_spinbox->setMinimum(0.0);
    _max_stiffness_spinbox->setMaximum(max_stiffness);
    _max_stiffness_spinbox->setValue(2000.0);

    connect(_max_stiffness_spinbox, &QDoubleSpinBox::editingFinished,
            this, &SlidersWidget::on_max_stiffness_changed);


    /* Create array of joint sliders */
    for(int i = 0; i < 10 && i < joint_names.size(); i++)
    {
        _widget_vec.emplace_back(this, i + 1);
        
        // set label
        _widget_vec.back().label->setText(QString::fromStdString(joint_names[i]));
        
        // set intial value
        _widget_vec.back().spinbox->setValue(0.0);
        on_spinbox_changed(i);
        
        // connect slider to spinbox
        connect(_widget_vec.back().slider, &QSlider::valueChanged,
               std::bind(&SlidersWidget::on_slider_changed, this, 
                         std::placeholders::_1, i)
               );
        
        // connect spinbox to slider
        connect(_widget_vec.back().spinbox, &QDoubleSpinBox::editingFinished,
               std::bind(&SlidersWidget::on_spinbox_changed, this, i) 
               );

        
        // set locked value when a lock is checked 
        connect(_widget_vec.back().lock, &QCheckBox::stateChanged,
               std::bind(&SlidersWidget::on_lock_checked, this, 
                         std::placeholders::_1, i)
               );
    }
    
    /* Hide unused */
    for(int i = joint_names.size(); i < 10; i++)
    {
        JointWidgets w(this, i + 1);
        w.label->hide();
        w.lock->hide();
        w.slider->hide();
        w.spinbox->hide();
    }

    /* Connect lock all / unlock all buttons */
    auto * lock_all = findChild<QPushButton *>("lockallButton");
    auto * unlock_all = findChild<QPushButton *>("unlockallButton");
    
    connect(lock_all, &QPushButton::pressed,
            this, &SlidersWidget::on_lockall_pressed);
    
    connect(unlock_all, &QPushButton::pressed,
            this, &SlidersWidget::on_unlockall_pressed);
    
    /* Apply initial value for max stiffness */
    on_max_stiffness_changed();

    /* Save binding combo box  */
    _binding_groupbox = findChild<QGroupBox *>("bindGroupBox");


}

void SlidersWidget::on_max_stiffness_changed()
{
    double k_max = _max_stiffness_spinbox->value();
    
    // check that max value is > than the max slider
    double curr_max = -1;

    for(auto w : _widget_vec)
    {
        curr_max = std::max(curr_max, std::fabs(w.spinbox->value()));
    }

    k_max = std::max(k_max, curr_max);
    _max_stiffness_spinbox->setValue(k_max);
    
    // update sliders
    for(auto w : _widget_vec)
    {
        double upd_min = ::sign(w.min) * std::min(std::fabs(w.min), k_max);
        double upd_max = ::sign(w.max) * std::min(std::fabs(w.max), k_max);

        double value = std::max(std::min(w.spinbox->value(), upd_max), upd_min);
        w.spinbox->setMaximum(upd_max);
        w.spinbox->setMinimum(upd_min);
        w.spinbox->setValue(value);
        w.spinbox->setSingleStep((upd_max - upd_min) / 100.);
        w.spinbox->setDecimals(2);

        
        w.slider->blockSignals(true);
        w.slider->setValue(::value_to_perc(value, upd_min, upd_max));
        w.slider->blockSignals(false);
    }
}

SlidersWidget::JointWidgets::JointWidgets(QWidget * ui, int id):
    locked_value(-1.0),
    min(0.0), max(3000.0)
{
    lock = ui->findChild<QCheckBox *>(QString("jlock_%1").arg(id));
    label = ui->findChild<QLabel *>(QString("jname_%1").arg(id));
    slider = ui->findChild<QSlider *>(QString("jslider_%1").arg(id));
    spinbox = ui->findChild<QDoubleSpinBox *>(QString("jspinbox_%1").arg(id));

    if(!lock)
    {
        throw std::runtime_error("Unable to get lock " + std::to_string(id));
    }

    if(!spinbox)
    {
        throw std::runtime_error("Unable to get spinbox " + std::to_string(id));
    }

    if(!slider)
    {
        throw std::runtime_error("Unable to get slider " + std::to_string(id));
    }

    if(!label)
    {
        throw std::runtime_error("Unable to get label " + std::to_string(id));
    }

    
}

void SlidersWidget::handle_locked_joints(int i, double value)
{
    auto wi = _widget_vec.at(i);
    
    if(std::fabs(wi.locked_value) < 0.1)
    {
        return;
    }
    
    for(int j = 0; j < _widget_vec.size(); j++)
    {
        if(i == j)
        {
            continue;
        }
        
        auto wj = _widget_vec.at(j);
        
        // j-th slider is binded, set its value in proportion to the i-th value
        if(wj.lock->isChecked())
        {
            double value_j = wj.locked_value / wi.locked_value * value;
            wj.spinbox->setValue(value_j);
            
            if(_callback && _callback_enabled)
            {
                _callback(wj.label->text().toStdString(), value_j);
            }
            
            double k_min = wj.spinbox->minimum();
            double k_max = wj.spinbox->maximum();
            wj.slider->blockSignals(true);
            wj.slider->setValue(::value_to_perc(wj.spinbox->value(), k_min, k_max));
            wj.slider->blockSignals(false);
        }
    }
}


void SlidersWidget::on_slider_changed(int perc, int i)
{
    auto wi = _widget_vec.at(i);
    
    // compute slider value
    double k_min = wi.spinbox->minimum();
    double k_max = wi.spinbox->maximum();
    double value = ::perc_to_value(perc, k_min, k_max);
    
    // update spinbox
    wi.spinbox->setValue(value);
    
    // if i-th slider binded, change all other binded sliders
    if(wi.lock->isChecked())
    {
        handle_locked_joints(i, value);
    }

    // trigger callback
    if(_callback && _callback_enabled)
    {
        _callback(wi.label->text().toStdString(), value);
    }
}

void SlidersWidget::on_spinbox_changed(int i)
{
    auto wi = _widget_vec.at(i);
    
    double value = wi.spinbox->value();
    double k_min = wi.spinbox->minimum();
    double k_max = wi.spinbox->maximum();
    
    // update slider
    wi.slider->blockSignals(true);
    wi.slider->setValue(::value_to_perc(value, k_min, k_max));
    wi.slider->blockSignals(false);
    
    // if i-th slider binded, change all other binded sliders
    if(wi.lock->isChecked())
    {
        handle_locked_joints(i, value);
    }

    // trigger callback
    if(_callback && _callback_enabled)
    {
        _callback(wi.label->text().toStdString(), value);
    }
}

void SlidersWidget::on_lock_checked(int state, int i)
{
    if(state == Qt::Checked)
    {
        for(auto& wi : _widget_vec)
        {
            if(wi.lock->checkState() == Qt::Checked)
            {
                wi.locked_value = wi.spinbox->value();
            }
        }
    }
}

void SlidersWidget::on_lockall_pressed()
{
    for(int i = 0; i < _widget_vec.size(); i++)
    {
        _widget_vec.at(i).lock->setChecked(true);
    }
}

void SlidersWidget::on_unlockall_pressed()
{
    for(int i = 0; i < _widget_vec.size(); i++)
    {
        _widget_vec.at(i).lock->setChecked(false);
    }
}

void SlidersWidget::setRange(std::vector<double> min, std::vector<double> max)
{
    if(min.size() != _widget_vec.size())
    {
        throw std::invalid_argument("min.size() != joint num");
    }

    if(max.size() != _widget_vec.size())
    {
        throw std::invalid_argument("max.size() != joint num");
    }

    double global_max = -1.;

    for(int i = 0; i < _widget_vec.size(); i++)
    {
        _widget_vec[i].min = min[i];
        _widget_vec[i].max = max[i];

        global_max = std::max(global_max, std::fabs(min[i]));
        global_max = std::max(global_max, std::fabs(max[i]));
    }

    _max_stiffness_spinbox->setValue(global_max);
    on_max_stiffness_changed();
}

void SlidersWidget::setInitialValue(std::vector<double> x_0)
{
    _callback_enabled = false;
    
    if(x_0.size() != _widget_vec.size())
    {
        throw std::invalid_argument("x_0.size() != joint num");
    }
    
    on_unlockall_pressed();

    auto compare_abs = [](double a, double b)
    { 
        return std::fabs(a) < std::fabs(b);
    };
    
    double max_val = *(std::max_element(x_0.begin(), x_0.end(), compare_abs));
    double curr_max = _max_stiffness_spinbox->value();
    
    if(curr_max < max_val)
    {
        _max_stiffness_spinbox->setValue(max_val);
        on_max_stiffness_changed();
    }

    for(int i = 0; i < _widget_vec.size(); i++)
    {
        _widget_vec[i].spinbox->setValue(x_0[i]);
        on_spinbox_changed(i);
    }
    
    _callback_enabled = true;
}

void SlidersWidget::setBindEnabled(bool enabled)
{
    if(!enabled)
    {
        _binding_groupbox->hide();

        for(auto w : _widget_vec)
        {
            w.lock->hide();
        }

    }
}

void SlidersWidget::setCallback( SlidersWidget::CallbackType f)
{
    _callback = f;
}

SlidersWidget::~SlidersWidget()
{
    
}

}
