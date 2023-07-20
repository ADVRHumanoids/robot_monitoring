#ifndef JOINT_STATE_WIDGET_H
#define JOINT_STATE_WIDGET_H

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QComboBox>

/**
 * @brief The JointStateWidget class shows the full
 * information for one robot joint
 */
class JointStateWidget : public QWidget
{

    Q_OBJECT

public:

    explicit JointStateWidget(QWidget * parent = nullptr);

    /**
     * @brief setJointName sets the widget joint name and id
     */
    void setJointName(QString jname, int jid);

    void setAux(QString aux_type, double value);

    QDoubleSpinBox * posref, * motopos, * linkpos;
    QDoubleSpinBox * velref, * motovel, * linkvel;
    QDoubleSpinBox * torref, * torref_imp, * tor;
    QDoubleSpinBox * aux;
    QDoubleSpinBox * mototemp, * drivertemp;
    QDoubleSpinBox * stiffness, * damping;

    QComboBox * aux_type_combo;

    QString getJointName() const { return _jname; }
    void setStatus(std::string status);
    void updateStatus();

signals:

    void plotAdded(QString field);

private:

    typedef std::chrono::high_resolution_clock::time_point time_point;

    QGroupBox * group;
    QString _jname;
    QLabel * _fault;

    time_point _last_fault_time;
    std::map<QString, time_point> _aux_timeout;


};

#endif // JOINT_STATE_WIDGET_H
