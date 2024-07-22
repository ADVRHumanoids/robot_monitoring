#include "robot_model.h"

#include "urdf/model.h"
#include "urdf/link.h"
#include "urdfreader.h"
#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

class RobotModel::Impl
{

public:

    Impl(std::string urdf, bool is_floating_base);

    void setJointPosition(QList<qreal> q);

    Pose getPose(QString frame);

    QList<QString> getJointNames();

    int ndof = 0;

    QList<QString> jointNames;

private:

    RigidBodyDynamics::Model _model;
    Eigen::VectorXd _qeig;
    std::shared_ptr<urdf::UrdfModel> _urdf;
};

void RobotModel::setUrdf(QString urdf, bool is_floating_base)
{
    i = std::make_unique<Impl>(urdf.toStdString(), is_floating_base);

    i->jointNames = i->getJointNames();
    emit jointNamesChanged(jointNames());

    i->ndof = i->getJointNames().size();
    emit ndofChanged(ndof());

    emit modelChanged();
}

RobotModel::RobotModel(QObject *parent):
    QObject(parent)
{
}

void RobotModel::setJointPosition(QVector<qreal> q)
{
    i ? i->setJointPosition(q) : void();
}

Pose RobotModel::getPose(QString frame)
{
    return i ? i->getPose(frame) : Pose();
}

int RobotModel::ndof()
{
    return i ? i->ndof : 0;
}

QList<QString> RobotModel::jointNames()
{
    return i ? i->jointNames : QList<QString>();
}

void RobotModel::Impl::setJointPosition(QList<qreal> q)
{
    if(q.size() != _qeig.size())
    {
        qFatal("wrong q size");
        return;
    }

    _qeig = Eigen::VectorXd::Map(q.data(), q.size());
    RigidBodyDynamics::UpdateKinematicsCustom(_model, &_qeig, nullptr, nullptr);
}

Pose RobotModel::Impl::getPose(QString frame)
{
    auto bid = _model.GetBodyId(frame.toStdString().c_str());

    if(bid == std::numeric_limits<uint>::max())
    {
        std::cout << "could not find link '" << frame.toStdString() << "' \n";
        return Pose();
    }

    auto pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(
                _model,
                _qeig,
                bid,
                Eigen::Vector3d::Zero(),
                false);

    auto rot = RigidBodyDynamics::CalcBodyWorldOrientation(
                _model,
                _qeig,
                bid,
                false);

    Eigen::Quaterniond rot_q_eig(rot.transpose());

    QList<qreal> qpos{pos.x(), pos.y(), pos.z()};

    QList<qreal> qrot{
        rot_q_eig.w(),
                rot_q_eig.x(),
                rot_q_eig.y(),
                rot_q_eig.z()
    };

    Pose ret;
    ret.translation = qpos;
    ret.rotation = qrot;

    return ret;
}

QList<QString> RobotModel::Impl::getJointNames()
{
    std::cout << "_model.mJoints.size() " << _model.mJoints.size() << "\n";
    std::cout << "_model.mBodies.size() " << _model.mBodies.size() << "\n";
    std::cout << "_model.dof_count " << _model.dof_count << "\n";

    QList<QString> ret(_model.dof_count);

    for(int i = 0; i < _model.mJoints.size(); i++)
    {
        auto link_name = _model.GetBodyName(i);

        if(link_name.empty())
        {
            std::cout << i << ": got empty link name \n";
            continue;
        }

        auto link = _urdf->getLink(link_name);

        if(!link)
        {
            std::cout << i << ": got null link \n";
            continue;
        }

        if(!link->parent_joint)
        {
            std::cout << i << ": got null parent joint name \n";
            continue;
        }

        auto joint_name = link->parent_joint->name;

        // std::cout << "found joint " << i << " with name " << joint_name <<
        //              " q_index = " << _model.mJoints[i].q_index <<
        //              " DoFCount = " << _model.mJoints[i].mDoFCount << std::endl;

        ret[_model.mJoints[i].q_index] = QString::fromStdString(joint_name);


    }

    return ret;
}

RobotModel::~RobotModel()
{

}

RobotModel::Impl::Impl(std::string urdf, bool is_floating_base)
{
    // Save locale setting
    const std::string oldLocale = setlocale(LC_NUMERIC, nullptr);

    // Force '.' as the radix point. If you comment this out,
    // you'll get output similar to the OP's GUI mode sample
    setlocale(LC_NUMERIC, "C");

    _urdf = urdf::UrdfModel::fromUrdfStr(urdf.c_str());

    RigidBodyDynamics::Addons::URDFReadFromString(urdf.c_str(),
                                                  &_model,
                                                  is_floating_base);

    // Restore locale setting
    setlocale(LC_NUMERIC, oldLocale.c_str());

    // set zero config
    _qeig.setZero(_model.dof_count);
    RigidBodyDynamics::UpdateKinematicsCustom(_model, &_qeig, nullptr, nullptr);

}
