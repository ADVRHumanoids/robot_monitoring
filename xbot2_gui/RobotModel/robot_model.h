#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <string>
#include <memory>

#include <QObject>
#include <QList>
#include <QtQmlCore/QtQmlCore>


class Pose
{
    Q_GADGET

public:

    Q_PROPERTY(QList<qreal> translation MEMBER translation);
    Q_PROPERTY(QList<qreal> rotation MEMBER rotation);
    QML_ELEMENT

    QList<qreal> translation;
    QList<qreal> rotation;
};


class RobotModel: public QObject
{
    Q_OBJECT

public:

    QML_ELEMENT

    Q_PROPERTY(int ndof READ ndof NOTIFY ndofChanged)

    Q_PROPERTY(QList<QString> jointNames READ jointNames NOTIFY jointNamesChanged)

    RobotModel(QObject * parent = nullptr);

    Q_INVOKABLE void setUrdf(QString urdf, bool is_floating_base);

    Q_INVOKABLE void setJointPosition(QVector<qreal> q);

    Q_INVOKABLE Pose getPose(QString frame);

    int ndof();

    QList<QString> jointNames();

    ~RobotModel();

signals:

    void ndofChanged(int ndof);

    void jointNamesChanged(QList<QString> jointNames);

    void modelChanged();

private:

    class Impl;
    std::unique_ptr<Impl> i;
};

#endif // ROBOT_MODEL_H
