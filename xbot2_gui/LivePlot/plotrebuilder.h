#ifndef PLOTREBUILDER_H
#define PLOTREBUILDER_H

#include <QtCharts/QtCharts>
#include <QtCharts/QXYSeries>
#include <QtQmlCore/QtQmlCore>

class PlotRebuilder: public QObject
{
    Q_OBJECT

public:

    QML_ELEMENT

    Q_INVOKABLE static QList<QPointF> getPoints(const QXYSeries* s)
    {
        qInfo() << "saving" << s->points().size() << "points from" << (void*)s;
        return s->points();
    }

    Q_INVOKABLE static void setPoints(QXYSeries* s, const QList<QPointF>& points)
    {
        qInfo() << "appending" << points.size() << "points to" << (void*)s;
        s->append(points);
    }
};

#endif // PLOTREBUILDER_H
