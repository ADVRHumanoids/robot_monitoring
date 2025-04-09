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
        s->clear();
        s->append(points);
    }

    Q_INVOKABLE static void setPoints(QXYSeries* s,
                                      const QList<qreal>& t,
                                      const QList<qreal>& x)
    {
        QList<QPointF> points(t.size());

        for(int i = 0; i < t.size(); i++)
        {
            points[i] = QPointF(t[i], x[i]);
        }

        s->clear();
        s->append(points);
    }

    Q_INVOKABLE static void setPoints(QXYSeries* s,
                                      qreal dt,
                                      const QList<qreal>& x)
    {
        QList<QPointF> points(x.size());

        for(int i = 0; i < x.size(); i++)
        {
            points[i] = QPointF(dt*i, x[i]);
        }

        s->clear();
        s->append(points);
    }
};

#endif // PLOTREBUILDER_H
