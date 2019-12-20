#ifndef CIRCLE_WIDGET_H
#define CIRCLE_WIDGET_H

#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QPainter>

class CircleWidget : public QWidget
{
public:
   CircleWidget(int number) : _number(number), _fill(false) {/*empty*/}

   virtual void paintEvent(QPaintEvent * e)
   {
      e->accept();

      QPainter p(this);

      QRect r = rect();
      p.setPen(Qt::black);
      if (_fill) p.setBrush(Qt::green);
      p.drawEllipse(r);

      p.drawText(r, Qt::AlignCenter, QString("%1").arg(_number));
   }

   virtual void mousePressEvent(QMouseEvent * e)
   {
      _fill = !_fill;
      update();  // this will induce another call to paintEvent() ASAP
      e->accept();
   }

private:
   const int _number;
   bool _fill;
};
#endif // CIRCLE_WIDGET_H
