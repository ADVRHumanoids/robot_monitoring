#ifndef XBOT2_PLUGIN_WID_H
#define XBOT2_PLUGIN_WID_H

#include <QWidget>
#include <functional>
#include <QProgressBar>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QLineEdit>

class XBot2PluginWidget : public QWidget
{

    Q_OBJECT

public:

    explicit XBot2PluginWidget(QString name,
                               QWidget * parent = nullptr);

    void setLoad(double load);
    void setStatus(QString status);

signals:

    void startStopPressed(bool start);
    void abortPressed();

private:

    QLabel * _pluginLabel;
    QPushButton * _startStopBtn;
    QPushButton * _abortBtn;
    QProgressBar * _loadBar;
    QLineEdit * _statusLine;

    bool _btnStarts;



};

#endif // XBOT2_WID_H
