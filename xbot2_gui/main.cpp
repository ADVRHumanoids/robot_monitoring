#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QtQuickControls2/QQuickStyle>
#include <QFont>
#include <QCommandLineParser>
#include <QtWidgets/QApplication>
#include <QtQml/qqmlregistration.h>

#include "Video/videostreampainter.h"
#include "RobotModel/robot_model.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/val.h>
#endif

#ifndef __EMSCRIPTEN__
#include <QtWebView/QtWebView>
#include "ViewerQuick3D/meshgeometry.h"
#endif

class AppData : public QObject
{
    Q_OBJECT

public:

    AppData(QObject * parent = nullptr):
        QObject(parent)
    {
        version = {XBOT2_GUI_VERSION_MAJOR, XBOT2_GUI_VERSION_MINOR, XBOT2_GUI_VERSION_PATCH};
        versionString = QString("%1.%2.%3").arg(version[0]).arg(version[1]).arg(version[2]);
#ifdef __EMSCRIPTEN__
        wasm = true;
#endif
    }

    Q_PROPERTY(QList<int> version MEMBER version);
    Q_PROPERTY(QString hostname MEMBER hostname);
    Q_PROPERTY(int port MEMBER port);
    Q_PROPERTY(bool wasm MEMBER wasm);
    Q_PROPERTY(bool portFromCmdLine MEMBER portFromCmdLine);
    Q_INVOKABLE void updateUi();
    Q_INVOKABLE uint64_t getTimeNs() const;
    Q_INVOKABLE static QUrl fromUserInput(const QString& userInput)
    {
        if (userInput.isEmpty())
            return QUrl::fromUserInput("about:blank");
        const QUrl result = QUrl::fromUserInput(userInput);
        return result.isValid() ? result : QUrl::fromUserInput("about:blank");
    }

public:

    QString hostname { "localhost" };
    int port { 8080 };
    QList<int> version;
    QString versionString;
    bool wasm = false;
    bool portFromCmdLine = false;


};

int main(int argc, char *argv[])
{
    // create appdata global object
    AppData appdata;

#ifdef __EMSCRIPTEN__
    emscripten::val location = emscripten::val::global("location");
    appdata.hostname = QString::fromStdString(location["hostname"].as<std::string>());
    appdata.port = std::stoi(location["port"].as<std::string>());
#endif


    // set app properties
    QApplication app(argc, argv);
    auto font = app.font();
    font.setPixelSize(12);
    app.setFont(font);

    app.setOrganizationName("iit-hhcm");
    app.setOrganizationDomain("hhcm.iit");
    app.setApplicationName("XBot2 UI");
    app.setApplicationVersion(appdata.versionString);


    // cli
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addVersionOption();

    parser.addOptions({
                       {{"p", "port"}, "Server port", "port"}
    });

    parser.process(app);

    if(parser.isSet("port"))
    {
        appdata.port = parser.value("port").toInt();
        appdata.portFromCmdLine = true;
        qInfo() << "set port to " << appdata.port;
    }

    // setup environment

    // verbose output
    //    qputenv("QSG_INFO", "1");
    QQuickStyle::setStyle("Material");
    qputenv("QT_QUICK_CONTROLS_MATERIAL_THEME", "Dark");
    qputenv("QT_QUICK_CONTROLS_MATERIAL_VARIANT", "Dense");
    qputenv("QT3D_RENDERER", "opengl");

#ifndef __EMSCRIPTEN__
    // initialize internal browser
    QtWebView::initialize();
#endif


    // register appdata
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("appData", &appdata);

    // load main qml file
    const QUrl url(QStringLiteral("qrc:/qt/qml/Main/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    // run event loop
    return app.exec();
}

#include "main.moc"

void AppData::updateUi()
{
    QCoreApplication::processEvents();
}

uint64_t AppData::getTimeNs() const
{
    auto now = std::chrono::high_resolution_clock::now();
    return now.time_since_epoch().count();
}
