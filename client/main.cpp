#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QtQuickControls2/QQuickStyle>
#include <QFont>
#include <QtWidgets/QApplication>

#include <QtQml/qqmlregistration.h>

#include "Video/videostreampainter.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/val.h>
#endif

class AppData : public QObject
{
    Q_OBJECT

public:

    Q_PROPERTY(QString hostname MEMBER hostname);
    Q_PROPERTY(int port MEMBER port);
    Q_INVOKABLE void updateUi();
    Q_INVOKABLE uint64_t getTimeNs() const;

public:
    QString hostname { "localhost" }; // "10.240.23.38" };
    int port { 8080 };


};

int main(int argc, char *argv[])
{
    // verbose output
    //    qputenv("QSG_INFO", "1");
    QQuickStyle::setStyle("Material");
    qputenv("QT_QUICK_CONTROLS_MATERIAL_THEME", "Dark");
    qputenv("QT_QUICK_CONTROLS_MATERIAL_VARIANT", "Dense");
    qputenv("QT3D_RENDERER", "opengl");


#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    AppData appdata;
#ifdef __EMSCRIPTEN__
    emscripten::val location = emscripten::val::global("location");
    appdata.hostname = QString::fromStdString(location["hostname"].as<std::string>());
    appdata.port = std::stoi(location["port"].as<std::string>());
    QQuickStyle::setStyle("Fusion");
#endif
    QApplication app(argc, argv);
    auto font = app.font();
    font.setPixelSize(12);
    app.setFont(font);

    app.setOrganizationName("iit-hhcm");
    app.setOrganizationDomain("hhcm.iit");
    app.setApplicationName("XBot2 UI");

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("appData", &appdata);

    qmlRegisterType<VideoStreamPainter>("NextUiModules", 1, 0, "VideoStreamPainter");
    qmlRegisterSingletonType(QUrl("qrc:/CommonProperties.qml"), "xbot2_gui.common", 1, 0, "CommonProperties");

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);


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
