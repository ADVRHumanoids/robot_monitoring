#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QtQuickControls2/QQuickStyle>
#include <QFont>

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

public:
    QString hostname { "localhost" };
    int port { 8080 };


};

int main(int argc, char *argv[])
{
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
    QGuiApplication app(argc, argv);
    auto font = app.font();
    font.setPixelSize(12);
    app.setFont(font);

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("appData", &appdata);
    for(auto p : engine.importPathList())
    {
        printf(p.toStdString().c_str());
        putc('\n', stdout);
        fflush(stdout);
    }
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
