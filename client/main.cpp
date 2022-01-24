#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#ifdef __EMSCRIPTEN__
    #include <emscripten/val.h>
#endif

struct AppData : QObject
{
    Q_OBJECT
    Q_PROPERTY(QString hostname MEMBER hostname);
    Q_PROPERTY(int port MEMBER port);

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
#endif

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("appData", &appdata);
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
