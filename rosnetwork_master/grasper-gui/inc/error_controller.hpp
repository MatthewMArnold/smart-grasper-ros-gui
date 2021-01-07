#ifndef ERROR_CONTROLLER_HPP_
#define ERROR_CONTROLLER_HPP_

#include <QObject>
#include <QtQuick/QQuickPaintedItem>
#include <queue>

class ErrorReporter;

class ErrorController
{
public:
    enum class ErrorType
    {
        TEENSY_DISCONNECTED = 0,
    };

    static ErrorController *getInstance()
    {
        if (instance == nullptr)
        {
            instance = new ErrorController();
        }
        return instance;
    }

    void initialize();

    void addError(ErrorReporter *callingClass, ErrorType error);

    void removeError(ErrorType error);

private:

    struct ErrorMsg
    {
        ErrorReporter *callingClass;
        ErrorType type;
    };

    static ErrorController *instance;

    std::queue<ErrorMsg> criticalErrorQueue;
    std::queue<ErrorMsg> nonCriticalErrorQueue;
};

#endif  // ERROR_CONTROLLER_HPP_
