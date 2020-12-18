#ifndef ERROR_CONTROLLER_HPP_
#define ERROR_CONTROLLER_HPP_

#include <QObject>
#include <QtQuick/QQuickPaintedItem>
#include <queue>

class ErrorCaller;

class ErrorController
{
public:
    enum class ErrorType
    {

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

    void addError(ErrorCaller *callingClass, ErrorType error);

private:

    struct ErrorMsg
    {
        ErrorCaller *callingClass;
        ErrorType type;
    };

    static ErrorController *instance;

    std::queue<ErrorMsg> criticalErrorQueue;
    std::queue<ErrorMsg> nonCriticalErrorQueue;
};

class ErrorCaller
{
    virtual void errorCleared(ErrorController::ErrorType type);
};

#endif  // ERROR_CONTROLLER_HPP_
