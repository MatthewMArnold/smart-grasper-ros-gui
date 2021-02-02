#ifndef ERROR_CONTROLLER_HPP_
#define ERROR_CONTROLLER_HPP_

#include <map>
#include <set>

#include <QDateTime>
#include <QObject>
#include <QtQuick/QQuickPaintedItem>

class ErrorReporter;

class ErrorController : public QObject
{
    Q_OBJECT

public:
    enum class ErrorType
    {
        SERIAL_NODE_NOT_RUNNING = 0,
        CAMERA_NODE_NOT_RUNNING = 1,
        TEENSY_DISCONNECTED = 2
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

    void addConnections(QObject *root);

public slots:
    void onCriticalMsgOkPressed();

signals:
    void showCriticalErrorDialogue(QVariant msg, QVariant dismissable);
    void hideCriticalErrorDialogue();
    void showNonurgentErrorDialogue(QVariant msg);
    void hideNonurgentErrorDialogue();

private:
    struct ErrorMsg
    {
    public:
        ErrorMsg()
            : callingClass(nullptr),
              type(ErrorType::SERIAL_NODE_NOT_RUNNING),
              timeAdded(0),
              priority(0)
        {
        }
        ErrorMsg(ErrorReporter *callingClass, ErrorType type, qint64 timeAdded, uint8_t priority)
            : callingClass(callingClass),
              type(type),
              timeAdded(timeAdded),
              priority(priority)
        {
        }
        ErrorReporter *callingClass;
        ErrorType type;
        qint64 timeAdded;
        uint8_t priority;
    };

    struct ErrorMsgCmp
    {
        bool operator()(const ErrorMsg &msg1, const ErrorMsg &msg2)
        {
            if (msg1.priority != msg2.priority)
            {
                return msg1.priority < msg2.priority;
            }
            else if (msg1.timeAdded != msg2.timeAdded)
            {
                return msg1.timeAdded < msg2.timeAdded;
            }
            else
            {
                return static_cast<int>(msg1.type) < static_cast<int>(msg2.type);
            }
        }
    };

    static ErrorController *instance;

    std::map<ErrorType, uint8_t> m_criticalErrorToPriorityMap;
    std::map<ErrorType, uint8_t> m_nonCriticalErrorToPriorityMap;
    std::set<ErrorMsg, ErrorMsgCmp> m_criticalErrorQueue;
    std::set<ErrorMsg, ErrorMsgCmp> m_nonCriticalErrorQueue;

    ErrorController();

    bool attemptToAddError(
        ErrorType error,
        ErrorReporter *callingClass,
        const std::map<ErrorType, uint8_t> &errorToPriorityMap,
        std::set<ErrorMsg, ErrorMsgCmp> &errorQueue);

    bool attemptToRemoveError(
        ErrorType error,
        const std::map<ErrorType, uint8_t> &errorToPriorityMap,
        std::set<ErrorMsg, ErrorMsgCmp> &errorQueue);

    void displayNoncriticalError();

    void displayCriticalError();
};

#endif  // ERROR_CONTROLLER_HPP_
