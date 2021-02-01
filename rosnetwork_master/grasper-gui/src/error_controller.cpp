#include "error_controller.hpp"

#include <iostream>

#include <QDebug>

#include "error_reporter.hpp"

ErrorController *ErrorController::instance = nullptr;

ErrorController::ErrorController() {}

void ErrorController::addConnections(QObject *root)
{
    QObject::connect(
        this,
        SIGNAL(showCriticalErrorDialogue(QVariant, QVariant)),
        root->findChild<QObject *>("criticalErrorBanner"),
        SLOT(triggerCriticalErrorDialogue(QVariant, QVariant)),
        Qt::QueuedConnection);
    QObject::connect(
        this,
        SIGNAL(hideCriticalErrorDialogue()),
        root->findChild<QObject *>("criticalErrorBanner"),
        SLOT(hideCriticalErrorDialogue()),
        Qt::QueuedConnection);
    QObject::connect(
        this,
        SIGNAL(showNonurgentErrorDialogue(QVariant)),
        root->findChild<QObject *>("noncriticalErrorBanner"),
        SLOT(triggerNonurgentErrorDialogue(QVariant)),
        Qt::QueuedConnection);
    QObject::connect(
        this,
        SIGNAL(hideNonurgentErrorDialogue()),
        root->findChild<QObject *>("noncriticalErrorBanner"),
        SLOT(hideNonurgentErrorDialogue()),
        Qt::QueuedConnection);
    QObject::connect(
        root->findChild<QObject *>("criticalErrorBanner"),
        SIGNAL(criticalErrorOKPressed()),
        this,
        SLOT(onCriticalMsgOkPressed()),
        Qt::QueuedConnection);
}

void ErrorController::onCriticalMsgOkPressed()
{
    if (m_criticalErrorQueue.size() != 0)
    {
        removeError(m_criticalErrorQueue.begin()->type);
    }
}

void ErrorController::initialize()
{
    // The more critical the error the higher the priority.
    // Start all noncritical errors with priority 0 and all
    // critical errors with priority 100. If priority for two
    // errors are the same, the most recently displayed error
    // will overwrite the previously written error. Once the
    // most recently written error is fixed, a queue of previous
    // errors that are not resolved is maintained and the next
    // most recently added error is added.
    m_criticalErrorToPriorityMap[ErrorType::SERIAL_NODE_NOT_RUNNING] = 1;
    m_criticalErrorToPriorityMap[ErrorType::TEENSY_DISCONNECTED] = 2;
    m_criticalErrorToPriorityMap[ErrorType::CAMERA_NODE_NOT_RUNNING] = 3;
}

void ErrorController::addError(ErrorReporter *callingClass, ErrorType error)
{
    if (attemptToAddError(error, callingClass, m_criticalErrorToPriorityMap, m_criticalErrorQueue))
    {
        displayCriticalError();
    }
    else if (attemptToAddError(
                 error,
                 callingClass,
                 m_nonCriticalErrorToPriorityMap,
                 m_nonCriticalErrorQueue))
    {
        displayNoncriticalError();
    }
}

void ErrorController::removeError(ErrorType error)
{
    if (attemptToRemoveError(error, m_criticalErrorToPriorityMap, m_criticalErrorQueue))
    {
        emit hideCriticalErrorDialogue();
        displayCriticalError();
    }
    else if (attemptToRemoveError(error, m_nonCriticalErrorToPriorityMap, m_nonCriticalErrorQueue))
    {
        emit hideNonurgentErrorDialogue();
        displayNoncriticalError();
    }
}

bool ErrorController::attemptToRemoveError(
    ErrorType error,
    const std::map<ErrorType, uint8_t> &errorToPriorityMap,
    std::set<ErrorMsg, ErrorMsgCmp> &errorQueue)
{
    if (errorToPriorityMap.find(error) != errorToPriorityMap.end())
    {
        ErrorMsg msgToRemove;
        qDebug() << "removing error " << (int)error;
        bool msgFound = false;
        for (const auto &errorMsg : errorQueue)
        {
            if (errorMsg.type == error)
            {
                msgFound = true;
                msgToRemove = errorMsg;
                break;
            }
        }
        if (msgFound)
        {
            errorQueue.erase(msgToRemove);
            if (msgToRemove.callingClass != nullptr)
            {
                msgToRemove.callingClass->errorCleared(msgToRemove.type);
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool ErrorController::attemptToAddError(
    ErrorType error,
    ErrorReporter *callingClass,
    const std::map<ErrorType, uint8_t> &errorToPriorityMap,
    std::set<ErrorMsg, ErrorMsgCmp> &errorQueue)
{
    if (errorToPriorityMap.find(error) != errorToPriorityMap.end())
    {
        ErrorMsg msgToAdd(
            callingClass,
            error,
            QDateTime::currentMSecsSinceEpoch(),
            errorToPriorityMap.at(error));
        ErrorMsg msgToRemove;
        // Remove errors in queue that are identical (but are going to be older
        // than the newly added error.
        if (std::find_if(
                errorQueue.begin(),
                errorQueue.end(),
                [msgToAdd, msgToRemove](const ErrorMsg &error) mutable {
                    // Types match, this is the message to be removed.
                    if (error.type == msgToAdd.type)
                    {
                        msgToRemove = error;
                        return true;
                    }
                    return false;
                }) != errorQueue.end())
        {
            // We have found the msgToRemove in the list, so remove it.
            errorQueue.erase(msgToRemove);
        }
        // Insert the msgToAdd, this has a unique message type in the
        // errorQueue.
        errorQueue.insert(msgToAdd);
        return true;
    }
    else
    {
        return false;
    }
}

void ErrorController::displayNoncriticalError()
{
    if (m_nonCriticalErrorQueue.size() == 0)
    {
        emit hideNonurgentErrorDialogue();
        return;
    }
    switch (m_nonCriticalErrorQueue.begin()->type)
    {
        case ErrorType::CAMERA_NODE_NOT_RUNNING:
        case ErrorType::SERIAL_NODE_NOT_RUNNING:
        case ErrorType::TEENSY_DISCONNECTED:
        default:
            break;
    }
}

void ErrorController::displayCriticalError()
{
    if (m_criticalErrorQueue.size() == 0)
    {
        emit hideCriticalErrorDialogue();
        return;
    }
    switch (m_criticalErrorQueue.begin()->type)
    {
        case ErrorType::CAMERA_NODE_NOT_RUNNING:
            emit showCriticalErrorDialogue(
                "Camera node is not running.\nCheck the Raspberry Pi.",
                true);
            break;
        case ErrorType::SERIAL_NODE_NOT_RUNNING:
            emit showCriticalErrorDialogue(
                "Serial node is not running.\nCheck the Raspberry Pi.",
                true);
            break;
        case ErrorType::TEENSY_DISCONNECTED:
            emit showCriticalErrorDialogue(
                "The teensy is not connected. Check the connection\nbetween "
                "the Raspberry Pi and teensy.",
                true);
            break;
        default:
            break;
    }
}
