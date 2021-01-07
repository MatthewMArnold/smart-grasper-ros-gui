#include "error_controller.hpp"
#include "error_reporter.hpp"

ErrorController *ErrorController::instance = nullptr;

void ErrorController::initialize()
{
}

void ErrorController::addError(ErrorReporter *callingClass, ErrorType error)
{
    Q_UNUSED(callingClass);
    Q_UNUSED(error);
}

void ErrorController::removeError(ErrorType error)
{
    Q_UNUSED(error);
}
