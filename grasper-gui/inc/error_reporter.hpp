#ifndef ERROR_REPORTER_HPP_
#define ERROR_REPORTER_HPP_

#include "error_controller.hpp"

class ErrorReporter
{
public:
    virtual void errorCleared(ErrorController::ErrorType type) = 0;
};

#endif  // ERROR_REPORTER_HPP_
