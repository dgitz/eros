/*! \file Utility.h
 */
#ifndef UTILITY_H
#define UTILITY_H

#include <eros/file.h>
#include <math.h>
#include <stdio.h>

#include <string>

namespace eros {
bool isEqual(double a, double b, double eps);
struct ExecResult {
    ExecResult(bool anyError, std::string errorString, std::string result)
        : AnyError(anyError), ErrorString(errorString), Result(result) {
    }
    ExecResult() : AnyError(false), ErrorString(""), Result("") {
    }
    bool AnyError;
    std::string ErrorString;
    std::string Result;
};
//! Execute a command
/*!
  \param cmd The command to execute
  \param wait_for_results If function should return results or not.
  \return The result of the command
*/
ExecResult exec(const char* cmd, bool wait_for_result);

std::string pretty(eros::file msg);
}  // namespace eros
#endif  // UTILITY_H