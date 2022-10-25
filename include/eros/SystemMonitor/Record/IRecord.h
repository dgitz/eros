#ifndef IRECORD_H
#define IRECORD_H
#include <eros/SystemMonitor/Field/IField.h>

#include <memory>
#include <vector>
namespace eros {
class IRecord
{
   public:
    IRecord() {
    }
    virtual ~IRecord() {
    }
    virtual std::vector<std::shared_ptr<IField>> getFields() = 0;
    virtual void setFields(std::vector<std::shared_ptr<IField>> fields) = 0;
};
}  // namespace eros
#endif