#ifndef IRECORD_H
#define IRECORD_H
#include <eros/SystemMonitor/Field/IField.h>

#include <vector>
namespace eros {
class IRecord
{
   public:
    IRecord() {
    }
    virtual ~IRecord() {
    }
    virtual std::vector<IField*> getFields() = 0;
    virtual void setFields(std::vector<IField*> fields) = 0;
};
}  // namespace eros
#endif