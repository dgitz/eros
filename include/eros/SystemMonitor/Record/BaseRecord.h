#ifndef BASERECORD_H
#define BASERECORD_H

#include "IRecord.h"
namespace eros {
class BaseRecord : public IRecord
{
   public:
    BaseRecord() {
    }
    virtual ~BaseRecord() {
    }
    std::vector<std::shared_ptr<IField>> getFields() {
        return fields;
    }
    void setFields(std::vector<std::shared_ptr<IField>> newFields) {
        fields = newFields;
    }

   private:
    std::vector<std::shared_ptr<IField>> fields;
};
}  // namespace eros
#endif