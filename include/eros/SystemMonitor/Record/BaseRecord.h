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
    std::vector<IField*> getFields() {
        return fields;
    }
    void setFields(std::vector<IField*> newFields) {
        fields = newFields;
    }

   private:
    std::vector<IField*> fields;
};
}  // namespace eros
#endif