#ifndef GENERICRECORD_H
#define GENERICRECORD_H
#include <eros/SystemMonitor/Record/BaseRecord.h>
namespace eros {
class GenericRecord : public BaseRecord
{
   public:
    GenericRecord() {
    }
    virtual ~GenericRecord() {
    }
};
}  // namespace eros
#endif