#ifndef GENERICFIELD_H
#define GENERICFIELD_H
#include <eros/SystemMonitor/Field/BaseField.h>
namespace eros {
class GenericField : public BaseField
{
   public:
    GenericField() {
    }
    virtual ~GenericField() {
    }
};
}  // namespace eros
#endif