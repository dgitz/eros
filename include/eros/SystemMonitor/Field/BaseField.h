#ifndef BASEFIELD_H
#define BASEFIELD_H
#include "IField.h"
namespace eros {
class BaseField : public IField
{
   public:
    BaseField() {
    }
    virtual ~BaseField() {
    }
    RenderData getData() {
        return data;
    }
    void setData(RenderData newData) {
        data = newData;
    }

   private:
    RenderData data;
};
}  // namespace eros
#endif