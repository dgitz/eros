#ifndef IFIELD_H
#define IFIELD_H
#include <eros/SystemMonitor/DataStructures.h>
namespace eros {
class IField
{
   public:
    IField() {
    }
    virtual ~IField() {
    }
    virtual RenderData getData() = 0;
    virtual void setData(RenderData data) = 0;
};
}  // namespace eros
#endif