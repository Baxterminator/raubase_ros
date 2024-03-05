#ifndef RAUBASE_MSG_QUEUE
#define RAUBASE_MSG_QUEUE

#include "common/shared_queue.hpp"
#include "common/types.hpp"
#include "teensy/interface/message.hpp"
namespace raubase::teensy {

struct MSGQueue : public SharedQueue<sptr<MSG>> {
  bool firstMSGSent() {
    lock();
    bool sent = front()->sent_count > 0;
    unlock();
    return sent;
  }
};

}  // namespace raubase::teensy

#endif