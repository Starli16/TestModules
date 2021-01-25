#define BUFF_LENGTH 300
#include <iostream>
#include <vector>
#include <mutex>
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/uart.h"

using apollo::cyber::Async;
using apollo::cyber::Yield;
class guide_Uart : public apollo::cyber::TimerComponent {
 public:
 char uart_buf[BUFF_LENGTH];
 Uart uart_dev=Uart("ttyTHS2");
 bool is_running;
 std::mutex uart_mutex;
 std::shared_ptr<Uart> dev_p;
 private:
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void Start();
  void RecvThreadFunc();
};
CYBER_REGISTER_COMPONENT(guide_Uart)
