#include "guide_uart.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <cstring>
using std::cout;
using std::endl;
using namespace std;

bool guide_Uart::Init() {
  memset(uart_buf,0,sizeof(uart_buf));
 // dev_p=std::make_shared<Uart>("ttyS0");
  //uart_dev=Uart("ttyTHS2");
  uart_dev.SetOpt(9600, 8, 'N' ,1);
  is_running=0;
  Start();
  return true;
}

bool guide_Uart::Proc() {  // Timer callback
  {
    std::lock_guard<mutex> guard(uart_mutex);
    AINFO<<uart_buf;
  }
  return true;
}
void guide_Uart::Clear() {  // shutdown
  is_running=0;
}
void guide_Uart::Start(){
  is_running=1;
  AINFO<<"Uart Started";
  auto async_result_ = Async(&guide_Uart::RecvThreadFunc, this);
}
void guide_Uart::RecvThreadFunc(){
  while(is_running){
    {
      char buf[BUFF_LENGTH];
      uart_dev.Read(buf,sizeof(uart_buf));
      {
        std::lock_guard<mutex> guard(uart_mutex);
        strcpy(uart_buf,buf);
      }
      AINFO<<"ReadOK";
      Yield();
    }
  }
}
