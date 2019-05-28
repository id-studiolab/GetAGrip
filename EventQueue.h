// By Richard Bekking
// richard@electronicsdesign.nl

#pragma ONCE 

#include "Fsm.h"

enum QueueState {
  READY,
  FULL,
  EMPTY
};

class EventQueue {
  public:
    EventQueue() : queue_top(0), queue_state(EMPTY) { for (int i = 0; i < max_entries; ++i) queue[i] = 0; };

    bool push(int value);
    int peek_from_top(int level);
    int pop();

    QueueState state();

  private:
    enum {max_entries = 10};
    
    QueueState queue_state;
    int queue_top;
    int queue[max_entries];    
};
