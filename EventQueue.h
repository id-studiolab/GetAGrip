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

    bool push(int value);           // Push an event on top of the stack
    int peek_from_top(int level);   // level = 0 means top of the stack. This function does not mutate the stack.
    int pop();                      // Pop the topmost event from the stack.

    QueueState state();             // Returns: READY, FULL or EMPTY

  private:
    enum {max_entries = 12};        // Adjust this if 5 entries is not enough (consumes more memory when larger)
    
    QueueState queue_state;
    int queue_top;
    int queue[max_entries];    
};
