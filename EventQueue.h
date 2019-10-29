// By Richard Bekking
// richard@electronicsdesign.nl

#ifndef EVENTQUEUE_H_
#define EVENTQUEUE_H_

#include "Fsm.h"

enum QueueState {
  READY,
  FULL,
  EMPTY
};

class EventQueue {
  public:
    EventQueue() = default;

    bool push(int value);           // Push an event on top of the stack
    int peek_from_top(int level);   // level = 0 means top of the stack. This function does not mutate the stack.
    int pop();                      // Pop the topmost event from the stack.

    QueueState state();             // Returns: READY, FULL or EMPTY

  private:
    enum {max_entries = 12};        // Adjust this if 5 entries is not enough (consumes more memory when larger)
    
    QueueState queue_state = {EMPTY};
    int queue_top = {0};
    int queue[max_entries] = {};    
};

#endif // EVENTQUEUE_H_
