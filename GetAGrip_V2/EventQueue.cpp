// By Richard Bekking
// richard@electronicsdesign.nl

#include "EventQueue.h"

bool EventQueue::push(int value)
{
  if (queue_top < max_entries) {
    ++queue_top;
    queue[queue_top] = value;

    if (queue_top == max_entries) {
      queue_state = FULL;
    }
    else {
      queue_state = READY;
    }

    return true; // Push succeeded
  }
  else {
    return false; // Push failed
  }
}

int EventQueue::peek_from_top(int level)
{
  int index = queue_top - level;
  if ((index >= 0) && (index < max_entries)) {
    return queue[index];
  }
  else {
    return 0;
  }
}

int EventQueue::pop()
{
  int retval = 0;

  if (queue_state != EMPTY) {
    retval = queue[queue_top];

    if (queue_top == 0) {
      queue_state = EMPTY;
    } else {
      queue_state = READY;
      --queue_top;
    }
  }
  else {
    queue_state = READY;
    retval = 0;
  }

  return retval;
}

QueueState EventQueue::state()
{
  return queue_state;
}
