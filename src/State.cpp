#include "State.h"

void changeState(State* next) {
  if (next == nullptr || next == currentState) return;
  if (currentState) currentState->exit();
  previousState = currentState;
  currentState = next;
  currentState->init();
}
