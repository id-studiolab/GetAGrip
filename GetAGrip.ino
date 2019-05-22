#include "Fsm.h"

// State-change triggers:
const int ALARM_EXPIRED = 0;
const int CTA_EXPIRED = 1;
const int STRESS_DETECTED = 2;
const int CHALLENGE_BUTTON_ACTIVATED = 3;
const int GPS_ACTIVITY = 4;

void on_poweroff_enter() {

}

void on_standby_enter() {

}

void on_stressalarm_enter() {

}

void on_challenge_enter() {

}

void on_calltoaction_enter() {

}

State state_standby(&on_standby_enter, NULL, NULL);
State state_stressalarm(&on_stressalarm_enter, NULL, NULL);
State state_challenge(&on_challenge_enter, NULL, NULL);
State state_calltoaction(&on_calltoaction_enter, NULL, NULL);

Fsm fsm_main(&state_standby);

void setup() {
  Serial.begin(9600);

  fsm_main.add_transition(&state_standby, &state_stressalarm,
                     STRESS_DETECTED,
                     NULL);
  fsm_main.add_transition(&state_stressalarm, &state_standby,
                     ALARM_EXPIRED,
                     NULL);
  fsm_main.add_transition(&state_stressalarm, &state_challenge,
                     CHALLENGE_BUTTON_ACTIVATED,
                     NULL);
  fsm_main.add_transition(&state_challenge, &state_standby,
                     CHALLENGE_BUTTON_ACTIVATED,
                     NULL);
  fsm_main.add_transition(&state_standby, &state_calltoaction,
                     GPS_ACTIVITY,
                     NULL);
  fsm_main.add_transition(&state_calltoaction, &state_challenge,
                     CHALLENGE_BUTTON_ACTIVATED,
                     NULL);
  fsm_main.add_transition(&state_calltoaction, &state_standby,
                     CTA_EXPIRED,
                     NULL);
  fsm_main.add_transition(&state_standby, &state_challenge,
                     CHALLENGE_BUTTON_ACTIVATED,
                     NULL);

}

void loop() {  
  fsm_main.run_machine();
  delay(100);
}
