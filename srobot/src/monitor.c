#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "monitor_types.h"
#include "monitor.h"

static int64_t OpState_cpy;
static bool slowdown_cpy;
static bool halt_cpy;
static bool alert_cpy;
static bool turnoffUVC_cpy;
static int64_t classifier_cpy;
static int64_t distance_to_target_cpy;
static int64_t s0[(1)] = {((int64_t)(0))};
static int64_t s1[(1)] = {((int64_t)(0))};
static bool s2[(1)] = {(false)};
static bool s3[(1)] = {(false)};
static bool s4[(1)] = {(true)};
static int64_t s5[(1)] = {((int64_t)(0))};
static bool s6[(1)] = {(false)};
static bool s7[(1)] = {(false)};
static bool s8[(1)] = {(true)};
static int64_t s9[(1)] = {((int64_t)(0))};
static bool s10[(1)] = {(false)};
static bool s11[(1)] = {(false)};
static bool s12[(1)] = {(true)};
static int64_t s13[(1)] = {((int64_t)(0))};
static bool s14[(1)] = {(false)};
static bool s15[(1)] = {(false)};
static bool s16[(1)] = {(true)};
static bool s17[(1)] = {(true)};
static bool s18[(1)] = {(true)};
static bool s19[(1)] = {(true)};
static bool s20[(1)] = {(true)};
static bool s21[(1)] = {(true)};
static bool s22[(1)] = {(true)};
static bool s23[(1)] = {(true)};
static bool s24[(1)] = {(true)};
static bool s25[(1)] = {(true)};
static size_t s0_idx = (0);
static size_t s1_idx = (0);
static size_t s2_idx = (0);
static size_t s3_idx = (0);
static size_t s4_idx = (0);
static size_t s5_idx = (0);
static size_t s6_idx = (0);
static size_t s7_idx = (0);
static size_t s8_idx = (0);
static size_t s9_idx = (0);
static size_t s10_idx = (0);
static size_t s11_idx = (0);
static size_t s12_idx = (0);
static size_t s13_idx = (0);
static size_t s14_idx = (0);
static size_t s15_idx = (0);
static size_t s16_idx = (0);
static size_t s17_idx = (0);
static size_t s18_idx = (0);
static size_t s19_idx = (0);
static size_t s20_idx = (0);
static size_t s21_idx = (0);
static size_t s22_idx = (0);
static size_t s23_idx = (0);
static size_t s24_idx = (0);
static size_t s25_idx = (0);

static int64_t s0_get(size_t x) {
  return (s0)[((s0_idx) + (x)) % ((size_t)(1))];
}

static int64_t s1_get(size_t x) {
  return (s1)[((s1_idx) + (x)) % ((size_t)(1))];
}

static bool s2_get(size_t x) {
  return (s2)[((s2_idx) + (x)) % ((size_t)(1))];
}

static bool s3_get(size_t x) {
  return (s3)[((s3_idx) + (x)) % ((size_t)(1))];
}

static bool s4_get(size_t x) {
  return (s4)[((s4_idx) + (x)) % ((size_t)(1))];
}

static int64_t s5_get(size_t x) {
  return (s5)[((s5_idx) + (x)) % ((size_t)(1))];
}

static bool s6_get(size_t x) {
  return (s6)[((s6_idx) + (x)) % ((size_t)(1))];
}

static bool s7_get(size_t x) {
  return (s7)[((s7_idx) + (x)) % ((size_t)(1))];
}

static bool s8_get(size_t x) {
  return (s8)[((s8_idx) + (x)) % ((size_t)(1))];
}

static int64_t s9_get(size_t x) {
  return (s9)[((s9_idx) + (x)) % ((size_t)(1))];
}

static bool s10_get(size_t x) {
  return (s10)[((s10_idx) + (x)) % ((size_t)(1))];
}

static bool s11_get(size_t x) {
  return (s11)[((s11_idx) + (x)) % ((size_t)(1))];
}

static bool s12_get(size_t x) {
  return (s12)[((s12_idx) + (x)) % ((size_t)(1))];
}

static int64_t s13_get(size_t x) {
  return (s13)[((s13_idx) + (x)) % ((size_t)(1))];
}

static bool s14_get(size_t x) {
  return (s14)[((s14_idx) + (x)) % ((size_t)(1))];
}

static bool s15_get(size_t x) {
  return (s15)[((s15_idx) + (x)) % ((size_t)(1))];
}

static bool s16_get(size_t x) {
  return (s16)[((s16_idx) + (x)) % ((size_t)(1))];
}

static bool s17_get(size_t x) {
  return (s17)[((s17_idx) + (x)) % ((size_t)(1))];
}

static bool s18_get(size_t x) {
  return (s18)[((s18_idx) + (x)) % ((size_t)(1))];
}

static bool s19_get(size_t x) {
  return (s19)[((s19_idx) + (x)) % ((size_t)(1))];
}

static bool s20_get(size_t x) {
  return (s20)[((s20_idx) + (x)) % ((size_t)(1))];
}

static bool s21_get(size_t x) {
  return (s21)[((s21_idx) + (x)) % ((size_t)(1))];
}

static bool s22_get(size_t x) {
  return (s22)[((s22_idx) + (x)) % ((size_t)(1))];
}

static bool s23_get(size_t x) {
  return (s23)[((s23_idx) + (x)) % ((size_t)(1))];
}

static bool s24_get(size_t x) {
  return (s24)[((s24_idx) + (x)) % ((size_t)(1))];
}

static bool s25_get(size_t x) {
  return (s25)[((s25_idx) + (x)) % ((size_t)(1))];
}

static int64_t s0_gen(void) {
  return ((s0_get)((0))) + ((int64_t)(1));
}

static int64_t s1_gen(void) {
  return (s0_get)((0));
}

static bool s2_gen(void) {
  return ((OpState_cpy) == ((int64_t)(0))) && (!((((!(slowdown_cpy)) && (!(halt_cpy))) && (!(alert_cpy))) && (!(turnoffUVC_cpy))));
}

static bool s3_gen(void) {
  return true;
}

static bool s4_gen(void) {
  return ((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(0))) && (!((((!(slowdown_cpy)) && (!(halt_cpy))) && (!(alert_cpy))) && (!(turnoffUVC_cpy)))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s1_get)((0)))) && ((((s1_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s2_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s3_get)((0)))) || ((((!(slowdown_cpy)) && (!(halt_cpy))) && (!(alert_cpy))) && (!(turnoffUVC_cpy))))))) && ((s4_get)((0)));
}

static int64_t s5_gen(void) {
  return (s0_get)((0));
}

static bool s6_gen(void) {
  return ((OpState_cpy) == ((int64_t)(1))) && (!((((!(slowdown_cpy)) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy))));
}

static bool s7_gen(void) {
  return true;
}

static bool s8_gen(void) {
  return ((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(1))) && (!((((!(slowdown_cpy)) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy)))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s5_get)((0)))) && ((((s5_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s6_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s7_get)((0)))) || ((((!(slowdown_cpy)) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy))))))) && ((s8_get)((0)));
}

static int64_t s9_gen(void) {
  return (s0_get)((0));
}

static bool s10_gen(void) {
  return ((OpState_cpy) == ((int64_t)(2))) && (!((((slowdown_cpy) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy))));
}

static bool s11_gen(void) {
  return true;
}

static bool s12_gen(void) {
  return ((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(2))) && (!((((slowdown_cpy) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy)))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s9_get)((0)))) && ((((s9_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s10_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s11_get)((0)))) || ((((slowdown_cpy) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy))))))) && ((s12_get)((0)));
}

static int64_t s13_gen(void) {
  return (s0_get)((0));
}

static bool s14_gen(void) {
  return ((OpState_cpy) == ((int64_t)(3))) && (!((((!(slowdown_cpy)) && (halt_cpy)) && (!(alert_cpy))) && (turnoffUVC_cpy)));
}

static bool s15_gen(void) {
  return true;
}

static bool s16_gen(void) {
  return ((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(3))) && (!((((!(slowdown_cpy)) && (halt_cpy)) && (!(alert_cpy))) && (turnoffUVC_cpy))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s13_get)((0)))) && ((((s13_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s14_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s15_get)((0)))) || ((((!(slowdown_cpy)) && (halt_cpy)) && (!(alert_cpy))) && (turnoffUVC_cpy)))))) && ((s16_get)((0)));
}

static bool s17_gen(void) {
  return ((!(!((classifier_cpy) == ((int64_t)(0))))) || ((distance_to_target_cpy) >= ((int64_t)(0)))) && ((s17_get)((0)));
}

static bool s18_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(1)))) || ((!(!((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(3))))) && ((s18_get)((0)));
}

static bool s19_gen(void) {
  return (((!(((!((classifier_cpy) == ((int64_t)(0)))) && ((classifier_cpy) == ((int64_t)(1)))) || (((classifier_cpy) == ((int64_t)(0))) && (!((classifier_cpy) == ((int64_t)(1))))))) && ((classifier_cpy) == ((int64_t)(2)))) || ((((!((classifier_cpy) == ((int64_t)(0)))) && ((classifier_cpy) == ((int64_t)(1)))) || (((classifier_cpy) == ((int64_t)(0))) && (!((classifier_cpy) == ((int64_t)(1)))))) && (!((classifier_cpy) == ((int64_t)(2)))))) && ((s19_get)((0)));
}

static bool s20_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(2)))) || ((!(!((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(3))))) && ((s20_get)((0)));
}

static bool s21_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(1)))) || ((!((distance_to_target_cpy) > ((int64_t)(7)))) || ((OpState_cpy) == ((int64_t)(1))))) && ((s21_get)((0)));
}

static bool s22_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(2)))) || ((!((distance_to_target_cpy) > ((int64_t)(7)))) || ((OpState_cpy) == ((int64_t)(2))))) && ((s22_get)((0)));
}

static bool s23_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(1)))) || ((!((!((distance_to_target_cpy) > ((int64_t)(7)))) && ((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(2))))) && ((s23_get)((0)));
}

static bool s24_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(2)))) || ((!((!((distance_to_target_cpy) > ((int64_t)(7)))) && ((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(3))))) && ((s24_get)((0)));
}

static bool s25_gen(void) {
  return ((!((classifier_cpy) == ((int64_t)(0)))) || ((OpState_cpy) == ((int64_t)(0)))) && ((s25_get)((0)));
}

static bool handleroperationalstate_0_guard(void) {
  return !(((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(0))) && (!((((!(slowdown_cpy)) && (!(halt_cpy))) && (!(alert_cpy))) && (!(turnoffUVC_cpy)))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s1_get)((0)))) && ((((s1_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s2_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s3_get)((0)))) || ((((!(slowdown_cpy)) && (!(halt_cpy))) && (!(alert_cpy))) && (!(turnoffUVC_cpy))))))) && ((s4_get)((0))));
}

static bool handleroperationalstate_1_guard(void) {
  return !(((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(1))) && (!((((!(slowdown_cpy)) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy)))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s5_get)((0)))) && ((((s5_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s6_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s7_get)((0)))) || ((((!(slowdown_cpy)) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy))))))) && ((s8_get)((0))));
}

static bool handleroperationalstate_2_guard(void) {
  return !(((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(2))) && (!((((slowdown_cpy) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy)))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s9_get)((0)))) && ((((s9_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s10_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s11_get)((0)))) || ((((slowdown_cpy) && (!(halt_cpy))) && (alert_cpy)) && (!(turnoffUVC_cpy))))))) && ((s12_get)((0))));
}

static bool handleroperationalstate_3_guard(void) {
  return !(((!(((((s0_get)((0))) - ((int64_t)(1))) <= ((s0_get)((0)))) && (((((s0_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && (((OpState_cpy) == ((int64_t)(3))) && (!((((!(slowdown_cpy)) && (halt_cpy)) && (!(alert_cpy))) && (turnoffUVC_cpy))))) || (((((s0_get)((0))) - ((int64_t)(1))) <= ((s13_get)((0)))) && ((((s13_get)((0))) <= (((s0_get)((0))) - ((int64_t)(1)))) && ((s14_get)((0)))))))) || ((((s0_get)((0))) <= ((s0_get)((0)))) && ((((s0_get)((0))) <= ((s0_get)((0)))) && ((!((s15_get)((0)))) || ((((!(slowdown_cpy)) && (halt_cpy)) && (!(alert_cpy))) && (turnoffUVC_cpy)))))) && ((s16_get)((0))));
}

static bool handlerdtt_assumption_guard(void) {
  return !(((!(!((classifier_cpy) == ((int64_t)(0))))) || ((distance_to_target_cpy) >= ((int64_t)(0)))) && ((s17_get)((0))));
}

static bool handlerstate_req103_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(1)))) || ((!(!((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(3))))) && ((s18_get)((0))));
}

static bool handlerclassifier_assumption_guard(void) {
  return !((((!(((!((classifier_cpy) == ((int64_t)(0)))) && ((classifier_cpy) == ((int64_t)(1)))) || (((classifier_cpy) == ((int64_t)(0))) && (!((classifier_cpy) == ((int64_t)(1))))))) && ((classifier_cpy) == ((int64_t)(2)))) || ((((!((classifier_cpy) == ((int64_t)(0)))) && ((classifier_cpy) == ((int64_t)(1)))) || (((classifier_cpy) == ((int64_t)(0))) && (!((classifier_cpy) == ((int64_t)(1)))))) && (!((classifier_cpy) == ((int64_t)(2)))))) && ((s19_get)((0))));
}

static bool handlerstate_req203_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(2)))) || ((!(!((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(3))))) && ((s20_get)((0))));
}

static bool handlerstate_req101_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(1)))) || ((!((distance_to_target_cpy) > ((int64_t)(7)))) || ((OpState_cpy) == ((int64_t)(1))))) && ((s21_get)((0))));
}

static bool handlerstate_req201_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(2)))) || ((!((distance_to_target_cpy) > ((int64_t)(7)))) || ((OpState_cpy) == ((int64_t)(2))))) && ((s22_get)((0))));
}

static bool handlerstate_req102_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(1)))) || ((!((!((distance_to_target_cpy) > ((int64_t)(7)))) && ((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(2))))) && ((s23_get)((0))));
}

static bool handlerstate_req202_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(2)))) || ((!((!((distance_to_target_cpy) > ((int64_t)(7)))) && ((distance_to_target_cpy) > ((int64_t)(3))))) || ((OpState_cpy) == ((int64_t)(3))))) && ((s24_get)((0))));
}

static bool handlerstate_req000_guard(void) {
  return !(((!((classifier_cpy) == ((int64_t)(0)))) || ((OpState_cpy) == ((int64_t)(0)))) && ((s25_get)((0))));
}

void step(void) {
  int64_t s0_tmp;
  int64_t s1_tmp;
  bool s2_tmp;
  bool s3_tmp;
  bool s4_tmp;
  int64_t s5_tmp;
  bool s6_tmp;
  bool s7_tmp;
  bool s8_tmp;
  int64_t s9_tmp;
  bool s10_tmp;
  bool s11_tmp;
  bool s12_tmp;
  int64_t s13_tmp;
  bool s14_tmp;
  bool s15_tmp;
  bool s16_tmp;
  bool s17_tmp;
  bool s18_tmp;
  bool s19_tmp;
  bool s20_tmp;
  bool s21_tmp;
  bool s22_tmp;
  bool s23_tmp;
  bool s24_tmp;
  bool s25_tmp;
  (OpState_cpy) = (OpState);
  (slowdown_cpy) = (slowdown);
  (halt_cpy) = (halt);
  (alert_cpy) = (alert);
  (turnoffUVC_cpy) = (turnoffUVC);
  (classifier_cpy) = (classifier);
  (distance_to_target_cpy) = (distance_to_target);
  if ((handleroperationalstate_0_guard)()) {
    {(handleroperationalstate_0)();}
  };
  if ((handleroperationalstate_1_guard)()) {
    {(handleroperationalstate_1)();}
  };
  if ((handleroperationalstate_2_guard)()) {
    {(handleroperationalstate_2)();}
  };
  if ((handleroperationalstate_3_guard)()) {
    {(handleroperationalstate_3)();}
  };
  if ((handlerdtt_assumption_guard)()) {
    {(handlerdtt_assumption)();}
  };
  if ((handlerstate_req103_guard)()) {
    {(handlerstate_req103)();}
  };
  if ((handlerclassifier_assumption_guard)()) {
    {(handlerclassifier_assumption)();}
  };
  if ((handlerstate_req203_guard)()) {
    {(handlerstate_req203)();}
  };
  if ((handlerstate_req101_guard)()) {
    {(handlerstate_req101)();}
  };
  if ((handlerstate_req201_guard)()) {
    {(handlerstate_req201)();}
  };
  if ((handlerstate_req102_guard)()) {
    {(handlerstate_req102)();}
  };
  if ((handlerstate_req202_guard)()) {
    {(handlerstate_req202)();}
  };
  if ((handlerstate_req000_guard)()) {
    {(handlerstate_req000)();}
  };
  (s0_tmp) = ((s0_gen)());
  (s1_tmp) = ((s1_gen)());
  (s2_tmp) = ((s2_gen)());
  (s3_tmp) = ((s3_gen)());
  (s4_tmp) = ((s4_gen)());
  (s5_tmp) = ((s5_gen)());
  (s6_tmp) = ((s6_gen)());
  (s7_tmp) = ((s7_gen)());
  (s8_tmp) = ((s8_gen)());
  (s9_tmp) = ((s9_gen)());
  (s10_tmp) = ((s10_gen)());
  (s11_tmp) = ((s11_gen)());
  (s12_tmp) = ((s12_gen)());
  (s13_tmp) = ((s13_gen)());
  (s14_tmp) = ((s14_gen)());
  (s15_tmp) = ((s15_gen)());
  (s16_tmp) = ((s16_gen)());
  (s17_tmp) = ((s17_gen)());
  (s18_tmp) = ((s18_gen)());
  (s19_tmp) = ((s19_gen)());
  (s20_tmp) = ((s20_gen)());
  (s21_tmp) = ((s21_gen)());
  (s22_tmp) = ((s22_gen)());
  (s23_tmp) = ((s23_gen)());
  (s24_tmp) = ((s24_gen)());
  (s25_tmp) = ((s25_gen)());
  ((s0)[s0_idx]) = (s0_tmp);
  ((s1)[s1_idx]) = (s1_tmp);
  ((s2)[s2_idx]) = (s2_tmp);
  ((s3)[s3_idx]) = (s3_tmp);
  ((s4)[s4_idx]) = (s4_tmp);
  ((s5)[s5_idx]) = (s5_tmp);
  ((s6)[s6_idx]) = (s6_tmp);
  ((s7)[s7_idx]) = (s7_tmp);
  ((s8)[s8_idx]) = (s8_tmp);
  ((s9)[s9_idx]) = (s9_tmp);
  ((s10)[s10_idx]) = (s10_tmp);
  ((s11)[s11_idx]) = (s11_tmp);
  ((s12)[s12_idx]) = (s12_tmp);
  ((s13)[s13_idx]) = (s13_tmp);
  ((s14)[s14_idx]) = (s14_tmp);
  ((s15)[s15_idx]) = (s15_tmp);
  ((s16)[s16_idx]) = (s16_tmp);
  ((s17)[s17_idx]) = (s17_tmp);
  ((s18)[s18_idx]) = (s18_tmp);
  ((s19)[s19_idx]) = (s19_tmp);
  ((s20)[s20_idx]) = (s20_tmp);
  ((s21)[s21_idx]) = (s21_tmp);
  ((s22)[s22_idx]) = (s22_tmp);
  ((s23)[s23_idx]) = (s23_tmp);
  ((s24)[s24_idx]) = (s24_tmp);
  ((s25)[s25_idx]) = (s25_tmp);
  (s0_idx) = (((s0_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s1_idx) = (((s1_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s2_idx) = (((s2_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s3_idx) = (((s3_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s4_idx) = (((s4_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s5_idx) = (((s5_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s6_idx) = (((s6_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s7_idx) = (((s7_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s8_idx) = (((s8_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s9_idx) = (((s9_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s10_idx) = (((s10_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s11_idx) = (((s11_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s12_idx) = (((s12_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s13_idx) = (((s13_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s14_idx) = (((s14_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s15_idx) = (((s15_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s16_idx) = (((s16_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s17_idx) = (((s17_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s18_idx) = (((s18_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s19_idx) = (((s19_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s20_idx) = (((s20_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s21_idx) = (((s21_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s22_idx) = (((s22_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s23_idx) = (((s23_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s24_idx) = (((s24_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s25_idx) = (((s25_idx) + ((size_t)(1))) % ((size_t)(1)));
}
