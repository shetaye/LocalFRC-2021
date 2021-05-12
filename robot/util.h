#ifndef UTIL_H
#define UTIL_H
// Numeric Utilities
template <typename T> T clamp(T val, T min, T max) {
  if (val < min) { return min; }
  if (val > max) { return max; }
  return val;
}
template <typename T> T map(T val, T from_min, T from_max, T to_min, T to_max) {
  return (val - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}
template <typename T> T deadzone(T val, T mag) {
  if (val > -mag && val < mag) { return (T)0; }
  return val;
}
#endif
