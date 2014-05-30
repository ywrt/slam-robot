#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <memory>

#include "usb.h"

class PololuSMC;
class PololuMaestro;

class Vehicle {
 public:
  Vehicle();
  ~Vehicle();

  void Stop();
  void Turn(double d);
  void Speed(double d);
 private:
  Usb usb_;
  std::unique_ptr<PololuSMC> motor_;
  std::unique_ptr<PololuMaestro> servos_;
};

#endif
