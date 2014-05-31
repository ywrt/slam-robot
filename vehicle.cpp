#include <cstdint>
#include <iostream>
#include <libusb.h>
#include <unistd.h>

#include "usb.h"
#include "vehicle.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;
#include "maestro-protocol.h"
#include "smc-protocol.h"


using namespace std;

// Access to a pololu Maestro servo controller.
class PololuMaestro : public UsbDevice {
 public:
  PololuMaestro(Usb* usb) : UsbDevice(usb) { }
  virtual ~PololuMaestro() {}

  void Init() {
    unsigned short productIdArray[] = {0x0089, 0x008a, 0x008b, 0x008c};
    UsbDevice::Init(0x1ffb, productIdArray);
  }

  // target is in [-1, 1]
  bool SetTarget(int servo, double target) {
    if (!device_handle_)
      return false;

    uint16_t value = target * 4 * 500 + 6000;
    int ret = libusb_control_transfer(device_handle_, 0x40,
        REQUEST_SET_TARGET, value, servo,
        0, 0, (ushort)5000);
    cout << value << " gave " << ret << " ret\n";
    return true;
  }
};

// Access to a pololu simple motor controller over USB.
class PololuSMC : public UsbDevice {
 public:
  PololuSMC(Usb* usb) : UsbDevice(usb) {}
  virtual ~PololuSMC() {}

  void Init() {
    unsigned short productIdArray[]={0x0098, 0x009a, 0x009c, 0x009e, 0x00a1};

    UsbDevice::Init(0x1ffb, productIdArray);
  }

  // speed is in [-1, 1]
  bool SetSpeed(double speed) {
    if (!device_handle_) return false;
    int direction = DIRECTION_FORWARD;
    if (speed < 0) {
      speed = -speed;
      direction = DIRECTION_REVERSE;  // Reverse.
    }

    uint16_t value = speed * 3200;
    int ret = libusb_control_transfer(device_handle_, 0x40,
        REQUEST_SET_SPEED, value, direction, 0, 0, (ushort)5000);
    cout << "SetSpeed to " << value << " gave " << ret << "\n";
    return true;
  }

  void resume() {
    // Exit safe start.
    libusb_control_transfer(device_handle_, 0x40,
        REQUEST_EXIT_SAFE_START, 0, 0, 0, 0, (ushort)5000);
    // Turn off USB kill
    libusb_control_transfer(device_handle_, 0x40,
        REQUEST_SET_USB_KILL, 0, 0, 0, 0, (ushort)5000);
  }

  void stop() {
    libusb_control_transfer(device_handle_, 0x40,
        REQUEST_SET_USB_KILL, 1, 0, 0, 0, (ushort)5000);
  }
};

Vehicle::Vehicle() : motor_(new PololuSMC(&usb_)), servos_(new PololuMaestro(&usb_)) {
  motor_->Init();
  motor_->resume();
  servos_->Init();
}

Vehicle::~Vehicle() {
  Stop();
}

void Vehicle::Stop() {
    servos_->SetTarget(0, 0);
    servos_->SetTarget(1, 0);

    motor_->SetSpeed(0);
    motor_->stop();
}

// Set speed. 0.18 seems good.
void Vehicle::Speed(double d) {
    motor_->SetSpeed(d);
}

// Set turn amount. 0.5 seems good too.
void Vehicle::Turn(double d) {
  servos_->SetTarget(0, d);
  servos_->SetTarget(1, -d);
}
