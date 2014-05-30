#ifndef USB_H_
#define USB_H_
#include <cstdint>
#include <iostream>
#include <libusb.h>
#include <unistd.h>

// Holds a USB context. (for libusb-1.0)
class Usb {
 public:
  ~Usb() {
    if (!context_)
      return;
    libusb_exit(context_);
    context_ = nullptr;
  }

  Usb() {
    libusb_init(&context_);
  }

  libusb_context* context() { return context_; }

 private:
  libusb_context* context_;
};

// A generic usb device, accessed via libusb-1.0
class UsbDevice {
 public:
  UsbDevice(Usb* usb) : usb_(usb), device_handle_(nullptr) { }
  virtual ~UsbDevice() {
    if (!device_handle_)
      return;
    libusb_close(device_handle_);
    device_handle_ = nullptr;
  }

  // return true if the vendor and product id match.
  bool deviceMatchesVendorProduct(libusb_device *device, unsigned short idVendor, unsigned short idProduct) {
      libusb_device_descriptor desc;
      libusb_get_device_descriptor(device, &desc);
      return idVendor == desc.idVendor && idProduct == desc.idProduct;
  }

  void Init(unsigned short vendorId,
            unsigned short productIDArray[]) {
    libusb_device **device_list=0;
    int count=libusb_get_device_list(usb_->context(), &device_list);
    for(int i=0;i<count;i++) {
      libusb_device *device=device_list[i];
      for(int Id=0;Id<4;Id++) {
        if(deviceMatchesVendorProduct(device,
            vendorId, productIDArray[Id])) {
          libusb_open(device, &device_handle_);
          goto done;
        }
      }
    }
    std::cout << "Failed to find a suitable USB device\n";

   done:
    libusb_free_device_list(device_list, 0);
  }

 protected:
  Usb* usb_;
  libusb_device_handle *device_handle_;
};

#endif  // USB_H_
