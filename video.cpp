#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <glog/logging.h>

#include <linux/videodev2.h>

#include "video.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

struct VideoDev::Buffer {
  void   *start;
  size_t  length;
};

namespace {

int xioctl(int fh, int request, void *arg) {
  int r;

  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

bool DequeueBuffer(int fd, struct v4l2_buffer* buf) {
  CLEAR(*buf);
  buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf->memory = V4L2_MEMORY_MMAP;

  while (true) {
    // Try and de-queue a buffer.
    if (xioctl(fd, VIDIOC_DQBUF, buf) == 0) {
      return true;
    }
    if (errno != EAGAIN) {
      perror("DQBUF");
      return false;
    }

    // No buffer. Sleeping awaiting one.
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);
    if (r == 0) {
      fprintf(stderr, "Select timeout\n");
      return false;
    }
  }
}

int open_device(const char* dev_name) {
  struct stat st;

  if (-1 == stat(dev_name, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n",
       dev_name, errno, strerror(errno));
    return -1;
  }

  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no device\n", dev_name);
    return -1;
  }

  int fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n",
       dev_name, errno, strerror(errno));
    return -1;
  }

  struct v4l2_capability cap;
  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n",
         dev_name);
      return -1;
    }
    perror("VIDIOC_QUERYCAP");
    return -1;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n",
       dev_name);
    return -1;
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    fprintf(stderr, "%s does not support streaming i/o\n",
       dev_name);
    return -1;
  }

  // Disable cropping (allowed to silently fail if cropping
  // isn't supported).
  struct v4l2_cropcap cropcap;
  CLEAR(cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_CROPCAP, &cropcap) == 0) {
    struct v4l2_crop crop;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    xioctl(fd, VIDIOC_S_CROP, &crop);
  }

  // Setup video format.
  struct v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = 640;
  fmt.fmt.pix.height      = 480;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
    perror("VIDIOC_S_FMT");
    return -1;
  }

  // Setup streaming parameters.
  struct v4l2_streamparm parm;
  CLEAR(parm);
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = 5;
  if (-1 == xioctl(fd, VIDIOC_S_PARM, &parm)) {
    perror("VIDIOC_S_FMT");
    return -1;
  }


  /* Buggy driver paranoia. */
  unsigned int min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  return fd;
}


}  // namespace

bool VideoDev::GetObservation(int cam, int id, cv::Mat* mat) {
  struct v4l2_buffer buf;
  if (!DequeueBuffer(fd_, &buf)) {
    fprintf(stderr, "Failed to Dequeue buffer\n");
    return false;
  }

  CHECK_LT(buf.index, buffers_.size());

  Buffer* b = buffers_[buf.index];
  int bytes = buf.bytesused;

  // TODO: Actually fill in the Mat. And probably convert from YUYV to RGB?
  *mat = cv::Mat(480, 640, CV_8UC3);

#define SAT(c) \
     if (c & (~255)) { if (c < 0) c = 0; else c = 255; }

  // Convert from YUYV to RGB.
  unsigned char * out = mat->ptr(0);
  for (int i = 0; i < bytes; i += 4) {
    unsigned char *in = ((unsigned char*)b->start) + i;
    int y1 = in[0];
    int cb = ((in[1] - 128) * 454) >> 8;
    int cg = (in[1] - 128) * 88;

    int y2 = in[2];
    int cr = ((in[3] - 128) * 359) >> 8;
    cg = (cg + (in[3] - 128) * 183) >> 8;

    int r = y1 + cr;
    int b = y1 + cb;
    int g = y1 - cg;

    SAT(r);
    SAT(g);
    SAT(b);
    *out++ = b;
    *out++ = g;
    *out++ = r;

    r = y2 + cr;
    b = y2 + cb;
    g = y2 - cg;

    SAT(r);
    SAT(g);
    SAT(b);
    *out++ = b;
    *out++ = g;
    *out++ = r;
  }

  // Re-queue buffer.
  if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
    fprintf(stderr, "Failed to re-queue buffer.\n");
    return false;
  }

  return true;
}

VideoDev::~VideoDev() {
  enum v4l2_buf_type type;

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
    fprintf(stderr, "Failed to call STREAMOFF\n");
  }

  for (const auto& buffer : buffers_) {
    if (munmap(buffer->start, buffer->length) != -1) {
      continue;
    }
    fprintf(stderr, "Failed to unmap buffer\n");
  }
  for (const auto& buffer : buffers_) {
    delete buffer;
  }

  close(fd_);
}

bool VideoDev::Init() {
  fd_ = open_device(device_);
  if (fd_ < 0) {
    fprintf(stderr, "Failed to open video device\n");
    return false;
  }

  // Request a number of MMAP'ed buffers.
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = num_buf_;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support "
         "memory mapping\n", device_);
      return false;
    }
    fprintf(stderr, "Failed to request buffers\n");
    return false;
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n",
       device_);
    return false;
  }

  // mmap the buffers into our address space.
  for (unsigned int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = i;

    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      fprintf(stderr, "QUERYBUF failed\n");
      return false;
    }

    Buffer* buffer = new Buffer;
    buffer->length = buf.length;
    buffer->start = 
      mmap(NULL /* start anywhere */,
      buf.length,
      PROT_READ | PROT_WRITE /* required */,
      MAP_SHARED /* recommended */,
      fd_, buf.m.offset);

    if (buffer->start == MAP_FAILED) {
      fprintf(stderr, "Failed to mmap buffer.\n");
      return false;
    }

    buffers_.push_back(buffer);
  }

  // queue the buffers, ready for receiving video frames.
  for (unsigned int i = 0; i < buffers_.size(); ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
      perror("Failed to queue buffer: VIDIOC_QBUF");
      return false;
    }
    printf("Queued %d\n", i);
  }

  // Turn on video streaming.
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) {
    perror("Failed to start streaming: VIDIOC_STREAMON");
    return false;
  }

  return true;
}
