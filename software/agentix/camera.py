"""
camera -- the robot's eye. One USB webcam, read as MJPEG over V4L2.

WHY NO LIBRARY. OpenCV is ~40 MB and a lot of RAM on a 1 GB Pi 3 just to
hand over JPEGs the camera already compressed. The camera does the work:
it emits finished JPEG frames, and all this module does is ask the kernel
for them (four ioctls and an mmap) and keep the newest one. No decode, no
re-encode, almost no CPU. Standard library only.

ONE OWNER. A V4L2 device streams to one reader. Everything -- the live
view in the browser, the agent's `look` tool, any number of viewers --
goes through the single Camera object here, which keeps the latest frame
and hands out copies.

ON DEMAND. Capture starts on the first request and stops IDLE_STOP_S
after the last one, so an unwatched robot is not streaming 720p over USB
and spending power it does not have to spare.

Self-test on the Pi (the web app must not be streaming at the time):
    python3 agentix/camera.py            # grabs one frame to /tmp
"""

import ctypes
import fcntl
import glob
import mmap
import os
import select
import struct
import threading
import time

WIDTH, HEIGHT = 1280, 720
FPS_STREAM = 8          # frames/s sent to each browser; the camera runs faster
IDLE_STOP_S = 30.0      # stop capturing this long after the last request
N_BUFFERS = 4
DEVICE_GLOB = "/dev/v4l/by-id/*-video-index0"

# ---- V4L2 ABI (linux/videodev2.h), 64-bit layout -----------------------
_BUF_TYPE_VIDEO_CAPTURE = 1
_MEMORY_MMAP = 1
_PIX_FMT_MJPEG = int.from_bytes(b"MJPG", "little")
_FIELD_ANY = 0

_FORMAT_SIZE = 208      # struct v4l2_format: u32 type, pad, 200-byte union
_REQBUFS_SIZE = 20      # struct v4l2_requestbuffers
_BUFFER_SIZE = 88       # struct v4l2_buffer
_B_INDEX, _B_TYPE, _B_BYTESUSED = 0, 4, 8
_B_MEMORY, _B_OFFSET, _B_LENGTH = 60, 64, 72


def _iowr(nr, size):
    return (3 << 30) | (size << 16) | (ord("V") << 8) | nr


def _iow(nr, size):
    return (1 << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_S_FMT = _iowr(5, _FORMAT_SIZE)
VIDIOC_REQBUFS = _iowr(8, _REQBUFS_SIZE)
VIDIOC_QUERYBUF = _iowr(9, _BUFFER_SIZE)
VIDIOC_QBUF = _iowr(15, _BUFFER_SIZE)
VIDIOC_DQBUF = _iowr(17, _BUFFER_SIZE)
VIDIOC_STREAMON = _iow(18, 4)
VIDIOC_STREAMOFF = _iow(19, 4)


class CameraError(Exception):
    pass


def find_device():
    """The USB camera's capture node. /dev/v4l/by-id survives the Pi's own
    codec/ISP nodes shuffling /dev/videoN numbers between boots."""
    found = sorted(glob.glob(DEVICE_GLOB))
    if found:
        return found[0]
    return "/dev/video0" if os.path.exists("/dev/video0") else None


def _buffer(index=0):
    b = bytearray(_BUFFER_SIZE)
    struct.pack_into("<II", b, _B_INDEX, index, _BUF_TYPE_VIDEO_CAPTURE)
    struct.pack_into("<I", b, _B_MEMORY, _MEMORY_MMAP)
    return b


def _complete(jpeg):
    """A plausible whole JPEG: SOI at the front, EOI near the end. USB
    hiccups deliver truncated frames now and then; better to skip one
    than to show half a picture or send one to the model."""
    return (len(jpeg) > 1000 and jpeg[:2] == b"\xff\xd8"
            and b"\xff\xd9" in jpeg[-16:])


class _Stream:
    """One open capture session: format set, buffers mapped, streaming."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self.maps = []
        try:
            self._setup()
        except Exception:
            self.close()
            raise

    def _setup(self):
        fmt = bytearray(_FORMAT_SIZE)
        struct.pack_into("<I", fmt, 0, _BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into("<IIII", fmt, 8, WIDTH, HEIGHT, _PIX_FMT_MJPEG,
                         _FIELD_ANY)
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)
        w, h, pix = struct.unpack_from("<III", fmt, 8)
        if pix != _PIX_FMT_MJPEG:
            raise CameraError("camera does not offer MJPEG")
        self.size = (w, h)

        req = bytearray(_REQBUFS_SIZE)
        struct.pack_into("<III", req, 0, N_BUFFERS, _BUF_TYPE_VIDEO_CAPTURE,
                         _MEMORY_MMAP)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)
        count = struct.unpack_from("<I", req, 0)[0]
        if count < 2:
            raise CameraError(f"camera gave {count} buffer(s)")

        for i in range(count):
            b = _buffer(i)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
            offset = struct.unpack_from("<I", b, _B_OFFSET)[0]
            length = struct.unpack_from("<I", b, _B_LENGTH)[0]
            self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                                       mmap.PROT_READ | mmap.PROT_WRITE,
                                       offset=offset))
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b)

        fcntl.ioctl(self.fd, VIDIOC_STREAMON,
                    ctypes.c_int(_BUF_TYPE_VIDEO_CAPTURE))

    def read(self, timeout=2.0):
        """The next frame's JPEG bytes, or None on timeout."""
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return None
        b = _buffer()
        fcntl.ioctl(self.fd, VIDIOC_DQBUF, b)
        index = struct.unpack_from("<I", b, _B_INDEX)[0]
        used = struct.unpack_from("<I", b, _B_BYTESUSED)[0]
        data = bytes(self.maps[index][:used])
        fcntl.ioctl(self.fd, VIDIOC_QBUF, b)
        return data

    def close(self):
        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF,
                        ctypes.c_int(_BUF_TYPE_VIDEO_CAPTURE))
        except OSError:
            pass
        for m in self.maps:
            try:
                m.close()
            except (BufferError, ValueError):
                pass
        self.maps = []
        try:
            os.close(self.fd)
        except OSError:
            pass


class Camera:
    """Latest-frame holder with an on-demand capture thread."""

    def __init__(self, path=None):
        self.path = path
        self._lock = threading.Lock()
        self._fresh = threading.Condition(self._lock)
        self._frame = None
        self._frame_t = 0.0
        self._seq = 0
        self._wanted_t = 0.0
        self._thread = None
        self.size = None
        self.error = None
        self.stats = {"frames": 0, "incomplete": 0, "timeouts": 0}

    # -- state -----------------------------------------------------------
    def describe(self):
        path = self.path or find_device()
        running = bool(self._thread and self._thread.is_alive())
        return {"device": path, "running": running,
                "size": list(self.size) if self.size else None,
                "error": self.error,
                "stats": dict(self.stats),
                "frame_age_s": (round(time.monotonic() - self._frame_t, 1)
                                if self._frame else None)}

    # -- consumers -------------------------------------------------------
    def _want(self):
        """Mark demand and make sure the capture thread is running."""
        with self._lock:
            self._want_locked()

    def _want_locked(self):
        self._wanted_t = time.monotonic()
        if self._alive():
            return
        self.error = None
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="camera")
        self._thread.start()

    def _alive(self):
        return bool(self._thread and self._thread.is_alive())

    def snapshot(self, timeout=5.0, max_age=0.5):
        """One recent JPEG. Raises CameraError if none arrives in time,
        with the capture thread's own reason when it has one."""
        deadline = time.monotonic() + timeout
        with self._fresh:
            self._want_locked()
            while True:
                if (self._frame is not None
                        and time.monotonic() - self._frame_t <= max_age):
                    return self._frame
                if self.error and not self._alive():
                    raise CameraError(self.error)
                left = deadline - time.monotonic()
                if left <= 0:
                    raise CameraError("no frame from camera")
                if not self._alive():
                    self._want_locked()     # it idled out just now
                self._fresh.wait(min(left, 0.5))

    def frames(self, fps=FPS_STREAM):
        """Yield JPEGs, at most `fps` a second, for as long as it is read.
        The generator keeps the camera wanted; when the client goes away
        it stops being iterated and the idle timer takes over."""
        gap = 1.0 / fps
        last_seq = -1
        while True:
            self._want()
            with self._fresh:
                self._fresh.wait_for(lambda: self._seq != last_seq or
                                     (self.error and not self._thread.is_alive()),
                                     timeout=3.0)
                if self._seq == last_seq:
                    if self.error:
                        raise CameraError(self.error)
                    continue
                frame, last_seq = self._frame, self._seq
            yield frame
            time.sleep(gap)

    # -- producer --------------------------------------------------------
    def _run(self):
        path = self.path or find_device()
        if not path:
            self.error = "no camera plugged in"
            with self._fresh:
                self._fresh.notify_all()
            return
        try:
            stream = _Stream(path)
        except (OSError, CameraError) as e:
            self.error = f"cannot open {path}: {e}"
            with self._fresh:
                self._fresh.notify_all()
            return
        self.size = stream.size
        misses = 0
        try:
            while time.monotonic() - self._wanted_t < IDLE_STOP_S:
                try:
                    jpeg = stream.read()
                except OSError as e:
                    self.error = f"camera read failed: {e}"
                    break
                if jpeg is None:
                    misses += 1
                    self.stats["timeouts"] += 1
                    if misses >= 3:
                        self.error = "camera stopped sending frames"
                        break
                    continue
                misses = 0
                if not _complete(jpeg):
                    self.stats["incomplete"] += 1
                    continue
                self.stats["frames"] += 1
                with self._fresh:
                    self._frame = jpeg
                    self._frame_t = time.monotonic()
                    self._seq += 1
                    self._fresh.notify_all()
        finally:
            stream.close()
            with self._fresh:
                self._frame = None
                self._fresh.notify_all()


def look(camera):
    """The agent's eye: one fresh frame plus what it needs to read it.

    The JPEG travels under "image_jpeg" as raw bytes; agent.py turns that
    into an image block for the model rather than a string."""
    jpeg = camera.snapshot()
    w, h = camera.size or (WIDTH, HEIGHT)
    return {"image_jpeg": jpeg, "width": w, "height": h,
            "bytes": len(jpeg)}


if __name__ == "__main__":
    cam = Camera()
    t0 = time.monotonic()
    jpg = cam.snapshot()
    dht = "present" if b"\xff\xc4" in jpg[:2000] else "MISSING"
    print(f"device {cam.describe()['device']}  size {cam.size}  "
          f"{len(jpg)} bytes in {time.monotonic() - t0:.2f} s  DHT {dht}")
    n, t0 = 0, time.monotonic()
    for _ in cam.frames(fps=1000):
        n += 1
        if n == 30:
            break
    print(f"{n / (time.monotonic() - t0):.1f} frames/s from the camera")
    with open("/tmp/crabora_eye.jpg", "wb") as f:
        f.write(jpg)
    print("wrote /tmp/crabora_eye.jpg")
