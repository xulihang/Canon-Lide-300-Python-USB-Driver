#!/usr/bin/env python3
"""
Canon LiDE 300 Scanner Control Script
======================================
Based on sane-backends pixma_mp150.c protocol implementation.
Uses pyusb (libusb) for USB communication.
Generation 4+ pixma protocol with XML dialog messages.

USB Driver Setup (Windows):
  Use Zadig (https://zadig.akeo.ie/) to install WinUSB or libusb-win32
  driver for the Canon LiDE 300 device.

Dependencies:
  pip install pyusb

Usage:
  python scan.py                         # 300 DPI color scan
  python scan.py --dpi 600 --mode gray   # 600 DPI grayscale
  python scan.py --dpi 150 -o photo.bmp  # 150 DPI to photo.bmp
"""

import struct
import sys
import time
import math
import argparse

try:
    import usb.core
    import usb.util
except ImportError:
    print("ERROR: pyusb is required.  pip install pyusb")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════════════════
CANON_VID    = 0x04A9
LIDE300_PID  = 0x1913

# Binary protocol commands (big-endian u16)
CMD_START_SESSION = 0xDB20
CMD_ABORT_SESSION = 0xEF20
CMD_GAMMA         = 0xEE20
CMD_SCAN_PARAM_3  = 0xD820
CMD_SCAN_START_3  = 0xD920
CMD_STATUS_3      = 0xDA20
CMD_READ_IMAGE    = 0xD420
CMD_ERROR_INFO    = 0xFF20

# Response status codes
STATUS_OK     = 0x0606
STATUS_BUSY   = 0x1414
STATUS_FAILED = 0x1515

# Protocol geometry
CMD_HEADER_LEN    = 16
RES_HEADER_LEN    = 8
CMD_LEN_FIELD_OFS = 14
IMAGE_BLOCK_SIZE  = 512 * 1024   # 524288 bytes

# Timeouts (ms)
BULK_TIMEOUT = 5000
XML_TIMEOUT  = 10000
IMG_TIMEOUT  = 30000

# LiDE 300 specs
LIDE300_MAX_XDPI    = 2400
LIDE300_MIN_XDPI    = 300
LIDE300_WIDTH_75DPI = 638    # max scannable width  in pixels @ 75 DPI
LIDE300_HEIGHT_75DPI = 877   # max scannable height in pixels @ 75 DPI

# ═══════════════════════════════════════════════════════════════════════════
#  XML Dialog Messages (Generation 4+ Pixma Protocol)
# ═══════════════════════════════════════════════════════════════════════════
XML_START_1 = (
    '<?xml version="1.0" encoding="utf-8" ?>'
    '<cmd xmlns:ivec="http://www.canon.com/ns/cmd/2008/07/common/">'
    '<ivec:contents><ivec:operation>StartJob</ivec:operation>'
    '<ivec:param_set servicetype="scan">'
    '<ivec:jobID>00000001</ivec:jobID>'
    '<ivec:bidi>1</ivec:bidi>'
    '</ivec:param_set></ivec:contents></cmd>'
)

XML_START_2 = (
    '<?xml version="1.0" encoding="utf-8" ?>'
    '<cmd xmlns:ivec="http://www.canon.com/ns/cmd/2008/07/common/"'
    ' xmlns:vcn="http://www.canon.com/ns/cmd/2008/07/canon/">'
    '<ivec:contents><ivec:operation>VendorCmd</ivec:operation>'
    '<ivec:param_set servicetype="scan">'
    '<ivec:jobID>00000001</ivec:jobID>'
    '<vcn:ijoperation>ModeShift</vcn:ijoperation>'
    '<vcn:ijmode>1</vcn:ijmode>'
    '</ivec:param_set></ivec:contents></cmd>'
)

XML_END = (
    '<?xml version="1.0" encoding="utf-8" ?>'
    '<cmd xmlns:ivec="http://www.canon.com/ns/cmd/2008/07/common/">'
    '<ivec:contents><ivec:operation>EndJob</ivec:operation>'
    '<ivec:param_set servicetype="scan">'
    '<ivec:jobID>00000001</ivec:jobID>'
    '</ivec:param_set></ivec:contents></cmd>'
)

import re as _re
_XML_OK_RE = _re.compile(r'<ivec:response[^>]*>\s*OK\s*</ivec:response>', _re.IGNORECASE)


# ═══════════════════════════════════════════════════════════════════════════
#  Utility helpers
# ═══════════════════════════════════════════════════════════════════════════
def align_up(x, n):
    """Round *x* up to the nearest multiple of *n*."""
    return ((x + n - 1) // n) * n


def make_checksum(data_bytes):
    """Return one byte so that ``sum(data_bytes + [byte]) & 0xFF == 0``."""
    return (-sum(data_bytes)) & 0xFF


def build_gamma_table(gamma=1.0, n=1024):
    """1024-entry, 16-bit LE gamma LUT (2048 bytes) as used by generation 2+
    pixma scanners.  With gamma=1.0 this produces a linear ramp 0…65535."""
    tbl = bytearray(n * 2)
    r = 1.0 / gamma
    sc_in = 1.0 / (n - 1)
    for i in range(n):
        v = int(65535.0 * math.pow(i * sc_in, r) + 0.5)
        v = min(v, 65535)
        tbl[2 * i]     = v & 0xFF
        tbl[2 * i + 1] = (v >> 8) & 0xFF
    return tbl


def write_bmp(path, width, height, rgb_data, channels=3):
    """Write a 24-bit (RGB) or 8-bit (gray) BMP file.
    *rgb_data* is raw pixel bytes, top-to-bottom, left-to-right."""
    bpp = channels * 8
    row_bytes = width * channels
    pad = (4 - row_bytes % 4) % 4
    padded = row_bytes + pad

    palette_size = 256 * 4 if channels == 1 else 0
    pixel_offset = 54 + palette_size
    file_size = pixel_offset + padded * height

    with open(path, 'wb') as f:
        # -- BMP file header (14 bytes) --
        f.write(b'BM')
        f.write(struct.pack('<I', file_size))
        f.write(struct.pack('<HH', 0, 0))
        f.write(struct.pack('<I', pixel_offset))
        # -- DIB header (BITMAPINFOHEADER, 40 bytes) --
        f.write(struct.pack('<I', 40))
        f.write(struct.pack('<i', width))
        f.write(struct.pack('<i', -height))        # negative → top-down
        f.write(struct.pack('<HH', 1, bpp))
        f.write(struct.pack('<I', 0))              # BI_RGB
        f.write(struct.pack('<I', padded * height))
        f.write(struct.pack('<ii', 2835, 2835))    # ~72 DPI
        f.write(struct.pack('<II', 0, 0))
        # -- palette (grayscale only) --
        if channels == 1:
            for g in range(256):
                f.write(bytes([g, g, g, 0]))
        # -- pixel data --
        zero_pad = b'\x00' * pad
        for y in range(height):
            off = y * row_bytes
            if off + row_bytes <= len(rgb_data):
                row = bytearray(rgb_data[off:off + row_bytes])
            else:
                avail = max(0, len(rgb_data) - off)
                row = bytearray(rgb_data[off:off + avail]) if avail else bytearray()
                row += b'\xff' * (row_bytes - len(row))
            if channels == 3:
                # BMP stores BGR
                for x in range(width):
                    p = x * 3
                    if p + 2 < len(row):
                        row[p], row[p + 2] = row[p + 2], row[p]
            f.write(row)
            f.write(zero_pad)

    print(f"  Saved {path}  ({width} x {height}, {'color' if channels==3 else 'gray'})")


def hexdump(data, n=64):
    """Return a short hex representation of *data* (first *n* bytes)."""
    snippet = data[:n]
    hexstr = ' '.join(f'{b:02X}' for b in snippet)
    if len(data) > n:
        hexstr += ' ...'
    return hexstr


# ═══════════════════════════════════════════════════════════════════════════
#  Scanner class
# ═══════════════════════════════════════════════════════════════════════════
class CanonLiDE300:
    """Control a Canon LiDE 300 via USB using the pixma binary+XML protocol."""

    def __init__(self):
        self.dev = None
        self.ep_out = None          # bulk OUT
        self.ep_in  = None          # bulk IN
        self.ep_int = None          # interrupt IN (optional)
        self.connected = False
        self.generation = 5         # LiDE 300 → generation 5
        self.current_status = bytearray(16)
        self.last_block = 0

    # ── USB connection ─────────────────────────────────────────────────
    def connect(self):
        print(f"Searching for Canon LiDE 300 "
              f"(VID=0x{CANON_VID:04X} PID=0x{LIDE300_PID:04X}) ...")
        self.dev = usb.core.find(idVendor=CANON_VID, idProduct=LIDE300_PID)
        if self.dev is None:
            print("ERROR: scanner not found.\n"
                  "  * Is it plugged in and powered on?\n"
                  "  * On Windows run Zadig to install the WinUSB driver.\n"
                  "  * On Linux run with sudo or set up udev rules.")
            return False
        try:
            # Print USB device info for diagnostics
            print(f"  Found: bus {self.dev.bus} addr {self.dev.address}")

            # Detach kernel driver from ALL interfaces (Linux: usblp, ippusbxd)
            cfg = self.dev.get_active_configuration()
            if cfg is None:
                self.dev.set_configuration()
                cfg = self.dev.get_active_configuration()
            for intf in cfg:
                ifnum = intf.bInterfaceNumber
                try:
                    if self.dev.is_kernel_driver_active(ifnum):
                        print(f"  Detaching kernel driver from interface {ifnum}")
                        self.dev.detach_kernel_driver(ifnum)
                except (usb.core.USBError, NotImplementedError):
                    pass

            # Reset device to clear stale state
            try:
                self.dev.reset()
                time.sleep(0.5)
                # Re-find after reset
                self.dev = usb.core.find(idVendor=CANON_VID, idProduct=LIDE300_PID)
                if self.dev is None:
                    print("ERROR: device lost after reset")
                    return False
                # Detach again after reset
                cfg = self.dev.get_active_configuration()
                if cfg is None:
                    self.dev.set_configuration()
                    cfg = self.dev.get_active_configuration()
                for intf in cfg:
                    ifnum = intf.bInterfaceNumber
                    try:
                        if self.dev.is_kernel_driver_active(ifnum):
                            self.dev.detach_kernel_driver(ifnum)
                    except (usb.core.USBError, NotImplementedError):
                        pass
            except usb.core.USBError as e:
                print(f"  Reset skipped: {e}")

            self.dev.set_configuration()
            cfg = self.dev.get_active_configuration()

            # Print all interfaces and endpoints for diagnostics
            for intf in cfg:
                print(f"  Interface {intf.bInterfaceNumber} "
                      f"alt={intf.bAlternateSetting} "
                      f"class=0x{intf.bInterfaceClass:02X}")
                for ep in intf:
                    d = "IN" if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN else "OUT"
                    t = {0: "CTRL", 1: "ISO", 2: "BULK", 3: "INTR"}[usb.util.endpoint_type(ep.bmAttributes)]
                    print(f"    EP 0x{ep.bEndpointAddress:02X} {d} {t} "
                          f"maxpkt={ep.wMaxPacketSize}")

            intf = cfg[(0, 0)]
            self.ep_out = usb.util.find_descriptor(
                intf, custom_match=lambda e:
                    usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
                    and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
            self.ep_in = usb.util.find_descriptor(
                intf, custom_match=lambda e:
                    usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
                    and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
            self.ep_int = usb.util.find_descriptor(
                intf, custom_match=lambda e:
                    usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
                    and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_INTR)
            if not self.ep_out or not self.ep_in:
                print("ERROR: required bulk endpoints not found on interface 0.")
                # Try other interfaces
                for intf2 in cfg:
                    if intf2.bInterfaceNumber == 0:
                        continue
                    ep_o = usb.util.find_descriptor(
                        intf2, custom_match=lambda e:
                            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
                            and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
                    ep_i = usb.util.find_descriptor(
                        intf2, custom_match=lambda e:
                            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
                            and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
                    if ep_o and ep_i:
                        print(f"  -> Using interface {intf2.bInterfaceNumber} instead")
                        self.ep_out = ep_o
                        self.ep_in = ep_i
                        break
            if not self.ep_out or not self.ep_in:
                print("ERROR: no usable bulk endpoints found on any interface.")
                return False

            # Explicitly claim the interface
            try:
                usb.util.claim_interface(self.dev, self.ep_out.interface_number
                                         if hasattr(self.ep_out, 'interface_number') else 0)
            except usb.core.USBError:
                pass

            self.connected = True
            print(f"  Connected.  OUT=0x{self.ep_out.bEndpointAddress:02X}  "
                  f"IN=0x{self.ep_in.bEndpointAddress:02X}")

            # Flush stale bulk data
            self._flush_bulk()

            return True
        except usb.core.USBError as e:
            print(f"USB error during connect: {e}")
            return False

    def disconnect(self):
        if self.dev:
            try:
                usb.util.dispose_resources(self.dev)
            except Exception:
                pass
        self.connected = False
        print("Disconnected.")

    # ── low-level USB I/O ──────────────────────────────────────────────
    def _write(self, data, timeout=BULK_TIMEOUT):
        self.ep_out.write(data, timeout=timeout)

    def _read(self, size, timeout=BULK_TIMEOUT):
        return bytes(self.ep_in.read(size, timeout=timeout))

    def _flush_bulk(self):
        """Drain any stale data sitting in the bulk-IN pipe."""
        for _ in range(8):
            try:
                self.ep_in.read(512, timeout=100)
            except usb.core.USBError:
                break

    def _clear_interrupts(self):
        if not self.ep_int:
            return
        for _ in range(16):
            try:
                self.ep_int.read(64, timeout=50)
            except usb.core.USBError:
                break

    # ── XML dialog layer ───────────────────────────────────────────────
    def _xml_dialog(self, xml_str):
        """Send an XML request, read XML response, return True if OK.
        Uses SANE-style retry (1s timeout × 8 retries).
        Returns (ok, response_detail) tuple."""
        payload = xml_str.encode('utf-8')
        op = _re.search(r'<ivec:operation>(\w+)</ivec:operation>', xml_str)
        op_name = op.group(1) if op else xml_str[:60]

        self._write(payload, timeout=XML_TIMEOUT)

        # Read response with retry (like pixma_cmd_transaction: rec_tmo=8)
        resp = None
        for attempt in range(8):
            try:
                resp = self._read(4096, timeout=1500)
                break
            except usb.core.USBError as e:
                is_timeout = 'timeout' in str(e).lower() or getattr(e, 'errno', 0) in (110, -116)
                if is_timeout:
                    print(f"  XML {op_name}: no response yet (retry {attempt+1}/8)")
                    continue
                raise

        if resp is None:
            print(f"  XML {op_name}: TIMEOUT (no response after 8 retries)")
            return False, "Timeout"

        resp_text = resp.decode('utf-8', errors='replace')
        ok = bool(_XML_OK_RE.search(resp_text))

        # Extract response_detail if present
        detail_m = _re.search(
            r'<ivec:response_detail[^>]*>\s*(.*?)\s*</ivec:response_detail>',
            resp_text, _re.DOTALL)
        detail = detail_m.group(1).strip() if detail_m else ""

        tag = "OK" if ok else "FAIL"
        print(f"  XML {op_name}: {tag}" + (f"  ({detail})" if detail else ""))
        if not ok:
            print(f"       response ({len(resp_text)} chars):\n{resp_text}")
        return ok, detail

    # ── binary command helpers ─────────────────────────────────────────
    def _exec(self, cmd_code, dataout=None, datain_len=0, timeout=BULK_TIMEOUT):
        """Build & send a binary command; return response data (after
        the 8-byte status header), or *None* if datain_len==0."""
        dout_len = len(dataout) if dataout else 0
        # -- build packet --
        pkt = bytearray(CMD_HEADER_LEN + dout_len)
        struct.pack_into('>H', pkt, 0, cmd_code)
        struct.pack_into('>H', pkt, CMD_LEN_FIELD_OFS, dout_len + datain_len)
        if dout_len:
            pkt[CMD_HEADER_LEN:] = dataout
            # checksum: make sum of data area == 0  (mod 256)
            pkt[-1] = make_checksum(pkt[CMD_HEADER_LEN:-1])
        # -- send --
        self._write(pkt, timeout=timeout)
        # -- receive --
        expect = RES_HEADER_LEN + datain_len
        resp = self._read_with_retry(expect, timeout)
        # -- check status --
        if len(resp) < 2:
            raise IOError("Empty response from scanner")
        status = struct.unpack_from('>H', resp, 0)[0]
        if status == STATUS_BUSY:
            raise BusyError()
        if status == STATUS_FAILED:
            raise IOError(f"Command 0x{cmd_code:04X} failed (status 0x{status:04X})")
        if status != STATUS_OK:
            raise IOError(f"Command 0x{cmd_code:04X} unexpected status 0x{status:04X}")
        if datain_len and len(resp) > RES_HEADER_LEN:
            return resp[RES_HEADER_LEN:]
        return None

    def _read_with_retry(self, size, timeout, retries=8):
        for attempt in range(retries):
            try:
                return self._read(size, timeout=timeout)
            except usb.core.USBError as e:
                is_timeout = 'timeout' in str(e).lower() or getattr(e, 'errno', 0) in (110, -116)
                if is_timeout and attempt < retries - 1:
                    time.sleep(1)
                else:
                    raise

    # ── protocol commands ──────────────────────────────────────────────
    def _start_session(self):
        """0xDB20 – open scan session."""
        self._exec(CMD_START_SESSION)

    def _abort_session(self):
        """0xEF20 – close scan session."""
        try:
            self._exec(CMD_ABORT_SESSION)
        except Exception:
            pass

    def _start_scan(self):
        """0xD920 – begin scanning (gen 3+)."""
        self._exec(CMD_SCAN_START_3)

    def _query_status(self):
        """0xDA20 – poll scanner status (gen 3+), returns 8 bytes."""
        data = self._exec(CMD_STATUS_3, datain_len=8)
        if data and len(data) >= 8:
            self.current_status[:8] = data[:8]
        return data

    def _is_calibrated(self):
        s0 = self.current_status[0]
        return (s0 & 0x01) == 1 or (s0 & 0x02) == 2

    def _send_gamma(self, gamma=1.0):
        """0xEE20 – upload one gamma LUT  (1024 × uint16-LE + 8-byte header)."""
        lut = build_gamma_table(gamma, 1024)
        data = bytearray(1024 * 2 + 8)          # 2056 bytes
        data[0] = 0x10
        data[2] = 0x08; data[3] = 0x04
        data[4:4 + len(lut)] = lut
        # last byte reserved for checksum (set by _exec)
        self._exec(CMD_GAMMA, dataout=data)

    def _send_scan_param(self, xdpi, ydpi, x, y, w, h,
                         channels=3, depth=8, calibrate=True):
        """0xD820 – set scan parameters (gen 3+ format, 0x38 bytes)."""
        d = bytearray(0x38)
        d[0x00] = 0x01        # flatbed
        d[0x01] = 0x01
        d[0x02] = 0x01
        d[0x05] = 0x01 if calibrate else 0x00
        struct.pack_into('>H', d, 0x08, xdpi | 0x8000)
        struct.pack_into('>H', d, 0x0A, ydpi | 0x8000)
        struct.pack_into('>I', d, 0x0C, x)
        struct.pack_into('>I', d, 0x10, y)
        struct.pack_into('>I', d, 0x14, w)
        struct.pack_into('>I', d, 0x18, h)
        d[0x1C] = 0x08 if channels == 3 else 0x04
        d[0x1D] = depth * channels
        d[0x1F] = 0x01
        d[0x20] = 0xFF
        d[0x21] = 0x81
        d[0x23] = 0x02
        d[0x24] = 0x01
        d[0x30] = 0x01
        self._exec(CMD_SCAN_PARAM_3, dataout=d)

    def _wait_ready(self, timeout_s=120):
        """Poll status until the scanner reports calibration complete."""
        print("  Waiting for calibration ...", end='', flush=True)
        self._query_status()
        t0 = time.time()
        while not self._is_calibrated():
            if time.time() - t0 > timeout_s:
                print(" TIMEOUT")
                raise TimeoutError("Scanner did not become ready")
            time.sleep(1)
            self._query_status()
        print(" ready.")

    def _read_image_block(self):
        """0xD420 – read one block of image data.
        Returns *(flags, raw_bytes)*.  flags & 0x28 == 0x28 → end of image."""
        cmd = bytearray(CMD_HEADER_LEN)
        struct.pack_into('>H', cmd, 0, CMD_READ_IMAGE)
        if (self.last_block & 0x20) == 0:
            req = (IMAGE_BLOCK_SIZE // 65536) * 65536 + 8
        else:
            req = 32 + 8
        struct.pack_into('>I', cmd, 12, req)

        self._write(cmd, timeout=BULK_TIMEOUT)

        # first read – up to 512 bytes (status header + start of data)
        first = self._read(512, timeout=IMG_TIMEOUT)
        hlen = 16
        if len(first) < hlen:
            self.last_block = 0x28
            return 0x28, b''

        status = struct.unpack_from('>H', first, 0)[0]
        flags  = first[8] & 0x38
        blk_sz = struct.unpack_from('>I', first, 12)[0]
        self.last_block = flags

        if status != STATUS_OK:
            raise IOError(f"read_image_block status 0x{status:04X}")
        if blk_sz == 0:
            return flags, b''

        # gather payload
        buf = bytearray(first[hlen:])
        if len(first) == 512 and len(buf) < blk_sz:
            remaining = blk_sz - len(buf)
            while remaining > 0:
                try:
                    chunk = self._read(min(remaining, IMAGE_BLOCK_SIZE),
                                       timeout=IMG_TIMEOUT)
                    buf.extend(chunk)
                    remaining -= len(chunk)
                    if len(chunk) == 0:
                        break
                except usb.core.USBError:
                    break

        return flags, bytes(buf[:blk_sz])

    # ── high-level scan ────────────────────────────────────────────────
    def scan(self, dpi=300, mode='color', output='scan_output.bmp'):
        """Perform a full flatbed scan and save the result as BMP.

        Parameters
        ----------
        dpi : int
            75, 150, 300, 600, 1200 or 2400
        mode : str
            ``'color'`` (24-bit RGB) or ``'gray'`` (8-bit grayscale)
        output : str
            Destination file path (.bmp)
        """
        if not self.connected:
            raise RuntimeError("Not connected")

        channels = 3 if mode == 'color' else 1
        depth = 8

        # Scale factor for DPI below the scanner minimum
        scale = max(1, LIDE300_MIN_XDPI // dpi) if dpi < LIDE300_MIN_XDPI else 1
        hw_dpi = dpi * scale                # actual DPI the scanner will use

        # Output image dimensions (at requested DPI)
        out_w = LIDE300_WIDTH_75DPI * dpi // 75
        out_h = LIDE300_HEIGHT_75DPI * dpi // 75

        # Scanner-side dimensions (at hw_dpi)
        xs = 0
        wx = align_up(out_w * scale + xs, 32)
        scan_h = min(out_h * scale, LIDE300_HEIGHT_75DPI * hw_dpi // 75)

        raw_line = wx * channels * (depth // 8)
        out_line = out_w * channels * (depth // 8)

        print(f"\n{'─'*60}")
        print(f" DPI requested : {dpi}   (hardware DPI: {hw_dpi}, scale: {scale})")
        print(f" Mode          : {mode}  ({channels}ch × {depth}bit)")
        print(f" Output size   : {out_w} × {out_h} px")
        print(f" Scanner width : {wx} px (raw, 32-aligned)")
        print(f" Raw line      : {raw_line} B   Output line: {out_line} B")
        print(f" Output file   : {output}")
        print(f"{'─'*60}\n")

        try:
            # Phase 1 – XML handshake (retry if scanner is still initializing)
            print("[1/10] XML StartJob ...")
            _RETRY_DETAILS = {'poweroninitializing', 'devicebusy', 'busy'}
            for xml_attempt in range(30):
                ok, detail = self._xml_dialog(XML_START_1)
                if ok:
                    break
                if detail.lower() in _RETRY_DETAILS:
                    print(f"  Scanner not ready ({detail}), "
                          f"waiting 2s ... ({xml_attempt+1}/30)")
                    time.sleep(2)
                else:
                    raise IOError(f"XML StartJob rejected: {detail}")
            else:
                raise IOError("Scanner did not become ready after 60s")

            print("[2/10] XML VendorCmd ModeShift ...")
            ok, detail = self._xml_dialog(XML_START_2)
            if not ok:
                raise IOError(f"XML ModeShift rejected: {detail}")

            # Phase 2 – clear pending interrupts
            print("[3/10] Clearing interrupts ...")
            self._clear_interrupts()

            # Phase 3 – open session (retry on busy)
            print("[4/10] Starting session ...")
            for attempt in range(10):
                try:
                    self._start_session()
                    break
                except BusyError:
                    if attempt < 9:
                        print(f"  busy – retry {attempt+2}/10")
                        time.sleep(1)
                    else:
                        raise IOError("Scanner stayed busy")

            # Phase 4 – gamma tables (×3 for gen 3+)
            print("[5/10] Sending gamma tables ×3 ...")
            for _ in range(3):
                self._send_gamma(1.0)

            # Phase 5 – scan parameters
            print("[6/10] Sending scan parameters ...")
            self._send_scan_param(
                xdpi=hw_dpi, ydpi=hw_dpi,
                x=0, y=0, w=wx, h=scan_h,
                channels=channels, depth=depth, calibrate=True)

            # Phase 6 – trigger scan
            print("[7/10] Starting scan ...")
            self._start_scan()

            # Phase 7 – wait for calibration
            print("[8/10] Calibrating ...")
            self._wait_ready()
            time.sleep(1)

            # Phase 8 – read image blocks
            print("[9/10] Reading image data ...")
            self.last_block = 0
            raw = bytearray()
            blk_n = 0
            t0 = time.time()
            while (self.last_block & 0x28) != 0x28:
                flags, data = self._read_image_block()
                blk_n += 1
                if data:
                    raw.extend(data)
                else:
                    time.sleep(0.01)
                if blk_n % 20 == 0:
                    elapsed = time.time() - t0
                    mb = len(raw) / 1048576
                    print(f"  block {blk_n}  {mb:.1f} MB  "
                          f"({mb/elapsed:.1f} MB/s)  flags=0x{flags:02X}")
            elapsed = time.time() - t0
            print(f"  Done: {len(raw)} bytes in {blk_n} blocks "
                  f"({elapsed:.1f}s)")

            # Phase 9 – clean up session
            print("[10/10] Ending session ...")
            self._abort_session()
            self._xml_dialog(XML_END)

            # ── post-process & save ────────────────────────────────────
            if not raw:
                print("WARNING: no image data received!")
                return False

            total_raw_lines = len(raw) // raw_line
            actual_h = min(total_raw_lines // scale, out_h)
            print(f"  Raw lines: {total_raw_lines}  →  output: {out_w}×{actual_h}")

            img = bytearray()
            if scale <= 1:
                for y in range(actual_h):
                    src = y * raw_line
                    img.extend(raw[src:src + out_line])
            else:
                # down-sample by averaging scale×scale blocks
                bpc = channels  # bytes per channel-set per pixel
                for y in range(actual_h):
                    row = bytearray(out_line)
                    for px in range(out_w):
                        for ch in range(channels):
                            total = 0
                            cnt = 0
                            for sy in range(scale):
                                for sx in range(scale):
                                    ry = y * scale + sy
                                    rx = px * scale + sx
                                    if ry < total_raw_lines and rx < wx:
                                        off = ry * raw_line + rx * bpc + ch
                                        if off < len(raw):
                                            total += raw[off]
                                            cnt += 1
                            row[px * bpc + ch] = total // max(cnt, 1)
                    img.extend(row)

            write_bmp(output, out_w, actual_h, img, channels)
            print(f"\nScan complete → {output}\n")
            return True

        except Exception as e:
            print(f"\nScan error: {e}")
            import traceback; traceback.print_exc()
            try: self._abort_session()
            except Exception: pass
            try: self._xml_dialog(XML_END)
            except Exception: pass
            return False


class BusyError(Exception):
    """Raised when the scanner returns STATUS_BUSY (0x1414)."""


# ═══════════════════════════════════════════════════════════════════════════
#  CLI entry point
# ═══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(
        description='Canon LiDE 300 scanner control  '
                    '(pixma protocol, XML dialogs, Python + libusb)')
    ap.add_argument('--dpi', type=int, default=300,
                    choices=[75, 150, 300, 600, 1200, 2400],
                    help='scan resolution (default: 300)')
    ap.add_argument('--mode', choices=['color', 'gray'], default='color',
                    help='color or grayscale (default: color)')
    ap.add_argument('-o', '--output', default='scan_output.bmp',
                    help='output BMP file (default: scan_output.bmp)')
    args = ap.parse_args()

    scanner = CanonLiDE300()
    try:
        if not scanner.connect():
            sys.exit(1)
        ok = scanner.scan(dpi=args.dpi, mode=args.mode, output=args.output)
        sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        try: scanner._abort_session()
        except Exception: pass
        try: scanner._xml_dialog(XML_END)
        except Exception: pass
        sys.exit(130)
    finally:
        scanner.disconnect()


if __name__ == '__main__':
    main()