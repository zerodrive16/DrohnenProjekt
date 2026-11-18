"""Minimaler Treiber für das YDLIDAR T-mini Plus (UART, 230400 Baud).

Kein YDLidar-SDK und kein numpy nötig, nur pyserial (läuft leicht auf dem Pi Zero).
Liefert komplette Umdrehungen als (zeit, winkel_grad, distanz_mm, intensität). Winkel: 0..360, im Uhrzeigersinn
(von oben gesehen), 0 Grad = Vorderseite des LiDARs laut Gehäusemarkierung.

Paketformat (YDLIDAR-Protokoll):
  PH(2)=AA 55 | CT(1) | LSN(1) | FSA(2) | LSA(2) | CS(2) | LSN Samples
  Samples mit Intensität: 3 Byte (I, D_lo, D_hi), sonst 2 Byte.
Die Sample-Größe wird automatisch über die Prüfsumme erkannt.
"""
import struct
import time

import serial

CMD_START = b"\xA5\x60"
CMD_STOP = b"\xA5\x65"
PKT_HDR = struct.Struct("<HBBHHH")  # PH, CT, LSN, FSA, LSA, CS


class TMiniPlus:
    def __init__(self, port="/dev/ttyUSB0", baud=230400):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()
        self.sample_size = None      # 3 oder 2, wird automatisch erkannt
        self.check_cs = True
        self._tried = 0
        self.cs_errors = 0
        self.packets = 0
        self._angles, self._dists, self._intens = [], [], []

    def start(self):
        try:
            self.ser.dtr = True      # manche Adapterboards schalten darüber den Motor
        except OSError:
            pass
        self.ser.write(CMD_STOP)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.write(CMD_START)

    def stop(self):
        try:
            self.ser.write(CMD_STOP)
            time.sleep(0.05)
        finally:
            self.ser.close()

    # ------------------------------------------------------------------
    @staticmethod
    def _checksum(ph, ct, lsn, fsa, lsa, data, ssize):
        cs = ph ^ (ct | (lsn << 8)) ^ fsa ^ lsa
        if ssize == 3:
            for i in range(0, lsn * 3, 3):
                cs ^= data[i]
                cs ^= data[i + 1] | (data[i + 2] << 8)
        else:
            for i in range(0, lsn * 2, 2):
                cs ^= data[i] | (data[i + 1] << 8)
        return cs

    def _detect(self, ph, ct, lsn, fsa, lsa, cs, start):
        """Sample-Größe über Prüfsumme bestimmen. True = erkannt."""
        for ssize in (3, 2):
            end = start + PKT_HDR.size + lsn * ssize
            if end > len(self.buf):
                return None              # noch nicht genug Daten
            data = self.buf[start + PKT_HDR.size:end]
            if self._checksum(ph, ct, lsn, fsa, lsa, data, ssize) == cs:
                self.sample_size = ssize
                print(f"[lidar] Sample-Größe erkannt: {ssize} Byte")
                return True
        self._tried += 1
        if self._tried > 100:
            print("[lidar] WARNUNG: Prüfsumme passt nie - nehme 3 Byte ohne Prüfung")
            self.sample_size, self.check_cs = 3, False
            return True
        return False

    def read_scans(self):
        """Generator: liefert (zeitstempel, winkel, distanzen, intensitäten) pro Umdrehung."""
        while True:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf += chunk
            yield from self._parse()

    def _parse(self):
        while True:
            start = self.buf.find(b"\xAA\x55")
            if start < 0:
                del self.buf[:-1]
                return
            if start > 0:
                del self.buf[:start]
                start = 0
            if len(self.buf) < PKT_HDR.size:
                return
            ph, ct, lsn, fsa, lsa, cs = PKT_HDR.unpack_from(self.buf)
            if lsn == 0 or not (fsa & 1) or not (lsa & 1):
                del self.buf[:2]          # ungültiger Header
                continue

            if self.sample_size is None:
                res = self._detect(ph, ct, lsn, fsa, lsa, cs, 0)
                if res is None:
                    return
                if res is False:
                    del self.buf[:2]
                    continue

            ss = self.sample_size
            total = PKT_HDR.size + lsn * ss
            if len(self.buf) < total:
                return
            data = bytes(self.buf[PKT_HDR.size:total])

            if self.check_cs and self._checksum(ph, ct, lsn, fsa, lsa, data, ss) != cs:
                self.cs_errors += 1
                del self.buf[:2]
                continue
            del self.buf[:total]
            self.packets += 1

            # CT Bit0 = 1: Start einer neuen Umdrehung
            if ct & 0x01:
                if len(self._angles) > 30:
                    yield time.time(), self._angles, self._dists, self._intens
                self._angles, self._dists, self._intens = [], [], []

            a0 = (fsa >> 1) / 64.0
            a1 = (lsa >> 1) / 64.0
            diff = a1 - a0
            if diff < 0:
                diff += 360.0
            for i in range(lsn):
                if ss == 3:
                    d = ((data[3 * i + 2] << 8) | data[3 * i + 1]) >> 2
                    inten = data[3 * i]
                else:
                    d = ((data[2 * i + 1] << 8) | data[2 * i]) >> 2
                    inten = 0
                if d == 0:
                    continue                 # kein Echo
                a = a0 + (diff * i / (lsn - 1) if lsn > 1 else 0.0)
                self._angles.append(a % 360.0)
                self._dists.append(d)
                self._intens.append(inten)
