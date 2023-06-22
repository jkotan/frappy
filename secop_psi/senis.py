#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# *****************************************************************************
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Module authors:
#   Markus Zolliker <markus.zolliker@psi.ch>
# *****************************************************************************
"""senis hall sensor"""


import threading
import time

import numpy as np
from serial import Serial

from secop.core import Attached, BoolType, FloatRange, IntRange, \
    Parameter, Property, Readable, StringType, TupleOf


class Temperature(Readable):
    pollerClass = None

    value = Parameter(datatype=FloatRange(unit='degC'))


class Bcomp(Readable):
    pollerClass = None

    value = Parameter(datatype=FloatRange(unit='T'))
    range = Parameter('working range', FloatRange(unit='T'), default=0)


class Raw(Readable):
    pollerClass = None

    value = Parameter(datatype=FloatRange())


class TeslameterBase(Readable):
    """code for both models

    the protocol is somewhat weird as the read command 'B' initiates a permanent update
    which has to be stopped in between the value polls and for other communication.

    the B components (and temperatures for 3MH6) are implemented as separate modules
    """
    x = Attached()
    y = Attached()
    z = Attached()

    value = Parameter('B vector', poll=True,
                     datatype=TupleOf(FloatRange(unit='T'), FloatRange(unit='T'), FloatRange(unit='T')))
    usb = Parameter('usb device', StringType(), readonly=False)
    enabled = Parameter('enable data acq', datatype=BoolType(), readonly=False, default=True)
    nsample = Parameter('number of samples for average', datatype=IntRange(1, 1000), readonly=False, default=1)

    def init_serial(self, baud):
        self._conn = Serial(self.usb, baud, timeout=0.1)
        self._lock = threading.Lock()
        self.stop_reading()

    def write_bytes(self, msg):
        with self._lock:
            self._conn.write(msg)

    def read_bytes(self, cnt):
        with self._lock:
            return self._conn.read(cnt)

    def stop_reading(self):
        self.write_bytes(b'S')
        self.read_bytes(9999)  # swallow bytes until timeout

    def write_enabled(self, value):
        if value:
            self.status = self.Status.IDLE, ''
        else:
            self.status = self.Status.DISABLED, 'disabled'
        self._x.status = self._y.status = self._z.status = self.status
        return value


class Teslameter3MH3(TeslameterBase):
    """simpler model without temperature and auto range

    remark: no query for the sample rate is possible, therefore set always to
    a default rate (therefore initwrite=True on the rate parameter)
    """
    range = Property('full scale', datatype=FloatRange(), default=2)

    def earlyInit(self):
        self.init_serial(115200)
        self.write_bytes(b'C')  # put into calibrated mode
        if self.read_bytes(1) != b'!':
            self.log.error('missing response to C command')
        self.write_bytes(b'A\x80\r')  # set to 90 SPS
        self.read_bytes(1)  # wait 0.1 sec as we get no reply

    def read_value(self):
        if not self.enabled:
            return self.value
        s = self._conn
        s.timeout = 0.1 + 0.02 * self.nsample
        for _ in range(2):
            self.write_bytes(b'B')
            # t = time.time()
            reply = self.read_bytes(8 * self.nsample)
            s.timeout = 0.1
            self.stop_reading()
            remainder = len(reply) % 8
            if remainder:
                reply = reply[:-remainder]
            if not reply:
                continue
            data = np.frombuffer(reply, dtype='i1,3<i2,i1')
            # first byte must be 'B' and last byte must be CR
            if np.all(data['f0'] == ord(b'B')) and np.all(data['f2'] == 13):
                break
        else:
            self.status = self.Status.ERROR, 'bad reply'
            raise ValueError('bad reply')
        self.status = self.Status.IDLE, ''
        value = np.average(data['f1'], axis=0) * self.range / 20000.
        self._x.value, self._y.value, self._z.value = value
        self._x.range = self._y.range =self._z.range = self.range
        return value


class Teslameter3MH6(TeslameterBase):
    """luxury model with probe and box temperature and autorange"""
    x_direct = Attached()
    y_direct = Attached()
    z_direct = Attached()
    probe_temp = Attached()
    box_temp = Attached()
    probe_temp_direct = Attached()
    box_temp_direct = Attached()

    range = Parameter('range or 0 for autorange', FloatRange(0, 20, unit='T'), readonly=False, default=0)
    rate = Parameter('sampling rate', datatype=FloatRange(10, 15000, unit='Hz'),
                     readonly=False, poll=True)
    avtime = Parameter('data acquisition time', FloatRange(), default=0)

    SAMPLING_RATES = {0xe0: 15000, 0xd0: 7500, 0xc0: 3750, 0xb0: 2000, 0xa1: 1000,
                      0x92: 500, 0x82: 100, 0x72: 60, 0x63: 50, 0x53: 30, 0x23: 10}
    RANGES = dict(zip(b'1234', [0.1, 0.5, 2, 20]))

    def earlyInit(self):
        self.init_serial(3000000)
        self.write_rate(10)

    def get_data(self):
        for _ in range(2):
            self.write_bytes(b'B')
            reply = self.read_bytes(25 * self.nsample)
            self.stop_reading()
            remainder = len(reply) % 25
            if remainder:
                reply = reply[:-remainder]
            if not reply:
                continue
            chk = np.frombuffer(reply, dtype='i1,23i1,i1')
            if not np.all(np.sum(chk['f1'], axis=1) % 256 == 0):
                status = 'checksum error'
                continue
            # first byte must be 'B' and last byte must be CR
            if np.all(chk['f0'] == ord(b'B')) and np.all(chk['f2'] == 13):
                break
            status = 'bad reply'
        else:
            self.status = self.Status.ERROR, status
            raise ValueError(status)
        self.status = self.Status.IDLE, ''
        return reply

    def read_value(self):
        if not self.enabled:
            return self.value
        t0 = time.time()
        s = self._conn
        s.timeout = 0.1 + self.nsample / self.rate
        self.write_bytes(b'C')  # put into calibrated mode
        if self.read_bytes(1) != b'c':
            self.log.error('missing response to C command')
        reply = self.get_data()
        data = np.frombuffer(reply,
            dtype=[('_head', 'i1'),
                 ('x', '>f4'), ('thc', '>f4'), ('y', '>f4'), ('z', '>f4'),
                 ('_ted', '>i2'), ('tec', '>f4'), ('_tail', 'i2')])

        mean = {}
        for key in data.dtype.fields:
            if not key.startswith('_'):
                mean[key] = np.average(data[key])

        self._x.value = mean['x'] * 0.001
        self._y.value = mean['y'] * 0.001
        self._z.value = mean['z'] * 0.001
        self._probe_temp.value = mean['thc']
        self._box_temp.value = mean['tec']

        self.write_bytes(b'D')  # put into NONcalibrated mode
        if self.read_bytes(1) != b'd':
            self.log.error('missing response to D command')
        reply = self.get_data()
        data = np.frombuffer(reply,
            dtype=[('_head', 'i1'),
                 ('x', '>i4'), ('thc', '>i4'), ('y', '>i4'), ('z', '>i4'),
                 ('_ted', '>i2'), ('tec', '>i4'), ('_tail', 'i2')])
        for key in data.dtype.fields:
            if not key.startswith('_'):
                mean[key] = np.average(data[key])

        self._x_direct.value = mean['x']
        self._y_direct.value = mean['y']
        self._z_direct.value = mean['z']
        self._probe_temp_direct.value = mean['thc']
        self._box_temp_direct.value = mean['tec'] * 0.01

        self.avtime = time.time() - t0
        return self._x.value, self._y.value, self._z.value

    def get_rate_code(self, value):
        for rate_code, sr in sorted(self.SAMPLING_RATES.items(), key=lambda kv: kv[1]):
            if value < sr * 1.1:
                break
        return sr, rate_code

    def write_rate(self, value):
        sr, code = self.get_rate_code(value)
        for _ in range(2):
            self.write_bytes(b'K%2.2x' % code)
            if self.read_bytes(2) == b'k%c' % code:
                break
            self.stop_reading()
        else:
            raise ValueError('bad response from rate command')
        return sr

    def read_rate(self):
        self.write_bytes(b'K?')
        reply = self.read_bytes(2)
        if reply[0:1] != b'k':
            raise ValueError('bad response from rate query')
        return self.SAMPLING_RATES[reply[1]]

    def read_range(self):
        self.write_bytes(b'amr?')
        reply = self.read_bytes(5)
        if reply == b'arng:':
            ranges = [self.RANGES[c] for c in self.read_bytes(3)]
            result = 0
        elif reply == b'mrng:':
            ranges = [self.RANGES[self.read_bytes(1)[0]]] * 3
            result = ranges[0]
        else:
            raise ValueError('bad reply to range query %s' % repr(reply))
        self._x.range, self._y.range, self._z.range = ranges
        return result

    def write_range(self, value):
        status = None
        for _ in range(2):
            if status:
                self.stop_reading()
            try:
                rng = self.read_range()
            except ValueError:
                status = 'can not read range'
                continue
            if value == rng:
                return value
            if value == 0:
                self.write_bytes(b'T')
                if self.read_bytes(3) != b'T-1':
                    status = 'bad reply to auto range command'
                    continue
                return 0
            if rng == 0:
                self.write_bytes(b'T')
                if self.read_bytes(3) != b'T-0':
                    status = 'bad reply to toggle manual range command'
                    continue
            for code, rng in sorted(self.RANGES.items()):
                if value < rng * 1.1:
                    break
            self.write_bytes(b'mr%c' % code)
            reply = self.read_bytes(6)
            if reply != b'mrng:%c' % code:
                status = 'bad reply to manual range command %s' % repr(reply)
                continue
            return rng
        raise ValueError(status)
