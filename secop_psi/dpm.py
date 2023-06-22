#  -*- coding: utf-8 -*-
# *****************************************************************************
#
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
#   M. Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************
"""transducer DPM3 read out"""

import time

from secop.core import Readable, Parameter, FloatRange, StringIO,\
     HasIodev, IntRange, Done


class DPM3IO(StringIO):
    end_of_line = '\r'
    timeout = 3
    identification = [('*1R135', '01')]


def hex2float(hexvalue, digits):
    value = int(hexvalue, 16)
    if value >= 0x800000:
        value -= 0x1000000
    return value / (10 ** digits)


def float2hex(value, digits):
    intvalue = int(round(value * 10 ** digits,0))
    if intvalue < 0:
        intvalue += 0x1000000
    return '%06X' % intvalue


class DPM3(HasIodev, Readable):
    OFFSET = 0x8f
    SCALE = 0x8c

    MAGNITUDE = {'1':  1, '2':  10, '3':  100, '4':  1e3, '5':  1e4, '6':  1e5,
                 '9': -1, 'A': -10, 'B': -100, 'C': -1e3, 'D': -1e4, 'E': -1e5}

    iodevClass = DPM3IO

    value = Parameter(datatype=FloatRange(unit='N'))
    digits = Parameter('number of digits for value', IntRange(0, 5), initwrite=True, readonly=False)
    # Note: we have to treat the units properly.
    # We got an output of 150 for 10N. The maximal force we want to deal with is 100N,
    #   thus a maximal output of 1500. 10=150/f
    offset = Parameter('', FloatRange(-1e5, 1e5), readonly=False, poll=True)
    scale_factor = Parameter('', FloatRange(-1e5, 1e5, unit='input_units/N'), readonly=False, poll=True)

    def query(self, adr, value=None):
        if value is not None:
            if adr == self.SCALE:
                absval = abs(value)
                for nibble, mag in self.MAGNITUDE.items():
                    if 10000 <= round(value * mag, 0) <= 99999:
                        break
                else:
                    # no suitable range found
                    if absval >= 99999.5:  # overrange
                        raise ValueError('%s is out of range' % value)
                    # underrange: take lowest
                    nibble = '9' if value < 0 else '1'
                    mag = self.MAGNITUDE[nibble]
                hex_val = nibble + '%05X' % int(round(value * mag, 0))
                if hex_val[1:] == '00000':
                    raise ValueError('scale factor can not be 0', value)
            else:
                hex_val = float2hex(value, self.digits)
            cmd = '*1F3%02X%s\r' % (adr, hex_val)
        else:
            cmd = ""
        cmd = cmd + '*1G3%02X' % adr
        hexvalue = self._iodev.communicate(cmd)
        if adr == self.SCALE:
            mag = self.MAGNITUDE[hexvalue[0:1]]
            value = int(hexvalue[1:], 16)
            return value/mag
        else:
            return hex2float(hexvalue, self.digits)

    def write_digits(self, value):
        # value defines the number of digits
        back_value = self._iodev.communicate('*1F135%02X\r*1G135' % (value + 1))
        self.digits = int(back_value, 16) - 1
        # recalculate proper scale and offset
        self.write_scale_factor(self.scale_factor)
        self.write_offset(self.offset)
        return Done

    def read_digits(self):
        back_value = self._iodev.communicate('*1G135')
        return int(back_value,16) - 1

    def read_value(self):
        return float(self._iodev.communicate('*1B1'))

    def read_offset(self):
        reply = self.query(self.OFFSET)
        return reply

    def write_offset(self, value):
        return self.query(self.OFFSET, value)

    def read_scale_factor(self):
        reply = self.query(self.SCALE)
        return float(reply) / 10 ** self.digits

    def write_scale_factor(self, value):
        reply = self.query(self.SCALE, value * 10 ** self.digits)
        return float(reply) / 10 ** self.digits
