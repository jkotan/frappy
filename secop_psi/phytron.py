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
#   Markus Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************

"""driver for pythron motors"""

from secop.core import Done, Command, EnumType, FloatRange, IntRange, \
    HasIodev, Parameter, Property, Drivable, PersistentMixin, PersistentParam, StringIO, StringType
from secop.errors import CommunicationFailedError, HardwareError


class PhytronIO(StringIO):
    end_of_line = '\x03'  # ETX
    identification = [('0IVR', 'MCC Minilog .*')]

    def communicate(self, command):
        head, _, reply = super().communicate('\x02' + command).partition('\x02')
        if reply[0] != '\x06':  # ACK
            raise CommunicationFailedError('missing ACK %r' % reply)
        return reply[1:]


class Motor(PersistentMixin, HasIodev, Drivable):
    axis = Property('motor axis X or Y', StringType(), default='X')
    address = Property('address', IntRange(0, 15), default=0)
    speed_factor = Property('steps / degree', FloatRange(0, None), default=2000)

    encoder_mode = Parameter('how to treat the encoder', EnumType('encoder', NO=0, READ=1, CHECK=2),
                             default=1, readonly=False)
    value = Parameter('angle', FloatRange(unit='deg'), poll=True)
    target = Parameter('target angle', FloatRange(unit='deg'), readonly=False)
    speed = Parameter('', FloatRange(0, 20, unit='deg/s'), readonly=False, poll=True)
    accel = Parameter('', FloatRange(2, 250, unit='deg/s/s'), readonly=False, poll=True)
    encoder_tolerance = Parameter('', FloatRange(unit='deg'), readonly=False, default=0.01)
    zero = PersistentParam('', FloatRange(unit='deg'), readonly=False, default=0)
    encoder = Parameter('encoder reading', FloatRange(unit='deg'), poll=True)
    sameside_offset = Parameter('offset when always approaching from the same side',
                                FloatRange(unit='deg'), readonly=True, default=0)

    iodevClass = PhytronIO
    fast_pollfactor = 0.02
    _sameside_pending = False

    def earlyInit(self):
        self.loadParameters()

    def get(self, cmd):
        return self._iodev.communicate('\x02%x%s%s' % (self.address, self.axis, cmd))

    def set(self, cmd, value):
        self._iodev.communicate('\x02%x%s%s%g' % (self.address, self.axis, cmd, value))

    def set_get(self, cmd, value, query):
        self.set(cmd, value)
        return self.get(query)

    def read_value(self):
        prev_enc = self.encoder
        pos = float(self.get('P20R')) + self.zero
        if self.encoder_mode != 'NO':
            enc = self.read_encoder()
        else:
            enc = pos
        status = self.get('=H')
        if status == 'N':
            if self.encoder_mode == 'CHECK':
                e1, e2 = sorted((prev_enc, enc))
                if e1 - self.encoder_tolerance <= pos <= e2 + self.encoder_tolerance:
                    self.status = self.Status.BUSY, 'driving'
                else:
                    self.log.error('encoder lag: %g not within %g..%g' % (pos, e1, e2))
                    self.get('S')  # stop
                    self.status = self.Status.ERROR, 'encoder lag error'
            else:
                self.status = self.Status.BUSY, 'driving'
        else:
            if self._sameside_pending:
                # drive to real target
                self.set('A', self.target - self.zero)
                self._sameside_pending = False
                return pos
            if (self.encoder_mode == 'CHECK' and
                    abs(enc - pos) > self.encoder_tolerance):
                self.status = self.Status.ERROR, 'encoder does not match pos'
            else:
                self.status = self.Status.IDLE, ''
        return pos

    def read_encoder(self):
        if self.encoder_mode == 'NO':
            return self.value
        return float(self.get('P22R')) + self.zero

    def read_speed(self):
        return float(self.get('P14R')) / self.speed_factor

    def write_speed(self, value):
        if abs(float(self.get('P03R')) * self.speed_factor - 1) > 0.001:
            raise HardwareError('speed factor does not match')
        return float(self.set_get('P14S', int(value * self.speed_factor), 'P14R')) / self.speed_factor

    def read_accel(self):
        return float(self.get('P15R')) / self.speed_factor

    def write_accel(self, value):
        if abs(float(self.get('P03R')) * self.speed_factor - 1) > 0.001:
            raise HardwareError('speed factor does not match')
        return float(self.set_get('P15S', int(value * self.speed_factor), 'P15R')) / self.speed_factor

    def write_target(self, value):
        if self.status[0] == self.Status.ERROR:
            raise HardwareError('need reset')
        self.status = self.Status.BUSY, 'changed target'
        if self.sameside_offset:
            # drive first to target + sameside_offset
            # we do not optimize when already driving from the right side
            self._sameside_pending = True
            self.set('A', value - self.zero + self.sameside_offset)
        else:
            self.set('A', value - self.zero)
        return value

    def write_zero(self, value):
        self.zero = value
        self.saveParameters()
        return Done

    def stop(self):
        self.get('S')

    @Command
    def reset(self):
        self.read_value()
        if self.status[0] == self.Status.ERROR:
            enc = self.encoder - self.zero
            pos = self.value - self.zero
            if abs(enc - pos) > self.encoder_tolerance:
                if enc < 0:
                    while enc < 0:
                        self.zero -= 360
                        enc += 360
                    self.set('P22S', enc)
                    self.saveParameters()
                self.set('P20S', enc)  # set pos to encoder
            self.read_value()
        # self.status = self.Status.IDLE, ''

# TODO:
#  '=E' electronics status
#  '=I+' / '=I-': limit switches
#  use P37 to determine if restarted