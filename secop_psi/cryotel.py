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

"""driver for cryotel stirling cryocooler"""

from secop.core import Command, EnumType, FloatRange, HasIodev, Parameter, Drivable, StringIO, StringType
from secop.errors import CommunicationFailedError, HardwareError


class CryotelIO(StringIO):
    end_of_line = ('\n', '\r')
    # eol_write will be given explicitly

    @Command(StringType(), result=StringType())
    def communicate(self, command):
        """send a command and receive a reply

        we receive an echo line first.
        in case the command does not contain '=', the effective reply is in a second line
        """
        with self._lock:
            echo = super().communicate(command)
            if echo.strip() != command.strip():
                raise CommunicationFailedError('missing echo')
            if '=' in command:
                reply = echo
            else:
                reply = self._conn.readline().decode()
        return reply


class Cryo(HasIodev, Drivable):
    value = Parameter('current temperature', FloatRange(unit='deg'))
    target = Parameter('target temperature', FloatRange(unit='deg'), readonly=False)
    mode = Parameter('control mode', EnumType('mode', off=0, power=1, temperature=2), readonly=False, poll=True)
    power = Parameter('power', FloatRange(unit='W'), poll=True)
    setpower = Parameter('requested power', FloatRange(unit='W'), default=0)
    cool_power = Parameter('power for cooling', FloatRange(unit='W'), default=240, readonly=False)
    hold_power = Parameter('power for holding T', FloatRange(unit='W'), default=120, readonly=False)
    cool_threshold = Parameter('switch to cool_power once above this value', FloatRange(unit='K'), default=100, readonly=False)
    hold_threshold = Parameter('switch to hold_power once below this value', FloatRange(unit='K'), default=95, readonly=False)

    iodevClass = CryotelIO
    cnt_inside = 0

    def get(self, cmd):
        return float(self._iodev.communicate(cmd))

    def set(self, cmd, value, check=False):
        setcmd = '%s=%.2f' % (cmd, value)
        self._iodev.communicate(setcmd)
        reply = float(self._iodev.communicate(cmd))
        if check:
            if value != reply:
                raise HardwareError('illegal reply from %s: %g' % (cmd, reply))
        return reply

    def read_value(self):
        temp = self.get('TC')
        status = self.status
        if self.mode == 1:  # P reg
            setpower = self.setpower
            if temp < self.hold_threshold:
                setpower = self.hold_power
                status = self.Status.IDLE, 'holding'
            elif temp > self.cool_threshold:
                setpower = self.cool_power
                status = self.Status.BUSY, 'cooling'
            if abs(setpower - self.setpower) > 0.009:
                self.setpower = self.set('SET PWOUT', setpower)
        elif self.mode == 2:  # T reg
            if self.status[0] == 'BUSY':
                if abs(temp - self.target) < 1:
                    self.cnt_inside += 1
                    if self.cnt_inside >= 10:
                        status = self.Status.IDLE, ''
                    else:
                        status = self.Status.BUSY, 'settling'
                else:
                    status = self.Status.BUSY, 'outside tolerance'
        else:
            status = self.Status.IDLE, 'off'
        if status != self.status:
            self.status = status
        return temp

    def read_target(self):
        return self.get('SET TTARGET')

    def write_target(self, value):
        if abs(value - self.target) > 0.009:
            self.status = self.Status.BUSY, 'changed target'
            self.cnt_inside = 0
        return self.set('SET TTARGET', value)

    def read_power(self):
        return self.get('P')

    def read_setpower(self):
        return self.get('SET PWOUT')

    def read_mode(self):
        is_stopped = self.get('SET SSTOP')
        if is_stopped:
            return 0  # off
        pidmode = self.get('SET PID')
        if pidmode == 2:
            return 2  # T reg
        return 1  # P reg

    def write_mode(self, value):
        if value == 0:
            self.set('SET SSTOP', 1, check=True)
            self.status = self.Status.IDLE, 'off'
            return value
        is_stopped = self.get('SET SSTOP')
        if is_stopped:
            self.set('SET SSTOP', 0, check=True)
        if value == 1:
            self.set('SET PID', 0, check=True)
            self.read_value()
        else:
            self.set('SET PID', 2, check=True)
            self.write_target(self.target)
        return value
