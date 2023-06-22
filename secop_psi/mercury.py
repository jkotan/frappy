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
"""oxford instruments mercury family"""


import math
import re
import time

from secop.core import Drivable, HasIodev, \
    Parameter, Property, Readable, StringIO
from secop.datatypes import EnumType, FloatRange, StringType
from secop.errors import HardwareError


class MercuryIO(StringIO):
    identification = [('*IDN?', r'IDN:OXFORD INSTRUMENTS:MERCURY*')]


VALUE_UNIT = re.compile(r'(.*\d)([A-Za-z]*)$')


def make_map(**kwds):
    """create a dict converting internal names to values and vice versa"""
    kwds.update({v: k for k, v in kwds.items()})
    return kwds


MODE_MAP = make_map(OFF=0, ON=1)
SAMPLE_RATE = make_map(OFF=1, ON=0)  # invert the codes used by OI


class MercuryChannel(HasIodev):
    slots = Property('''slot uids
    
                     example: DB6.T1,DB1.H1
                     slot ids for sensor (and control output)''',
                     StringType())
    channel_name = Parameter('mercury nick name', StringType())
    channel_type = ''  #: channel type(s) for sensor (and control)

    def query(self, adr, value=None):
        """get or set a parameter in mercury syntax

        :param adr: for example "TEMP:SIG:TEMP"
        :param value: if given and not None, a write command is executed
        :return:  the value

        remark: the DEV:<slot> is added automatically, when adr starts with the channel type
                in addition, when addr starts with '0:' or '1:', the channel type is added
        """
        for i, (channel_type, slot) in enumerate(zip(self.channel_type.split(','), self.slots.split(','))):
            if adr.startswith('%d:' % i):
                adr = 'DEV:%s:%s:%s' % (slot, channel_type, adr[2:])  # assume i <= 9
                break
            if adr.startswith(channel_type + ':'):
                adr = 'DEV:%s:%s' % (slot, adr)
                break
        if value is not None:
            try:
                value = '%g' % value  # this works for float, integers and enums
            except ValueError:
                value = str(value)  # this alone would not work for enums, and not be nice for floats
            cmd = 'SET:%s:%s' % (adr, value)
            reply = self._iodev.communicate(cmd)
            if reply != 'STAT:%s:VALID' % cmd:
                raise HardwareError('bad response %r to %r' % (reply, cmd))
            # chain a read command anyway
        cmd = 'READ:%s' % adr
        reply = self._iodev.communicate(cmd)
        head, _, result = reply.rpartition(':')
        if head != 'STAT:%s' % adr:
            raise HardwareError('bad response %r to %r' % (reply, cmd))
        match = VALUE_UNIT.match(result)
        if match:  # result can be interpreted as a float with optional units
            return float(match.group(1))
        return result

    def read_channel_name(self):
        return self.query('')


class TemperatureSensor(MercuryChannel, Readable):
    channel_type = 'TEMP'
    value = Parameter(unit='K')
    raw = Parameter('raw value', FloatRange())

    def read_value(self):
        return self.query('TEMP:SIG:TEMP')

    def read_raw(self):
        return self.query('TEMP:SIG:RES')


class HasProgressCheck:
    """mixin for progress checks

    Implements progress checks based on tolerance, settling time and timeout.
    The algorithm does its best to support changes of these parameters on the
    fly. However, the full history is not considered, which means for example
    that the spent time inside tolerance stored already is not altered when
    changing tolerance.
    """
    tolerance = Parameter('absolute tolerance', FloatRange(0), readonly=False, default=0)
    relative_tolerance = Parameter('_', FloatRange(0, 1), readonly=False, default=0)
    settling_time = Parameter(
        '''settling time

        total amount of time the value has to be within tolerance before switching to idle.
        ''', FloatRange(0), readonly=False, default=0)
    timeout = Parameter(
        '''timeout

        timeout = 0: disabled, else:
        A timeout happens, when the difference value - target is not improved by more than
        a factor 2 within timeout.

        More precisely, we expect a convergence curve which decreases the difference
        by a factor 2 within timeout/2.
        If this expected progress is delayed by more than timeout/2, a timeout happens.
        If the convergence is better than above, the expected curve is adjusted continuously.
        In case the tolerance is reached once, a timeout happens when the time after this is
        exceeded by more than settling_time + timeout.
        ''', FloatRange(0, unit='sec'), readonly=False, default=3600)
    status = Parameter('status determined from progress check')
    value = Parameter()
    target = Parameter()

    _settling_start = None  # supposed start of settling time (0 when outside)
    _first_inside = None  # first time within tolerance
    _spent_inside = 0  # accumulated settling time
    # the upper limit for t0, for the curve timeout_dif * 2 ** -(t - t0)/timeout not touching abs(value(t) - target)
    _timeout_base = 0
    _timeout_dif = 1

    def check_progress(self, value, target):
        """called from read_status

        intended to be also be used for alternative implementations of read_status
        """
        base = max(abs(target), abs(value))
        tol = base * self.relative_tolerance + self.tolerance
        if tol == 0:
            tol = max(0.01, base * 0.01)
        now = time.time()
        dif = abs(value - target)
        if self._settling_start:  # we were inside tol
            self._spent_inside = now - self._settling_start
            if dif > tol:  # transition inside -> outside
                self._settling_start = None
        else:  # we were outside tol
            if dif <= tol:  # transition outside -> inside
                if not self._first_inside:
                    self._first_inside = now
                self._settling_start = now - self._spent_inside
        if self._spent_inside > self.settling_time:
            return 'IDLE', ''
        result = 'BUSY', ('inside tolerance' if self._settling_start else 'outside tolerance')
        if self.timeout:
            if self._first_inside:
                if now > self._first_inside + self.settling_time + self.timeout:
                    return 'WARNING', 'settling timeout'
                return result
            tmo2 = self.timeout / 2

            def exponential_convergence(t):
                return self._timeout_dif * 2 ** -(t - self._timeout_base) / tmo2

            if dif < exponential_convergence(now):
                # convergence is better than estimated, update expected curve
                self._timeout_dif = dif
                self._timeout_base = now
            elif dif > exponential_convergence(now - tmo2):
                return 'WARNING', 'convergence timeout'
        return result

    def reset_progress(self, value, target):
        """must be called from write_target, whenever the target changes"""
        self._settling_start = None
        self._first_inside = None
        self._spent_inside = 0
        self._timeout_base = time.time()
        self._timeout_dif = abs(value - target)

    def read_status(self):
        if self.status[0] == 'IDLE':
            # do not change when idle already
            return self.status
        return self.check_progress(self.value, self.target)

    def write_target(self, value):
        raise NotImplementedError()


class Loop(HasProgressCheck, MercuryChannel):
    """common base class for loops"""
    mode = Parameter('control mode', EnumType(manual=0, pid=1), readonly=False)
    prop = Parameter('pid proportional band', FloatRange(), readonly=False)
    integ = Parameter('pid integral parameter', FloatRange(unit='min'), readonly=False)
    deriv = Parameter('pid differential parameter', FloatRange(unit='min'), readonly=False)
    """pid = Parameter('control parameters', StructOf(p=FloatRange(), i=FloatRange(), d=FloatRange()),readonly=False)"""
    pid_table_mode = Parameter('', EnumType(off=0, on=1), readonly=False)

    def read_prop(self):
        return self.query('0:LOOP:P')

    def read_integ(self):
        return self.query('0:LOOP:I')

    def read_deriv(self):
        return self.query('0:LOOP:D')

    def write_prop(self, value):
        return self.query('0:LOOP:P', value)

    def write_integ(self, value):
        return self.query('0:LOOP:I', value)

    def write_deriv(self, value):
        return self.query('0:LOOP:D', value)

    def read_enable_pid_table(self):
        return self.query('0:LOOP:PIDT').lower()

    def write_enable_pid_table(self, value):
        return self.query('0:LOOP:PIDT', value.upper()).lower()

    def read_mode(self):
        return MODE_MAP[self.query('0:LOOP:ENAB')]

    def write_mode(self, value):
        if value == 'manual':
            self.status = 'IDLE', 'manual mode'
        elif self.status[0] == 'IDLE':
            self.status = 'IDLE', ''
        return MODE_MAP[self.query('0:LOOP:ENAB', value)]

    def write_target(self, value):
        raise NotImplementedError

    # def read_pid(self):
    #     # read all in one go, in order to reduce comm. traffic
    #     cmd = 'READ:DEV:%s:TEMP:LOOP:P:I:D' % self.slots.split(',')[0]
    #     reply = self._iodev.communicate(cmd)
    #     result = list(reply.split(':'))
    #     pid = result[6::2]
    #     del result[6::2]
    #     if ':'.join(result) != cmd:
    #         raise HardwareError('bad response %r to %r' % (reply, cmd))
    #     return dict(zip('pid', pid))
    #
    # def write_pid(self, value):
    #     # for simplicity use single writes
    #     return {k: self.query('LOOP:%s' % k.upper(), value[k]) for k in 'pid'}


class TemperatureLoop(Loop, TemperatureSensor, Drivable):
    channel_type = 'TEMP,HTR'
    heater_limit = Parameter('heater output limit', FloatRange(0, 100, unit='W'), readonly=False)
    heater_resistivity = Parameter('heater resistivity', FloatRange(10, 1000, unit='Ohm'), readonly=False)
    ramp = Parameter('ramp rate', FloatRange(0, unit='K/min'), readonly=False)
    enable_ramp = Parameter('enable ramp rate', EnumType(off=0, on=1), readonly=False)
    auto_flow = Parameter('enable auto flow', EnumType(off=0, on=1), readonly=False)
    heater_output = Parameter('heater output', FloatRange(0, 100, unit='W'), readonly=False)

    def read_heater_limit(self):
        return self.query('HTR:VLIM') ** 2 / self.heater_resistivity

    def write_heater_limit(self, value):
        result = self.query('HTR:VLIM', math.sqrt(value * self.heater_resistivity))
        return result ** 2 / self.heater_resistivity

    def read_heater_resistivity(self):
        value = self.query('HTR:RES')
        if value:
            return value
        return self.heater_resistivity

    def write_heater_resistivity(self, value):
        return self.query('HTR:RES', value)

    def read_enable_ramp(self):
        return self.query('TEMP:LOOP:RENA').lower()

    def write_enable_ramp(self, value):
        return self.query('TEMP:LOOP:RENA', EnumType(off=0, on=1)(value).name).lower()

    def read_auto_flow(self):
        return self.query('TEMP:LOOP:FAUT').lower()

    def write_auto_flow(self, value):
        return self.query('TEMP:LOOP:FAUT', EnumType(off=0, on=1)(value).name).lower()

    def read_ramp(self):
        return self.query('TEMP:LOOP:RSET')

    def write_ramp(self, value):
        if not value:
            self.write_enable_ramp(0)
            return 0
        if value:
            self.write_enable_ramp(1)
        return self.query('TEMP:LOOP:RSET', value)

    def read_target(self):
        # TODO: check about working setpoint
        return self.query('TEMP:LOOP:TSET')

    def write_target(self, value):
        if self.mode != 'pid':
            self.log.warning('switch to pid loop mode')
            self.write_mode('pid')
        self.reset_progress(self.value, value)
        return self.query('TEMP:LOOP:TSET', value)

    def read_heater_output(self):
        # TODO: check that this really works, else next line
        return self.query('HTR:SIG:POWR')
        # return self.query('HTR:SIG:VOLT') ** 2 / self.heater_resistivity

    def write_heater_output(self, value):
        if self.mode != 'manual':
            self.log.warning('switch to manual heater mode')
            self.write_mode('manual')
        return self.query('HTR:SIG:VOLT', math.sqrt(value * self.heater_resistivity))


class PressureSensor(MercuryChannel, Readable):
    channel_type = 'PRES'
    value = Parameter(unit='mbar')

    def read_value(self):
        return self.query('PRES:SIG:PRES')


class PressureLoop(Loop, PressureSensor, Drivable):
    channel_type = 'PRES,AUX'

    valve_pos = Parameter('valve position', FloatRange(0, 100, unit='%'), readonly=False)

    def read_valve_pos(self):
        return self.query('AUX:SIG:PERC')

    def write_valve_pos(self, value):
        if self.mode != 'manual':
            self.log.warning('switch to manual valve mode')
            self.write_mode('manual')
        return self.query('AUX:SIG:PERC', value)

    def write_target(self, value):
        self.reset_progress(self.value, value)
        return self.query('PRES:LOOP:PRST', value)


class HeLevel(MercuryChannel, Readable):
    channel_type = 'LVL'
    sample_rate = Parameter('_', EnumType(slow=0, fast=1), readonly=False, poll=True)
    hysteresis = Parameter('hysteresis for detection of increase', FloatRange(0, 100, unit='%'), readonly=False)
    fast_timeout = Parameter('timeout for switching to slow', FloatRange(0, unit='sec'), readonly=False)
    _min_level = 200
    _max_level = -100
    _last_increase = None  # None when in slow mode, last increase time in fast mode

    def check_rate(self, sample_rate):
        """check changes in rate

        :param sample_rate:  (int or enum) 0: slow, 1: fast
        initialize affected attributes
        """
        if sample_rate != 0:  # fast
            if not self._last_increase:
                self._last_increase = time.time()
                self._max_level = -100
        elif self._last_increase:
            self._last_increase = None
            self._min_level = 200
        return sample_rate

    def read_sample_rate(self):
        return self.check_rate(SAMPLE_RATE[self.query('LVL:HEL:PULS:SLOW')])

    def write_sample_rate(self, value):
        self.check_rate(value)
        return SAMPLE_RATE[self.query('LVL:HEL:PULS:SLOW', SAMPLE_RATE[value])]

    def read_value(self):
        level = self.query('LVL:SIG:HEL:LEV')
        # handle automatic switching depending on increase
        now = time.time()
        if self._last_increase:  # fast mode
            if level > self._max_level:
                self._last_increase = now
                self._max_level = level
            elif now > self._last_increase + self.fast_timeout:
                # no increase since fast timeout -> slow
                self.write_sample_rate('slow')
        else:
            if level > self._min_level + self.hysteresis:
                # substantial increase -> fast
                self.write_sample_rate('fast')
            else:
                self._min_level = min(self._min_level, level)
        return level


class N2Level(MercuryChannel, Readable):
    channel_type = 'LVL'

    def read_value(self):
        return self.query('LVL:SIG:NIT:LEV')


class MagnetOutput(MercuryChannel, Drivable):
    pass
