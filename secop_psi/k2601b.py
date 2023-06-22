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
"""Keithley 2601B source meter

not tested yet"""

from secop.core import Attached, BoolType, EnumType, FloatRange, \
    HasIodev, Module, Parameter, StringIO, Writable, Done


class K2601bIO(StringIO):
    identification = [('print(localnode.description)', 'Keithley Instruments SMU 2601B.*')]


SOURCECMDS = {
    0: 'reset()'
       ' smua.source.func = smua.OUTPUT_DCAMPS'
       ' display.smua.measure.func = display.MEASURE_VOLTS'
       ' smua.source.autorangei = 1'
       ' smua.source.output = %d print("ok"")',
    1: 'reset()'
       ' smua.source.func = smua.OUTPUT_DCVOLTS'
       ' display.smua.measure.func = display.MEASURE_DCAMPS'
       ' smua.source.autorangev = 1'
       ' smua.source.output = %d print("ok"")',
}


class SourceMeter(HasIodev, Module):

    resistivity = Parameter('readback resistivity', FloatRange(unit='Ohm'), poll=True)
    power = Parameter('readback power', FloatRange(unit='W'), poll=True)
    mode = Parameter('measurement mode', EnumType(current_off=0, voltage_off=1, current_on=2, voltage_on=3),
                     readonly=False, poll=True)

    iodevClass = K2601bIO

    def read_resistivity(self):
        return self.sendRecv('print(smua.measure.r())')

    def read_power(self):
        return self.sendRecv('print(smua.measure.p())')

    def read_mode(self):
        return float(self.sendRecv('print(smua.source.func+2*smua.source.output)'))

    def write_mode(self, value):
        assert 'ok' == self.sendRecv(SOURCECMDS[value % 2] % (value >= 2))
        return self.read_mode()


class Current(HasIodev, Writable):
    sourcemeter = Attached()

    value = Parameter('measured current', FloatRange(unit='A'), poll=True)
    target = Parameter('set current', FloatRange(unit='A'), poll=True)
    active = Parameter('current is controlled', BoolType(), default=False)  # polled by SourceMeter
    limit = Parameter('current limit', FloatRange(0, 2.0, unit='A'), default=2, poll=True)

    def initModule(self):
        self._sourcemeter.registerCallbacks(self)

    def read_value(self):
        return self.sendRecv('print(smua.measure.i())')

    def read_target(self):
        return self.sendRecv('print(smua.source.leveli)')

    def write_target(self, value):
        if not self.active:
            raise ValueError('current source is disabled')
        if value > self.limit:
            raise ValueError('current exceeds limit')
        return self.sendRecv('smua.source.leveli = %g print(smua.source.leveli)' % value)

    def read_limit(self):
        if self.active:
            return self.limit
        return self.sendRecv('print(smua.source.limiti)')

    def write_limit(self, value):
        if self.active:
            return value
        return self.sendRecv('smua.source.limiti = %g print(smua.source.limiti)' % value)

    def write_active(self, value):
        if value:
            self._sourcemeter.write_mode('current_on')
        elif self._sourcemeter.mode == 'current_on':
            self._sourcemeter.write_mode('current_off')
        return self.active

    def update_mode(self, mode):
        # will be called whenever the attached sourcemeters mode changes
        self.active = mode == 'current_on'


class Voltage(HasIodev, Writable):
    sourcemeter = Attached()

    value = Parameter('measured voltage', FloatRange(unit='V'), poll=True)
    target = Parameter('set voltage', FloatRange(unit='V'), poll=True)
    active = Parameter('voltage is controlled', BoolType(), default=False)  # polled by SourceMeter
    limit = Parameter('current limit', FloatRange(0, 2.0, unit='V'), default=2, poll=True)

    def initModule(self):
        self._sourcemeter.registerCallbacks(self)

    def read_value(self):
        return self.sendRecv('print(smua.measure.v())')

    def read_target(self):
        return self.sendRecv('print(smua.source.levelv)')

    def write_target(self, value):
        if not self.active:
            raise ValueError('voltage source is disabled')
        if value > self.limit:
            raise ValueError('voltage exceeds limit')
        return self.sendRecv('smua.source.levelv = %g print(smua.source.levelv)' % value)

    def read_limit(self):
        if self.active:
            return self.limit
        return self.sendRecv('print(smua.source.limitv)')

    def write_limit(self, value):
        if self.active:
            return value
        return self.sendRecv('smua.source.limitv = %g print(smua.source.limitv)' % value)

    def write_active(self, value):
        if value:
            self._sourcemeter.write_mode('voltage_on')
        elif self._sourcemeter.mode == 'voltage_on':
            self._sourcemeter.write_mode('voltage_off')
        return self.active

    def update_mode(self, mode):
        # will be called whenever the attached sourcemeters mode changes
        self.active = mode == 'voltage_on'
