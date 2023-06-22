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
#   Daniel Margineda <daniel.margineda@psi.ch>
# *****************************************************************************
"""Signal Recovery SR7270: lockin amplifier for AC susceptibility"""

from secop.core import Readable, Parameter, Command, FloatRange, TupleOf, \
    HasIodev, StringIO, Attached, IntRange, BoolType, EnumType


class SR7270(StringIO):
    end_of_line = b'\x00'

    def communicate(self, command):  # remove dash from terminator
        reply = StringIO.communicate(self, command)
        status = self._conn.readbytes(2, 0.1)  # get the 2 status bytes
        return reply + ';%d;%d' % tuple(status)


class XY(HasIodev, Readable):
    x = Attached()
    y = Attached()
    freq_arg = Attached()
    amp_arg = Attached()
    tc_arg = Attached()
    phase_arg = Attached()
    dac_arg = Attached()

    # parameters required an initial value but initwrite write the default value for polled parameters
    value = Parameter('X, Y', datatype=TupleOf(FloatRange(unit='V'), FloatRange(unit='V')))
    freq = Parameter('exc_freq_int',
                     FloatRange(0.001, 250e3, unit='Hz'),
                     poll=True, readonly=False, initwrite=True, default=1000)
    amp = Parameter('exc_volt_int',
                    FloatRange(0.00, 5, unit='Vrms'),
                    poll=True, readonly=False, initwrite=True, default=0.1)
    range = Parameter('sensitivity value', FloatRange(0.00, 1, unit='V'), poll=True, default=1)
    irange = Parameter('sensitivity index', IntRange(0, 27), poll=True, readonly=False, default=25)
    autorange = Parameter('autorange_on', EnumType('autorange', off=0, soft=1, hard=2),
                          readonly=False, default=0, initwrite=True)
    tc = Parameter('time constant value', FloatRange(10e-6, 100, unit='s'), poll=True, default=0.1)
    itc = Parameter('time constant index', IntRange(0, 30), poll=True, readonly=False, initwrite=True, default=14)
    nm = Parameter('noise mode', BoolType(), readonly=False, default=0)
    phase = Parameter('Reference phase control', FloatRange(-360, 360, unit='deg'),
                      poll=True, readonly=False, initwrite=True, default=0)
    vmode = Parameter('Voltage input configuration', IntRange(0, 3), readonly=False, default=3),
    # dac = Parameter('output DAC channel value', datatype=TupleOf(IntRange(1, 4), FloatRange(0.0, 5000, unit='mV')),
    #                 poll=True, readonly=False, initwrite=True, default=(3,0))
    dac = Parameter('output DAC channel value', FloatRange(-10000, 10000, unit='mV'),
                    poll=True, readonly=False, initwrite=True, default=0)

    iodevClass = SR7270

    def comm(self, command):
        reply, status, overload = self.sendRecv(command).split(';')
        if overload != '0':
            self.status = self.Status.WARN, 'overload %s' % overload
        else:
            self.status = self.Status.IDLE, ''
        return reply

    def read_value(self):
        reply = self.comm('XY.').split(',')
        x = float(reply[0])
        y = float(reply[1])
        if self.autorange == 1:  # soft
            if max(abs(x), abs(y)) >= 0.9*self.range and self.irange < 27:
                self.write_irange(self.irange+1)
            elif max(abs(x), abs(y)) <= 0.3*self.range and self.irange > 1:
                self.write_irange(self.irange-1)
        self._x.value = x  # to update  X,Y classes which will be the collected data.
        self._y.value = y
        return x, y

    def read_freq(self):
        reply = self.comm('OF.')
        return reply

    def write_freq(self, value):
        self.comm('OF. %g' % value)
        return value

    def write_autorange(self, value):
        if value == 2:  # hard
            self.comm('AS')  # put hardware autorange on
            self.comm('AUTOMATIC. 1')
        else:
            self.comm('AUTOMATIC. 0')
        return value

    def read_autorange(self):
        reply = self.comm('AUTOMATIC')
        # determine hardware autorange
        if reply == 1:  # "hardware auto range is on"
            return 2  # hard
        if self.autorange == 0:  # soft
            return self.autorange()  # read autorange
        return reply  # off

    # oscillator amplitude module
    def read_amp(self):
        reply = self.comm('OA.')
        return reply

    def write_amp(self, value):
        self.comm('OA. %g' % value)
        return value

    # external output DAC
    def read_dac(self):
        # reply = self.comm('DAC  %g' % channel) # failed to add the DAC channel you want to control
        reply = self.comm('DAC 3')  # stack to channel 3
        return reply

    def write_dac(self, value):
        # self.comm('DAC %g %g' % channel % value)
        self.comm('DAC 3 %g' % value)
        return value

    # sensitivity module
    def read_range(self):
        reply = self.comm('SEN.')
        return reply

    def write_irange(self, value):
        self.comm('SEN %g' % value)
        self.read_range()
        return value

    def read_irange(self):
        reply = self.comm('SEN')
        return reply

    # time constant module/ noisemode off or 0 allows to use all the time constant range
    def read_nm(self):
        reply = self.comm('NOISEMODE')
        return reply

    def write_nm(self, value):
        self.comm('NOISEMODE %d' % int(value))
        self.read_nm()
        return value

    def read_tc(self):
        reply = self.comm('TC.')
        return reply

    def write_itc(self, value):
        self.comm('TC %g' % value)
        self.read_tc()
        return value

    def read_itc(self):
        reply = self.comm('TC')

        return reply

    # phase and autophase
    def read_phase(self):
        reply = self.comm('REFP.')
        return reply

    def write_phase(self, value):
        self.comm('REFP %d' % round(1000*value, 0))
        self.read_phase()
        return value

    @Command()
    def aphase(self):
        """auto phase"""
        self.read_phase()
        reply = self.comm('AQN')
        self.read_phase()

    # voltage input configuration 0:grounded,1=A,2=B,3=A-B
    # def read_vmode(self):
    #     reply = self.comm('VMODE')
    #     return reply

    def write_vmode(self, value):
        self.comm('VMODE %d' % value)
        # self.read_vmode()
        return value


class Comp(Readable):
    pollerClass = None
    value = Parameter(datatype=FloatRange(unit='V'))


class arg(Readable):
    pollerClass = None
    value = Parameter(datatype=FloatRange(unit=''))
