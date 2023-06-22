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
"""WAVE FUNCTION LECROY  XX: SIGNAL GENERATOR"""

from secop.core import Readable, Parameter, FloatRange, \
    IntRange, BoolType, EnumType, Module, Property


class Channel(Module):
    channel = Property('choose channel to manipulate', IntRange(1, 2))
    freq = Parameter('frequency', FloatRange(1e-6, 20e6, unit='Hz'),
                     poll=True, initwrite=True, default=1000)
    amp = Parameter('exc_volt_int', FloatRange(0.00, 5, unit='Vrms'),
                    poll=True, readonly=False, initwrite=True, default=0.1)
    offset = Parameter('offset_volt_int', FloatRange(0.0, 10, unit='V'),
                       poll=True, readonly=False, initwrite=True, default=0.0)
    wave = Parameter('type of wavefunction',
                     EnumType('WaveFunction', SINE=1, SQUARE=2, RAMP=3, PULSE=4, NOISE=5, ARB=6, DC=7),
                     poll=True, readonly=False, default='SINE')
    phase = Parameter('signal phase', FloatRange(0, 360, unit='deg'),
                      poll=True, readonly=False, initwrite=True, default=0)
    enabled = Parameter('enable output channel', datatype=EnumType('OnOff', OFF=0, ON=1),
                        readonly=False, default='OFF')
    symm = Parameter('wavefunction symmetry', FloatRange(0, 100, unit=''),
                     poll=True, readonly=False, default=0)

    def read_value(self):
        return self.sendRecv('C%d:BSWV FRQ?' % self.channel)

    def write_target(self, value):
        self.sendRecv('C%d:BSWV FRQ, %gHz' % (self.channel, value))
        return value

    # signal wavefunction parameter
    def read_wave(self):
        return self.sendRecv('C%d:BSWV WVTP?' % self.channel)

    def write_wave(self, value):  # string value
        self.sendRecv('C%d:BSWV WVTP, %s' % (self.channel, value.name))
        return value

    # signal amplitude parameter
    def read_amp(self):
        return self.sendRecv('C%d:BSWV AMP?' % self.channel)

    def write_amp(self, value):
        self.sendRecv('C%d:BSWV AMP, %g' % (self.channel, value))
        return value

    # offset value parameter
    def read_offset(self):
        return self.sendRecv('C%d:BSWV OFST?' % self.channel)

    def write_offset(self, value):
        self.sendRecv('C%d:BSWV OFST %g' % (self.channel, value))
        return value

    # channel symmetry
    def read_symm(self):
        return self.sendRecv('C%d:BSWV SYM?' % self.channel)

    def write_symm(self, value):
        self.sendRecv('C%d:BSWV SYM %g' % (self.channel, value))
        return value

    # wave phase parameter
    def read_phase(self):
        return self.sendRecv('C%d:BSWV PHSE?' % self.channel)

    def write_phase(self, value):
        self.sendRecv('C%d:BSWV PHSE %g' % (self.channel, value))
        return value

    # dis/enable output channel
    def read_enabled(self):
        return self.sendRecv('C%d: OUTP?' % self.channel)

    def write_enabled(self, value):
        self.sendRecv('C%d: OUTP %s' % (self.channel, value.name))
        return value


# devices are defined as arg less output enable what is defined as arg2

class arg(Readable):
    pollerClass = None
    value = Parameter(datatype=FloatRange(unit=''))


class arg2(Readable):
    pollerClass = None
    value = Parameter(datatype=BoolType())
