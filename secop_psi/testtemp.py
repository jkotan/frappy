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
"""Temp"""

from secop.datatypes import FloatRange, IntRange, StringType
from secop.modules import Drivable, Parameter, Readable
from secop.io import HasIodev

Status = Drivable.Status


class TempLoop(HasIodev, Drivable):
    """temperature channel on Lakeshore 336"""

    value = Parameter(datatype=FloatRange(unit='K'), default=0, poll=True)
    status = Parameter(poll=False)
    target = Parameter(datatype=FloatRange(1.0, 402.0, unit='K'), default=1.3, poll=True)
    tolerance = Parameter('the tolerance', FloatRange(-400, 400), default=1, readonly=False)
    pollinterval = Parameter(visibility=3)
    channel = Parameter('the Lakeshore channel', datatype=StringType(), export=False)
    loop = Parameter('the Lakeshore loop number', datatype=IntRange(1, 3), export=False)

    def earlyInit(self):
        super(TempLoop, self).earlyInit()
        self.status = [Status.IDLE, 'idle']

    def read_value(self):
        result = self.sendRecv('KRDG?%s' % self.channel)
        result = float(result)
        if self.status == Status.BUSY and abs(result - self.target) < self.tolerance:
            self.status = [Status.IDLE, 'reached target']
        return result

    def read_target(self):
        result = self.sendRecv('SETP?%d' % self.loop)
        return float(result)

    def write_target(self, value):
        result = float(self.sendRecv('SETP %d,%g;SETP? %d' % (self.loop, value, self.loop)))
        self.status = [Status.BUSY, 'changed target']
        float('x')
        return result

    def stop(self):
        self.target = self.value
        self.status = [Status.IDLE, 'stopped']


class TempChannel(HasIodev, Readable):
    """temperature channel on Lakeshore 336"""

    value = Parameter(datatype=FloatRange(unit='K'), default=0, poll=True)
    status = Parameter(poll=False, constant=[Status.IDLE, 'idle'])
    pollinterval = Parameter(visibility=3)
    channel = Parameter('the Lakeshore channel', datatype=StringType(), export=False)

    def read_value(self):
        result = self.sendRecv('KRDG?%s' % self.channel)
        result = float(result)
        return result
