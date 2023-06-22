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
"""Delay generator stanford 645"""

from secop.core import FloatRange, HasIodev, Module, Parameter, StringIO


class DG645(StringIO):
    end_of_line = '\n'


class Delay(HasIodev, Module):

    on1 = Parameter('on delay 1', FloatRange(unit='sec'), readonly=False, default=0)
    off1 = Parameter('off delay 1', FloatRange(unit='sec'), readonly=False, default=60e-9)
    on2 = Parameter('on delay 2', FloatRange(unit='sec'), readonly=False, default=0)
    off2 = Parameter('off delay 2', FloatRange(unit='sec'), readonly=False, default=150e-9)

    iodevClass = DG645

    def read_on1(self):
        return self.sendRecv('DLAY?2').split(',')[1]

    def read_off1(self):
        return self.sendRecv('DLAY?3').split(',')[1]

    def read_on2(self):
        return self.sendRecv('DLAY?4').split(',')[1]

    def read_off2(self):
        return self.sendRecv('DLAY?5').split(',')[1]

    def write_on1(self, value):
        return self.sendRecv('DLAY 2,0,%g;DLAY?2' % value).split(',')[1]

    def write_off1(self, value):
        result = self.sendRecv('DLAY 3,0,%g;DLAY?3' % value)
        return result.split(',')[1]

    def write_on2(self, value):
        return self.sendRecv('DLAY 4,0,%g;DLAY?4' % value).split(',')[1]

    def write_off2(self, value):
        return self.sendRecv('DLAY 5,0,%g;DLAY?5' % value).split(',')[1]
