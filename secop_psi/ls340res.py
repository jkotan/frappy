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
"""very simple LakeShore Model 340 driver, resistivity only"""

import time

from secop.datatypes import StringType, FloatRange
from secop.modules import Parameter, Property, Readable
from secop.io import HasIodev, StringIO


class LscIO(StringIO):
    identification = [('*IDN?', 'LSCI,MODEL340,.*')]
    end_of_line = '\r'
    wait_before = 0.05


class ResChannel(HasIodev, Readable):
    """temperature channel on Lakeshore 340"""

    iodevClass = LscIO

    value = Parameter(datatype=FloatRange(unit='Ohm'))
    channel = Property('the channel A,B,C or D', StringType())

    def read_value(self):
        return self._iodev.communicate('SRDG?%s' % self.channel)
