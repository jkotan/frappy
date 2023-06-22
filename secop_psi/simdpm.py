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
"""simulated transducer DPM3 read out"""

import random
import math
from secop.core import Readable, Parameter, FloatRange, Attached
from secop.lib import clamp


class DPM3(Readable):
    motor = Attached()
    jitter = Parameter('simulated jitter', FloatRange(unit='N'), default=0.1, readonly=False)
    hysteresis = Parameter('simulated hysteresis', FloatRange(unit='deg'), default=100, readonly=False)
    friction = Parameter('friction', FloatRange(unit='N/deg'), default=2.5, readonly=False)
    slope = Parameter('slope', FloatRange(unit='N/deg'), default=0.5, readonly=False)
    offset = Parameter('offset', FloatRange(unit='N'), default=0, readonly=False)

    _pos = 0

    def read_value(self):
        mot = self._motor
        d = self.friction * self.slope
        self._pos = clamp(self._pos, mot.value - d, mot.value + d)
        f = (mot.value - self._pos) / self.slope
        if mot.value > 0:
            f = max(f, (mot.value - self.hysteresis) / self.slope)
        else:
            f = min(f, (mot.value + self.hysteresis) / self.slope)
        return f + self.jitter * (random.random() - random.random()) * 0.5
