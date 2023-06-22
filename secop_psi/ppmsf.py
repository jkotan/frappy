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
"""PPMS mf proxy"""

import secop_psi.ppms
from secop.core import Drivable, Enum, EnumType, FloatRange, Override, Parameter
from secop.datatypes import StatusType
from secop.proxy import proxy_class


class Field(proxy_class(secop_psi.ppms.Field)):
    """magnetic field"""

    # pylint: disable=invalid-name
    Status = Enum(Drivable.Status,
        PREPARED = 150,
        PREPARING = 340,
        RAMPING = 370,
        FINALIZING = 390,
    )
    # pylint: disable=invalid-name
    PersistentMode = Enum('PersistentMode', persistent=0, driven=1)
    ApproachMode = Enum('ApproachMode', linear=0, no_overshoot=1, oscillate=2)

    remoteParameters = {
        'value':
            Override(datatype=FloatRange(-1, 1, unit='T'), poll=True),
        'status':
            Override(datatype=StatusType(Status), poll=True),
        'target':
            Override(datatype=FloatRange(-15, 15, unit='T'), poll=True),
        'ramp':
            Parameter('ramping speed', readonly=False,
                      datatype=FloatRange(0.064, 1.19, unit='T/min'), poll=True),
        'approachmode':
            Parameter('how to approach target', readonly=False,
                      datatype=EnumType(ApproachMode), poll=True),
        'persistentmode':
            Parameter('what to do after changing field', readonly=False,
                      datatype=EnumType(PersistentMode), poll=True),
        'pollinterval':
            Override(visibility=3),
    }
