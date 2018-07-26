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
#   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
#
# *****************************************************************************
"""test data types."""
from __future__ import print_function

import sys
sys.path.insert(0, sys.path[0] + '/..')

# no fixtures needed
import pytest


from secop.datatypes import BoolType, EnumType

from secop.metaclass import ModuleMeta
from secop.params import Command, Parameter, Override
from secop.modules import Module, Readable, Writable, Drivable, Communicator


def test_Communicator():
    logger = type('LoggerStub', (object,), dict(
        debug = lambda self, *a: print(*a),
        info = lambda self, *a: print(*a),
    ))()

    dispatcher = type('DispatcherStub', (object,), dict(
        announce_update = lambda self, m, pn, pv: print('%s:%s=%r' % (m.name, pn, pv)),
    ))()

    o = Communicator(logger, {}, 'o1', dispatcher)
    o.init()

def test_ModuleMeta():
    newclass = ModuleMeta.__new__(ModuleMeta, 'TestReadable', (Drivable, Writable, Readable, Module), {
        "parameters" : {
            'param1' : Parameter('param1', datatype=BoolType(), default=False),
            'param2': Parameter('param2', datatype=BoolType(), default=True),
        },
        "commands": {
            "cmd": Command('stuff',[BoolType()], BoolType())
        },
        "accessibles": {
            'a1': Parameter('a1', datatype=BoolType(), default=False),
            'a2': Parameter('a2', datatype=BoolType(), default=True),
            'value':Override(datatype=BoolType(), default = True),
            'cmd2': Command('another stuff', [BoolType()], BoolType()),
        },
        "do_cmd": lambda self, arg: not arg,
        "do_cmd2": lambda self, arg: not arg,
        "read_param1": lambda self, *args: True,
        "read_param2": lambda self, *args: False,
        "read_a1": lambda self, *args: True,
        "read_a2": lambda self, *args: False,
        "read_value": lambda self, *args: True,
        "init": lambda self, *args: [None for self.accessibles['value'].datatype in [EnumType('value', OK=1, Bad=2)]],
    })
    # every cmd/param has to be collected to accessibles
    assert newclass.accessibles
    with pytest.raises(AttributeError):
        assert newclass.commands
    with pytest.raises(AttributeError):
        assert newclass.parameters

    logger = type('LoggerStub', (object,), dict(
        debug = lambda self, *a: print(*a),
        info = lambda self, *a: print(*a),
    ))()

    dispatcher = type('DispatcherStub', (object,), dict(
        announce_update = lambda self, m, pn, pv: print('%s:%s=%r' % (m.name, pn, pv)),
    ))()

    o1 = newclass(logger, {}, 'o1', dispatcher)
    o2 = newclass(logger, {}, 'o1', dispatcher)
    params_found= set()
    ctr_found = set()
    for obj in [o1, o2]:
        for n, o in obj.accessibles.items():
            print(n)
            assert o not in params_found
            params_found.add(o)
            assert o.ctr not in ctr_found
            ctr_found.add(o.ctr)
    o1.init()
    o2.init()
