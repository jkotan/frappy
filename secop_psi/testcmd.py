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
"""Test command arguments"""

from secop.core import ArrayOf, BoolType, Command, FloatRange, \
    Module, Parameter, StringType, StructOf, TupleOf


class TestCmd(Module):

    struct = Parameter('struct', StructOf(a=StringType(), b=FloatRange(), c=BoolType(), optional=['b']),
                       readonly=False, default=dict(a='', c=True))
    array = Parameter('array', ArrayOf(BoolType()),
                      readonly=False, default=[])
    tuple = Parameter('tuple', TupleOf(StringType(), FloatRange(), BoolType(),
                                       TupleOf(BoolType()), StructOf(a=StringType())),
                      readonly=False, default=('', 0, False, (False,), dict(a='')))

    @Command(argument=TupleOf(StringType(), FloatRange(), BoolType(), TupleOf(BoolType()), StructOf(a=StringType())),
             result=StringType())
    def arg(self, *arg):
        """5 args"""
        self.tuple = arg
        return repr(arg)

    @Command(argument=StructOf(a=StringType(), b=FloatRange(), c=BoolType(), optional=['b']),
             result=StringType())
    def keyed(self, **arg):
        """keyworded arg"""
        self.struct = arg
        return repr(arg)

    @Command(argument=FloatRange(), result=StringType())
    def one(self, arg):
        """1 arg"""
        return repr(arg)

    @Command(result=StringType())
    def none(self):
        """no arg"""
        return repr(None)
