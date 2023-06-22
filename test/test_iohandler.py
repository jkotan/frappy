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
#   Markus Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************
"""test commandhandler."""

import pytest

from secop.datatypes import FloatRange, IntRange, Property, StringType
from secop.errors import ProgrammingError
from secop.iohandler import CmdParser, IOHandler
from secop.modules import Module, Parameter


@pytest.mark.parametrize('fmt, text, values, text2', [
        ('%d,%d', '2,3', [2,3], None),
        ('%c%s', '222', ['2', '22'], None), # %c does not need a spearator
        ('%s%%%d_%%', 'abc%12_%', ['abc', 12], None), # % as separator and within separator
        ('%s.+', 'string,without+period.+', ['string,without+period',], None), # special characters
        ('%d,%.3f,%x,%o', '1,1.2346,fF,17', [1, 1.2346, 255, 15], '1,1.235,ff,17'), # special formats
    ])
def test_CmdParser(fmt, text, values, text2):
    parser = CmdParser(fmt)
    print(parser.format(*values))
    assert parser.parse(text) == values
    if text2 is None:
        text2 = text
    assert parser.format(*values) == text2

def test_CmdParser_ex():
    with pytest.raises(ValueError):
        CmdParser('%d%s') # no separator


class Data:
    """a cyclic list where we put data to be checked or used during test"""
    def __init__(self):
        self.data = []

    def push(self, tag, *args):
        if len(args) == 1:
            args = args[0]
        self.data.append((tag, args))

    def pop(self, expected):
        tag, data = self.data.pop(0)
        print('pop(%s) %r' % (tag, data))
        if tag != expected:
            raise ValueError('expected tag %s, not %s' % (expected, tag))
        return data

    def empty(self):
        return not self.data


class DispatcherStub:
    OMIT_UNCHANGED_WITHIN = 0

    def __init__(self, updates):
        self.updates = updates

    def announce_update(self, module, pname, pobj):
        if pobj.readerror:
            self.updates['error', pname] = str(pobj.readerror)
        else:
            self.updates[pname] = pobj.value


class LoggerStub:
    def debug(self, *args):
        pass
    info = warning = exception = debug


class ServerStub:
    def __init__(self, updates):
        self.dispatcher = DispatcherStub(updates)


def test_IOHandler():
    class Hdl(IOHandler):
        CMDARGS = ['channel', 'loop']
        CMDSEPARATOR ='|'

        def __init__(self, name, querycmd, replyfmt):
            changecmd = querycmd.replace('?', ' ')
            if not querycmd.endswith('?'):
                changecmd += ','
            super().__init__(name, querycmd, replyfmt, changecmd)

    group1 = Hdl('group1', 'SIMPLE?', '%g')
    group2 = Hdl('group2', 'CMD?%(channel)d', '%g,%s,%d')

    class Module1(Module):
        channel = Property('the channel', IntRange(), default=3)
        loop = Property('the loop', IntRange(), default=2)
        simple = Parameter('a readonly', FloatRange(), default=0.77, handler=group1)
        real = Parameter('a float value', FloatRange(), default=12.3, handler=group2, readonly=False)
        text = Parameter('a string value', StringType(), default='x', handler=group2, readonly=False)

        def sendRecv(self, command):
            assert data.pop('command') == command
            return data.pop('reply')

        def analyze_group1(self, val):
            assert data.pop('val') == val
            return dict(simple=data.pop('simple'))

        def analyze_group2(self, gval, sval, dval):
            assert data.pop('gsv') == (gval, sval, dval)
            real, text = data.pop('rt')
            return dict(real=real, text=text)

        def change_group2(self, change):
            gval, sval, dval = change.readValues()
            assert data.pop('old') == (gval, sval, dval)
            assert data.pop('self') == (self.real, self.text)
            assert data.pop('new') == (change.real, change.text)
            return data.pop('changed')

    data = Data()
    updates = {}
    module = Module1('mymodule', LoggerStub(), {'.description': ''}, ServerStub(updates))
    print(updates)
    updates.clear() # get rid of updates from initialisation

    # for sendRecv
    data.push('command', 'SIMPLE?')
    data.push('reply', '4.51')
    # for analyze_group1
    data.push('val', 4.51)
    data.push('simple', 45.1)
    value = module.read_simple()
    assert value == 45.1
    assert module.simple == value
    assert data.empty()
    assert updates.pop('simple') == 45.1
    assert not updates

    # for sendRecv
    data.push('command', 'CMD?3')
    data.push('reply', '1.23,text,5')
    # for analyze_group2
    data.push('gsv', 1.23, 'text', 5)
    data.push('rt', 12.3, 'string')
    value = module.read_real()
    assert module.real == value
    assert module.real == updates.pop('real') == 12.3
    assert module.text == updates.pop('text') == 'string'
    assert data.empty()
    assert not updates

    # for sendRecv
    data.push('command', 'CMD?3')
    data.push('reply', '1.23,text,5')
    # for analyze_group2
    data.push('gsv', 1.23, 'text', 5)
    data.push('rt', 12.3, 'string')
    # for change_group2
    data.push('old', 1.23, 'text', 5)
    data.push('self', 12.3, 'string')
    data.push('new', 12.3, 'FOO')
    data.push('changed', 1.23, 'foo', 9)
    # for sendRecv
    data.push('command', 'CMD 3,1.23,foo,9|CMD?3')
    data.push('reply', '1.23,foo,9')
    # for analyze_group2
    data.push('gsv', 1.23, 'foo', 9)
    data.push('rt', 12.3, 'FOO')
    value = module.write_text('FOO')
    assert module.text == value
    assert module.text == updates.pop('text') == 'FOO'
    assert module.real == updates.pop('real')  == 12.3
    assert data.empty()
    assert not updates

    with pytest.raises(ProgrammingError): # can not use a handler for different modules
        # pylint: disable=unused-variable
        class Module2(Module):
            simple = Parameter('a readonly', FloatRange(), default=0.77, handler=group1)
