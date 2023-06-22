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
"""LakeShore 240 temperature monitor"""

import struct
from secop.core import FloatRange, HasIodev, Readable, Parameter, BytesIO
from secop.errors import CommunicationFailedError

SD1= 0x10
SD2= 0x68
FC = 0x49
ED = 0x16

STATUS_REQ = 0x49

def dehex(msg):
    return bytes(int(b, 16) for b in msg.split())

def hexify(msg):
    return ' '.join('%2.2X' % b for b in msg)


class Ls240(HasIodev, Readable):
    value = Parameter('sensor reading', FloatRange(unit='Ohm'))

    iodevClass = BytesIO

    def request(self, replylen, data='', ext=None, dst_adr=3, src_adr=2):
        data = dehex(data)
        if ext is None:
            ext = len(data) > 1
        if ext:
            dst_adr |= 0x80
            src_adr |= 0x80
        if len(data) > 1:
            length = len(data) + 2
            hdr = [SD2, length, length, SD2]
        else:
            hdr = [SD1]
        mid = [dst_adr, src_adr] + list(data)
        checksum = sum(mid) % 256
        msg = bytes(hdr + mid + [checksum, ED])
        for i in range(10):
             try:
                 # print('>', hexify(msg))
                 reply = self._iodev.communicate(msg, replylen)
                 # print('<', hexify(reply))
             except (TimeoutError, CommunicationFailedError):
                 continue
             return reply
        return None

    def read_value(self):

        # check connection
        self.request(6, '49')

        # get diag
        # 3C: slave diag, (what means 3E?) 
        reply = self.request(17, '6D 3C 3E')
        assert reply[13:15] == b'\x0f\x84'  # LS240 ident

        # set parameters
        # 3D set param (what means 3E?)
        # B0 FF FF: no watchdog, 00: min wait, 0F 84: ident, 01: group
        assert b'\xe5' ==  self.request(1, '5D 3D 3E B0 FF FF 00 0F 84 01')

        # set config
        # 3E set config (what means 2nd 3E?)
        # 93: input only, 4 == 3+1 bytes
        assert b'\xe5' ==  self.request(1, '7D 3E 3E 93')

        # get diag
        # 3C: slave diag, (what means 3E?) 
        reply = self.request(17, '5D 3C 3E')
        assert reply[13:15] == b'\x0f\x84'  # LS240 ident

        # get data
        # do not know what 42 24 means
        reply = self.request(13, '7D 42 24', ext=0)
        print('DATA', reply)
        value = struct.unpack('>f', reply[7:11])[0]
        print('VALUE', value)
        return value
