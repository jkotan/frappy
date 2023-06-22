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

"""drivers for trinamic PD-1161 motors"""

import time
import struct
from math import log10
from secop.core import BoolType, Command, EnumType, FloatRange, IntRange, \
    HasIodev, Parameter, Property, Drivable, TupleOf, Done
from secop.bytesio import BytesIO
from secop.errors import CommunicationFailedError, HardwareError, BadValueError, IsBusyError


MOTOR_STOP = 3
MOVE = 4
SET_AXIS_PAR = 5
GET_AXIS_PAR = 6
SET_GLOB_PAR = 9
GET_GLOB_PAR = 10
# STORE_GLOB_PAR = 11

BAUDRATES = [9600, 0, 19200, 0, 38400, 57600, 0, 115200]

ANGLE_SCALE = 1.8/256
# assume factory settings for pulse and ramp divisors:
SPEED_SCALE = 1E6 / 2 ** 15 * ANGLE_SCALE
MAX_SPEED = 2047 * SPEED_SCALE
ACCEL_SCALE = 1E12 / 2 ** 31 * ANGLE_SCALE
MAX_ACCEL = 2047 * ACCEL_SCALE
CURRENT_SCALE = 2.8/250
ENCODER_RESOLUTION = 0.4  # 365 / 1024, rounded up


class HwParam(Parameter):
    adr = Property('parameter address', IntRange(0, 255), export=False)
    scale = Property('parameter address', FloatRange(), export=False)

    def __init__(self, description, datatype, adr, scale=1, poll=True, persistent=True, **kwds):
        """hardware parameter"""
        if isinstance(datatype, FloatRange) and not kwds.get('fmtstr'):
            datatype.fmtstr = '%%.%df' % max(0, 1 - int(log10(scale)))
        super().__init__(description, datatype, poll=poll, adr=adr, scale=scale,
                         persistent=persistent, **kwds)

    def copy(self):
        res = HwParam(self.description, self.datatype.copy(), self.adr)
        res.name = self.name
        res.init(self.propertyValues)
        return res


class Motor(HasIodev, Drivable):
    address = Property('module address', IntRange(0, 255), default=1)
    fact = Property('gear factor', FloatRange(unit='deg/step'), default=1.8/256)
    # 1.8 deg / 256 microsteps

    # limit_pin_mask = Property('input pin mask for lower/upper limit switch',
    #                           TupleOf(IntRange(0, 15), IntRange(0, 15)),
    #                           default=(8, 0))

    value = Parameter('motor position', FloatRange(unit='deg'))
    zero = Parameter('zero point', FloatRange(unit='$'), readonly=False, default=0, persistent=True)
    encoder = HwParam('encoder reading', FloatRange(unit='$'),
                      209, ANGLE_SCALE, readonly=True, initwrite=False)
    steppos = HwParam('position from motor steps', FloatRange(unit='$'),
                      1, ANGLE_SCALE, readonly=True, initwrite=False)
    target = Parameter('_', FloatRange(unit='$'), default=0)
    movelimit = Parameter('max. angle to drive in one go', FloatRange(unit='$'),
                          readonly=False, default=90, persistent=True)
    tolerance = Parameter('positioning tolerance', FloatRange(unit='$'),
                          readonly=False, default=0.9, persistent=True)
    encoder_tolerance = HwParam('the allowed deviation between steppos and encoder\n\nmust be > tolerance',
                                FloatRange(0, 360., unit='$'),
                                212, ANGLE_SCALE, readonly=False, group='more')
    speed = HwParam('max. speed', FloatRange(0, MAX_SPEED, unit='$/sec'),
                    4, SPEED_SCALE, readonly=False, group='more')
    minspeed = HwParam('min. speed', FloatRange(0, MAX_SPEED, unit='$/sec'),
                       130, SPEED_SCALE, readonly=False, group='motorparam')
    currentspeed = HwParam('current speed', FloatRange(-MAX_SPEED, MAX_SPEED, unit='$/sec'),
                           3, SPEED_SCALE, readonly=True, group='motorparam')
    maxcurrent = HwParam('_', FloatRange(0, 2.8, unit='A'),
                         6, CURRENT_SCALE, readonly=False, group='motorparam')
    standby_current = HwParam('_', FloatRange(0, 2.8, unit='A'),
                              7, CURRENT_SCALE, readonly=False, group='motorparam')
    acceleration = HwParam('_', FloatRange(4.6 * ACCEL_SCALE, MAX_ACCEL, unit='deg/s^2'),
                           5, ACCEL_SCALE, readonly=False, group='motorparam')
    target_reached = HwParam('_', BoolType(), 8, group='hwstatus')
    move_status = HwParam('_', IntRange(0, 3),
                          207, readonly=True, persistent=False, group='hwstatus')
    error_bits = HwParam('_', IntRange(0, 255),
                         208, readonly=True, persistent=False, group='hwstatus')
    free_wheeling = HwParam('_', FloatRange(0, 60., unit='sec'),
                            204, 0.001, readonly=False, group='motorparam')
    baudrate = Parameter('_', EnumType({'%d' % v: i for i, v in enumerate(BAUDRATES)}),
                         readonly=False, default=0, poll=True, visibility=3, group='more')
    pollinterval = Parameter(group='more', persistent=True)

    fast_pollfactor = 0.001  # poll as fast as possible when busy
    _started = 0
    _prevconn = None
    iodevClass = BytesIO
    _calcTimeout = True
    _pos_state = None
    _save_filename = None
    _try_count = 0
    _back = None

    def comm(self, cmd, adr, value=0, bank=0):
        """set or get a parameter

        :param adr: parameter number
        :param cmd: SET command (in the GET case, 1 is added to this)
        :param bank: the bank
        :param value: if given, the parameter is written, else it is returned
        :return: the returned value
        """
        if self._calcTimeout:
            self._calcTimeout = False
            baudrate = getattr(self._iodev._conn.connection, 'baudrate', None)
            if baudrate:
                if baudrate not in BAUDRATES:
                    raise CommunicationFailedError('unsupported baud rate: %d' % baudrate)
                self._iodev.timeout = 0.03 + 200 / baudrate

        for itry in range(3):
            byt = struct.pack('>BBBBi', self.address, cmd, adr, bank, round(value))
            try:
                reply = self._iodev.communicate(byt + bytes([sum(byt) & 0xff]), 9)
            except Exception as e:
                exc = e
                self.log.exception('%s, try again', e)
                continue
            if sum(reply[:-1]) & 0xff != reply[-1]:
                exc = CommunicationFailedError('checksum error')
                continue
            break
        else:
            raise exc
        radr, modadr, status, rcmd, result = struct.unpack('>BBBBix', reply)
        if status != 100:
            self.log.warning('bad status from cmd %r %s: %d', cmd, adr, status)
        if radr != 2 or modadr != self.address or cmd != rcmd:
            raise CommunicationFailedError('bad reply %r to command %s %d' % (reply, cmd, adr))
        return result

    def get(self, pname, **kwds):
        """get parameter"""
        pobj = self.parameters[pname]
        scale = pobj.scale
        return self.comm(GET_AXIS_PAR, pobj.adr, **kwds) * scale

    def set(self, pname, value, check=True, **kwds):
        """set parameter and check result"""
        pobj = self.parameters[pname]
        scale = pobj.scale
        rawvalue = round(value / scale)
        self.comm(SET_AXIS_PAR, pobj.adr, rawvalue, **kwds)
        if check:
            result = self.comm(GET_AXIS_PAR, pobj.adr, **kwds)
            if result != rawvalue:
                raise HardwareError('result does not match %d != %d' % (result, rawvalue))
            value = result * scale
        return value

    def startModule(self, started_callback):
        # self._initialize = True

        # get encoder value from motor
        encoder = self.get('encoder') + self.zero

        self.fix_encoder(encoder)
        self.comm(SET_GLOB_PAR, 255, 1, bank=2)
        self._prevconn = self._iodev._conn
        self._initialize = False

        super().startModule(started_callback)

    def fix_encoder(self, encoder_from_hw):
        """fix encoder value

        :param encoder_from_hw: the encoder value read from the HW

        self.encoder is assumed to contain the last known (persistent) value
        if encoder has not changed modulo 360, adjust by a integer multiple of 360
        set status to error when encoder has changed be more thean encoder_tolerance
        """
        # calculate nearest, most probable value
        adjusted_encoder = encoder_from_hw + round((self.encoder - encoder_from_hw) / 360.) * 360
        if abs(self.encoder - adjusted_encoder) < self.encoder_tolerance:
            # encoder has not changed modulo 360
            self._pos_state = 'init'
            self._reset_to = encoder_from_hw
        else:
            self.log.error('saved encoder value (%.2f) does not match reading (%.2f)', self.encoder, encoder_from_hw)
            if adjusted_encoder != encoder_from_hw:
                self.log.info('take next closest encoder value (%.2f)' % adjusted_encoder)
            self._pos_state = 'need_reset'
            self.status = self.Status.ERROR, 'saved encoder value does not match reading'
        self.set('encoder', adjusted_encoder - self.zero, check=False)

    def fix_steppos(self, tolerance):
        """adjust steppos to encoder value"""
        if abs(self.encoder - self._reset_to) < tolerance and abs(self.steppos - self._reset_to) < tolerance:
            self._pos_state = None
            return False
        self.log.warning('adjust steppos (%.2f) to encoder (%.2f) and move to (%.2f)', self.steppos, self.encoder, self._reset_to)
        self.set('steppos', self.encoder - self.zero, check=False)
        self.comm(MOVE, 0, (self.encoder - self.zero) / ANGLE_SCALE)
        return True

    def read_value(self):
        encoder = self.read_encoder()
        steppos = self.read_steppos()

        initialized = self._initialize or self.comm(GET_GLOB_PAR, 255, bank=2)
        if not initialized or self._prevconn != self._iodev._conn:  # power loss or connection interrupt
            # get values from motor (may have been lost due to power down)
            # get persistent values
            writeDict = self.loadParameters()
            # self.encoder now contains the last known (persistent) value
            self.log.info('set to previous saved values %r', writeDict)
            for pname, value in writeDict.items():
                try:
                    getattr(self, 'write_' + pname)(value)
                except Exception as e:
                    self.log.warning('can not write %r to %r (%r)' % (value, pname, e))
            self.fix_encoder(encoder)
            self.fix_steppos(self.tolerance)
            self.comm(SET_GLOB_PAR, 255, 1, bank=2)  # set initialized flag
            self._prevconn = self._iodev._conn

        return encoder if abs(encoder - steppos) > self.tolerance else steppos

    def read_status(self):
        self.read_value()  # make sure encoder and steppos are fresh
        if not self._started:
            if self._pos_state == 'reset':
                self._try_count += 1
                # often, several attempts are needed, as the motor might move 1-2 full steps
                # and the encoder reading seems to need time
                if self._try_count % 5:
                    print(self.encoder, self.steppos, self._reset_to, self.comm(GET_AXIS_PAR, 0) * ANGLE_SCALE + self.zero)
                    return self.Status.BUSY, 'resetting'
                if self._try_count <= 50:
                    if not self.fix_steppos(ENCODER_RESOLUTION):
                        self._pos_state = 'reset'  # try anyway at least 3 times
                    return self.Status.BUSY, 'resetting'
                if self._try_count > 70:
                    self._pos_state = 'need_reset'
                    return self.Status.ERROR, 'reset failed (too many tries)'
                if self.fix_steppos(self.tolerance):
                    return self.Status.BUSY, 'resetting'
                return self.Status.IDLE, ''
            elif abs(self.encoder - self.steppos) > self.encoder_tolerance:
                self._pos_state = 'need_reset'
                if self.status[0] != self.Status.ERROR:
                    self.log.error('encoder (%.2f) does not match internal pos (%.2f)', self.encoder, self.steppos)
                    return self.Status.ERROR, 'encoder does not match internal pos'
            return self.status
        if ((abs(self.target - self.encoder) < self.tolerance or time.time() > self._started + 1)
                and (self.read_target_reached() or self.read_move_status())):
            diff = self.target - self.encoder
            if abs(diff) > self.tolerance:
                if (abs(self.target - self.steppos) < self.tolerance and
                        abs(self.encoder - self.steppos) < self.encoder_tolerance):
                    self._try_count += 1
                    if self._try_count < 3:
                        # often, two attempts are needed, as steppos and encoder might have been
                        # off by 1-2 full steps before moving
                        self.fix_steppos(self.tolerance)
                        # self.comm(MOVE, 0, (self.target - self.zero) / ANGLE_SCALE)
                        self.log.warning('try move again')
                        return self.Status.BUSY, 'try again'
                self.log.error('out of tolerance by %.3g', diff)
                self._started = 0
                return self.Status.ERROR, 'out of tolerance'
            else:
                self._started = 0
                return self.Status.IDLE, ''
        return self.Status.BUSY, 'moving'

    def write_target(self, target):
        self.read_value()  # make sure encoder and steppos are fresh
        if abs(target - self.encoder) > self.movelimit:
            raise BadValueError('can not move more than %s deg' % self.movelimit)
        diff = self.encoder - self.steppos
        if self._pos_state == 'need_reset':
            raise HardwareError('need reset (%s)' % self.status[1])
        if abs(diff) > self.tolerance or self._pos_state:
            if abs(diff) > self.encoder_tolerance and self._pos_state not in ('reset', 'init'):
                self._pos_state = 'need_reset'
                self.status = self.Status.ERROR, 'encoder does not match internal pos'
                raise HardwareError('need reset (encoder does not match internal pos)')
            self.fix_steppos(self.encoder_tolerance)
        self._started = time.time()
        self._try_count = 0
        self.log.info('move to %.1f', target)
        self.comm(MOVE, 0, (target - self.zero) / ANGLE_SCALE)
        self.status = self.Status.BUSY, 'changed target'
        return target

    def write_zero(self, value):
        diff = value - self.zero
        self.encoder += diff
        self.steppos += diff
        self.value += diff
        return value

    def read_encoder(self):
        return self.get('encoder') + self.zero

    def read_steppos(self):
        return self.get('steppos') + self.zero

    def read_encoder_tolerance(self):
        return self.get('encoder_tolerance')

    def write_encoder_tolerance(self, value):
        return self.set('encoder_tolerance', value)

    def read_target_reached(self):
        return self.get('target_reached')

    def read_speed(self):
        return self.get('speed')

    def write_speed(self, value):
        return self.set('speed', value)

    def read_minspeed(self):
        return self.get('minspeed')

    def write_minspeed(self, value):
        return self.set('minspeed', value)

    def read_currentspeed(self):
        return self.get('currentspeed')

    def read_acceleration(self):
        return self.get('acceleration')

    def write_acceleration(self, value):
        return self.set('acceleration', value)

    def read_maxcurrent(self):
        return self.get('maxcurrent')

    def write_maxcurrent(self, value):
        return self.set('maxcurrent', value)

    def read_standby_current(self):
        return self.get('standby_current')

    def write_standby_current(self, value):
        return self.set('standby_current', value)

    def read_free_wheeling(self):
        return self.get('free_wheeling')

    def write_free_wheeling(self, value):
        return self.set('free_wheeling', value)

    def read_move_status(self):
        return self.get('move_status')

    def read_error_bits(self):
        return self.get('error_bits')

    @Command(FloatRange())
    def set_zero(self, value):
        self.write_zero(value - self.read_value())

    def read_baudrate(self):
        return self.comm(GET_GLOB_PAR, 65)

    def write_baudrate(self, value):
        self.comm(SET_GLOB_PAR, 65, int(value))

    @Command()
    def reset(self):
        """set steppos to encoder value, if not within tolerance"""
        if self._started:
            raise IsBusyError('can not reset while moving')
        self.read_value()
        encoder = self.encoder
        for itry in range(5):
            if abs(self.encoder - self.steppos) > 0.4:
                print('adjust', self.encoder, self.steppos)
                self.set('steppos', self.encoder - self.zero, check=False)
                self.comm(MOVE, 0, (self.encoder - self.zero) / ANGLE_SCALE)
                time.sleep(0.1)
                self.read_value()
                diff = encoder - self.encoder
                if abs(diff) > 0.4:
                    print('move', diff)
                    self.comm(MOVE, 0, (self.steppos + diff - self.zero) / ANGLE_SCALE)
                    time.sleep(0.5)
            else:
                print('OK', self.encoder, self.steppos)
                self._pos_state = None
                self.status = self.Status.IDLE, 'ok'
                return
            self.read_value()
        print('FAILED')
        self._pos_state = 'reset_failed'
        self.status = self.Status.ERROR, 'reset failed'
        return

    @Command()
    def fix(self):
        """set steppos to encoder value, if not within tolerance"""
        self.read_value()
        if abs(self.encoder - self.steppos) < 0.4:
            self._pos_state = None
            self._back = None
            self.status = self.Status.IDLE, 'ok'
            return
        if self._back is None:
            self._back = self.encoder
            print('BACK', self._back)
        self.log.warning('adjust steppos (%.2f) to encoder (%.2f)', self.steppos, self.encoder)
        self.set('steppos', self.encoder - self.zero, check=False)
        self.comm(MOVE, 0, (self.encoder - self.zero) / ANGLE_SCALE)
        time.sleep(0.2)
        self.read_value()
        self.log.info('current steppos (%.2f) encoder (%.2f)', self.steppos, self.encoder)

    @Command()
    def back(self):
        """set steppos to encoder value, if not within tolerance"""
        if self._back is None:
            self.log.info('already back')
            return
        self.read_value()
        if abs(self.encoder - self._back) > 0.1:
            print('BACK', self._back)
            self.comm(MOVE, 0, (self.steppos + self.encoder - self._back - self.zero) / ANGLE_SCALE)
            time.sleep(0.1)
            self.set('steppos', self._back - self.zero, check=False)

    @Command()
    def stop(self):
        self.comm(MOTOR_STOP, 0)
        self.status = self.Status.IDLE, 'stopped'
        self._started = 0

    @Command(FloatRange())
    def step(self, value):
        self.comm(MOVE, 1, value / ANGLE_SCALE)

    @Command(IntRange(), result=IntRange())
    def get_axis_par(self, adr):
        return self.comm(GET_AXIS_PAR, adr)

    @Command((IntRange(), FloatRange()), result=IntRange())
    def set_axis_par(self, adr, value):
        return self.comm(SET_AXIS_PAR, adr, value)
