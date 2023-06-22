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
"""use transducer and motor to adjust force"""

import time
import math
from secop.core import Drivable, Parameter, FloatRange, Done, \
    Attached, Command, PersistentMixin, PersistentParam, BoolType
from secop.errors import BadValueError


class Uniax(PersistentMixin, Drivable):
    value = Parameter(unit='N')
    motor = Attached()
    transducer = Attached()
    limit = Parameter('abs limit of force', FloatRange(0, 150, unit='N'), readonly=False, default=150)
    tolerance = Parameter('force tolerance', FloatRange(0, 10, unit='N'), readonly=False, default=0.1)
    slope = PersistentParam('spring constant', FloatRange(unit='deg/N'), readonly=False,
                            default=0.5, persistent='auto')
    pid_i = PersistentParam('integral', FloatRange(), readonly=False, default=0.5, persistent='auto')
    filter_interval = Parameter('filter time', FloatRange(0, 60, unit='s'), readonly=False, default=1)
    current_step = Parameter('', FloatRange(unit='deg'), default=0)
    force_offset = PersistentParam('transducer offset', FloatRange(unit='N'), readonly=False, default=0,
                                   initwrite=True, persistent='auto')
    hysteresis = PersistentParam('force hysteresis', FloatRange(0, 150, unit='N'), readonly=False, default=5,
                                 persistent='auto')
    adjusting = Parameter('', BoolType(), readonly=False, default=False, initwrite=True)
    adjusting_current = PersistentParam('current when adjusting force', FloatRange(0, 2.8, unit='A'), readonly=False,
                                        default=0.5, persistent='auto')
    safe_step = PersistentParam('max. motor step when adjusting force', FloatRange(0, unit='deg'), readonly=False,
                                default=5, persistent='auto')
    safe_current = PersistentParam('current when moving far', FloatRange(0, 2.8, unit='A'), readonly=False,
                                   default=0.2, persistent='auto')
    low_pos = Parameter('max. position for positive forces', FloatRange(unit='deg'), readonly=False, needscfg=False)
    high_pos = Parameter('min. position for negative forces', FloatRange(unit='deg'), readonly=False, needscfg=False)
    pollinterval = 0.1
    fast_pollfactor = 1

    _mot_target = None  # for detecting manual motor manipulations
    _filter_start = 0
    _cnt = 0
    _sum = 0
    _cnt_rderr = 0
    _cnt_wrerr = 0
    _action = None
    _last_force = 0
    _expected_step = 1
    _fail_cnt = 0
    _in_cnt = 0
    _init_action = False
    _zero_pos_tol = None
    _find_target = 0

    def earlyInit(self):
        self._zero_pos_tol = {}
        self._action = self.idle

    def drive_relative(self, step, ntry=3):
        """drive relative, try 3 times"""
        mot = self._motor
        mot.read_value()  # make sure motor value is fresh
        if self.adjusting and abs(step) > self.safe_step:
            step = math.copysign(self.safe_step, step)
        self.current_step = step
        for _ in range(ntry):
            try:
                self._mot_target = self._motor.write_target(mot.value + step)
                self._cnt_wrerr = max(0, self._cnt_wrerr - 1)
                return True
            except Exception as e:
                self.log.warning('drive error %s', e)
                self._cnt_wrerr += 1
                if self._cnt_wrerr > 5:
                    raise
                self.log.warning('motor reset')
                self._motor.reset()
        return False

    def reset_filter(self, now=0.0):
        self._sum = self._cnt = 0
        self._filter_start = now or time.time()

    def motor_busy(self):
        mot = self._motor
        if mot.isBusy():
            if mot.target != self._mot_target:
                self.action = self.idle
            return True
        return False

    def next_action(self, action):
        """call next action

        :param action: function to be called next time
        :param do_now: do next action in the same cycle
        """
        self._action = action
        self._init_action = True
        self.log.info('action %r', action.__name__)

    def init_action(self):
        """return true when called the first time after next_action"""
        if self._init_action:
            self._init_action = False
            return True
        return False

    def zero_pos(self, value,):
        """get high_pos or low_pos, depending on sign of value

        :param force: when not 0, return an estimate for a good starting position
        """

        name = 'high_pos' if value > 0 else 'low_pos'
        if name not in self._zero_pos_tol:
            return None
        return getattr(self, name)

    def set_zero_pos(self, force, pos):
        """set zero position high_pos or low_pos, depending on sign and value of force

        :param force: the force used for calculating zero_pos
        :param pos: the position used for calculating zero_pos
        """
        name = 'high_pos' if force > 0 else 'low_pos'
        if pos is None:
            self._zero_pos_tol.pop(name, None)
            return None
        pos -= force * self.slope
        tol = (abs(force) - self.hysteresis) * self.slope * 0.2
        if name in self._zero_pos_tol:
            old = self.zero_pos(force)
            if abs(old - pos) < self._zero_pos_tol[name] + tol:
                return
        self._zero_pos_tol[name] = tol
        self.log.info('set %s = %.1f +- %.1f (@%g N)' % (name, pos, tol, force))
        setattr(self, name, pos)
        return pos

    def find(self, force, target):
        """find active (engaged) range"""
        sign = math.copysign(1, target)
        if force * sign > self.hysteresis or force * sign > target * sign:
            if self.motor_busy():
                self.log.info('motor stopped - substantial force detected: %g', force)
                self._motor.stop()
            elif self.init_action():
                self.next_action(self.adjust)
                return
            if abs(force) > self.hysteresis:
                self.set_zero_pos(force, self._motor.read_value())
            self.next_action(self.adjust)
            return
        if force * sign < -self.hysteresis:
            self._previous_force = force
            self.next_action(self.free)
            return
        if self.motor_busy():
            if sign * self._find_target < 0:  # target sign changed
                self._motor.stop()
                self.next_action(self.find)  # restart find
                return
        else:
            self._find_target = target
            zero_pos = self.zero_pos(target)
            side_name = 'positive' if target > 0 else 'negative'
            if not self.init_action():
                if abs(self._motor.target - self._motor.value) > self._motor.tolerance:
                    # no success on last find try, try short and strong step
                    self.write_adjusting(True)
                    self.log.info('one step to %g', self._motor.value + self.safe_step)
                    self.drive_relative(sign * self.safe_step)
                    return
            if zero_pos is not None:
                self.status = 'BUSY', 'change to %s side' % side_name
                zero_pos += sign * (self.hysteresis * self.slope - self._motor.tolerance)
                if (self._motor.value - zero_pos) * sign < -self._motor.tolerance:
                    self.write_adjusting(False)
                    self.log.info('change side to %g', zero_pos)
                    self.drive_relative(zero_pos - self._motor.value)
                    return
                # we are already at or beyond zero_pos
                self.next_action(self.adjust)
                return
            self.write_adjusting(False)
            self.status = 'BUSY', 'find %s side' % side_name
            self.log.info('one turn to %g', self._motor.value + sign * 360)
            self.drive_relative(sign * 360)

    def free(self, force, target):
        """free from high force at other end"""
        if self.motor_busy():
            return
        if abs(force) > abs(self._previous_force) + self.tolerance:
            self.stop()
            self.status = 'ERROR', 'force increase while freeing'
            self.log.error(self.status[1])
            return
        if abs(force) < self.hysteresis:
            self.next_action(self.find)
            return
        if self.init_action():
            self._free_way = 0
            self.log.info('free from high force %g', force)
            self.write_adjusting(True)
        sign = math.copysign(1, target)
        if self._free_way > (abs(self._previous_force) + self.hysteresis) * self.slope:
            self.stop()
            self.status = 'ERROR', 'freeing failed'
            self.log.error(self.status[1])
            return
        self._free_way += self.safe_step
        self.drive_relative(sign * self.safe_step)

    def within_tolerance(self, force, target):
        """within tolerance"""
        if self.motor_busy():
            return
        if abs(target - force) > self.tolerance:
            self.next_action(self.adjust)
        elif self.init_action():
            self.status = 'IDLE', 'within tolerance'

    def adjust(self, force, target):
        """adjust force"""
        if self.motor_busy():
            return
        if abs(target - force) < self.tolerance:
            self._in_cnt += 1
            if self._in_cnt >= 3:
                self.next_action(self.within_tolerance)
                return
        else:
            self._in_cnt = 0
        if self.init_action():
            self._fail_cnt = 0
            self.write_adjusting(True)
            self.status = 'BUSY', 'adjusting force'
        elif not self._filtered:
            return
        else:
            force_step = force - self._last_force
            if self._expected_step:
                # compare detected / expected step
                q = force_step / self._expected_step
                if q < 0.1:
                    self._fail_cnt += 1
                elif q > 0.5:
                    self._fail_cnt = max(0, self._fail_cnt - 1)
                if self._fail_cnt >= 10:
                    if force < self.hysteresis:
                        self.log.warning('adjusting failed - try to find zero pos')
                        self.set_zero_pos(target, None)
                        self.next_action(self.find)
                    elif self._fail_cnt > 20:
                        self.stop()
                        self.status = 'ERROR', 'force seems not to change substantially'
                        self.log.error(self.status[1])
                        return
        self._last_force = force
        force_step = (target - force) * self.pid_i
        if abs(target - force) < self.tolerance * 0.5:
            self._expected_step = 0
            return
        self._expected_step = force_step
        step = force_step * self.slope
        self.drive_relative(step)

    def idle(self, *args):
        if self.init_action():
            self.write_adjusting(False)
            if self.status[0] == 'BUSY':
                self.status = 'IDLE', 'stopped'

    def read_value(self):
        try:
            force = self._transducer.read_value()
            self._cnt_rderr = max(0, self._cnt_rderr - 1)
        except Exception as e:
            self._cnt_rderr += 1
            if self._cnt_rderr > 10:
                self.stop()
                self.status = 'ERROR', 'too many read errors: %s' % e
                self.log.error(self.status[1])
            return Done

        now = time.time()
        if self.motor_busy():
            # do not filter while driving
            self.value = force
            self.reset_filter()
            self._filtered = False
        else:
            self._sum += force
            self._cnt += 1
            if now < self._filter_start + self.filter_interval:
                return Done
            force = self._sum / self._cnt
            self.value = force
            self.reset_filter(now)
            self._filtered = True
        if abs(force) > self.limit + self.hysteresis:
            self.status = 'ERROR', 'above max limit'
            self.log.error(self.status[1])
            return Done
        if self.zero_pos(force) is None and abs(force) > self.hysteresis and self._filtered:
            self.set_zero_pos(force, self._motor.read_value())
        self._action(self.value, self.target)
        return Done

    def write_target(self, target):
        if abs(target) > self.limit:
            raise BadValueError('force above limit')
        if abs(target - self.value) <= self.tolerance:
            if self.isBusy():
                self.stop()
                self.next_action(self.within_tolerance)
            else:
                self.status = 'IDLE', 'already at target'
                self.next_action(self.within_tolerance)
                return target
        self.log.info('new target %g', target)
        self._cnt_rderr = 0
        self._cnt_wrerr = 0
        self.status = 'BUSY', 'changed target'
        if self.value * math.copysign(1, target) > self.hysteresis:
            self.next_action(self.adjust)
        else:
            self.next_action(self.find)
        return target

    @Command()
    def stop(self):
        self._action = self.idle
        if self._motor.isBusy():
            self.log.info('stop motor')
            self._motor.stop()
        self.next_action(self.idle)

    def write_force_offset(self, value):
        self.force_offset = value
        self._transducer.write_offset(value)
        return Done

    def write_adjusting(self, value):
        mot = self._motor
        if value:
            mot_current = self.adjusting_current
            mot.write_move_limit(self.safe_step)
        else:
            mot_current = self.safe_current
            mot.write_safe_current(mot_current)
        if abs(mot_current - mot.maxcurrent) > 0.01:  # resolution of current: 2.8 / 250
            self.log.info('motor current %g', mot_current)
            mot.write_maxcurrent(mot_current)
        return value
