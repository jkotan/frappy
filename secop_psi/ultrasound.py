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
"""frappy support for ultrasound"""

import math
#import serial
import os
import time

import numpy as np

import iqplot
from adq_mr import Adq
from secop.core import Attached, BoolType, Done, FloatRange, HasIodev, \
    IntRange, Module, Parameter, Readable, StringIO, StringType
from secop.properties import Property


def fname_from_time(t, extension):
    tm = time.localtime(t)
    dirname = os.path.join('..', 'data', time.strftime("%Y-%m-%d_%H", tm))
    filename = time.strftime("%Y-%m-%d_%H-%M-%S_", tm)
    filename = filename + ("%.1f" % t)[-1]
    if not os.path.isdir(dirname):
        os.makedirs(dirname)
    return os.path.join(dirname, filename)


class Roi(Readable):
    main = Attached()

    value = Parameter('amplitude', FloatRange(), default=0)
    phase = Parameter('phase', FloatRange(unit='deg'), default=0)
    i = Parameter('in phase', FloatRange(), default=0)
    q = Parameter('out of phase', FloatRange(), default=0)
    time = Parameter('start time', FloatRange(unit='nsec'), readonly=False)
    size = Parameter('interval (symmetric around time)', FloatRange(unit='nsec'), readonly=False)
    enable = Parameter('calculate this roi', BoolType(), readonly=False, default=True)
    #status = Parameter(export=False)
    pollinterval = Parameter(export=False)

    interval = (0,0)

    def initModule(self):
        self._main.register_roi(self)
        self.calc_interval()

    def calc_interval(self):
        self.interval = (self.time - 0.5 * self.size, self.time + 0.5 * self.size)

    def write_time(self, value):
        self.time = value
        self.calc_interval()
        return Done

    def write_size(self, value):
        self.size = value
        self.calc_interval()
        return Done


class Pars(Module):
    description = 'relevant parameters from SEA'

    timestamp = Parameter('unix timestamp', StringType(), default='0', readonly=False)
    temperature = Parameter('T', FloatRange(unit='K'), default=0, readonly=False)
    mf = Parameter('field', FloatRange(unit='T'), default=0, readonly=False)
    sr = Parameter('rotaion angle', FloatRange(unit='deg'), default=0, readonly=False)


class FreqStringIO(StringIO):
    end_of_line = '\r'


class Frequency(HasIodev, Readable):
    pars = Attached()
    sr = Property('samples per record', datatype=IntRange(), default=16384)
    maxy = Property('plot y scale', datatype=FloatRange(), default=0.5)

    value = Parameter('frequency@I,q', datatype=FloatRange(unit='Hz'), default=0)
    basefreq = Parameter('base frequency', FloatRange(unit='Hz'), readonly=False)
    nr = Parameter('number of records', datatype=IntRange(1,10000), default=500)
    freq = Parameter('target frequency', FloatRange(unit='Hz'), readonly=False, poll=True)
    amp = Parameter('amplitude', FloatRange(unit='dBm'), readonly=False, poll=True)
    control = Parameter('control loop on?', BoolType(), readonly=False, default=True)
    time = Parameter('pulse start time', FloatRange(unit='nsec'),
                      readonly=False)
    size = Parameter('pulse length (starting from time)', FloatRange(unit='nsec'),
                    readonly=False)
    pulselen = Parameter('adjusted pulse length (integer number of periods)', FloatRange(unit='nsec'), default=1)
    maxstep = Parameter('max frequency step', FloatRange(unit='Hz'), readonly=False,
                      default=10000)
    minstep = Parameter('min frequency step for slope calculation', FloatRange(unit='Hz'),
                        readonly=False, default=4000)
    slope = Parameter('inphase/frequency slope', FloatRange(), readonly=False,
                      default=1e6)
    plot = Parameter('create plot images', BoolType(), readonly=False, default=True)
    save = Parameter('save data', BoolType(), readonly=False, default=True)
    pollinterval = Parameter(datatype=FloatRange(0,120))

    iodevClass = FreqStringIO

    lastfreq = None
    old = None
    starttime = None
    interval = (0,0)

    def earlyInit(self):
        #assert self.iodev.startswith('serial:')
        #self._iodev = serial.Serial(self.iodev[7:])
        self.adq = Adq(self.nr, self.sr)
        self.roilist = []
        self.write_nr(self.nr)
        self.skipctrl = 0
        self.plotter = iqplot.Plot(self.maxy)
        self.calc_interval()

    def calc_interval(self):
        self.interval = (self.time, self.time + self.size)

    def write_time(self, value):
        self.time = value
        self.calc_interval()
        return Done

    def write_size(self, value):
        self.size = value
        self.calc_interval()
        return Done

    def write_nr(self, value):
        # self.pollinterval = value * 0.0001
        return value

    def register_roi(self, roi):
        self.roilist.append(roi)

    def set_freq(self):
        freq = self.freq + self.basefreq
        self.sendRecv('FREQ %.15g;FREQ?' % freq)
        #self._iodev.readline().decode('ascii')
        return freq

    def write_amp(self, amp):
        reply = self.sendRecv('AMPR %g;AMPR?' % amp)
        return float(reply)

    def read_amp(self):
        reply = self.sendRecv('AMPR?')
        return float(reply)

    def write_freq(self, value):
        self.skipctrl = 2  # suppress control for the 2 next steps
        return value

    def read_freq(self):
        """used as main polling loop body"""
        if self.lastfreq is None:
            self.lastfreq = self.set_freq()
            self.adq.start()
        if self.starttime is None:
            self.starttime = time.time()
        times = []
        times.append(('init', time.time()))
        seadata = {p: float(getattr(self._pars, p)) for p in self._pars.parameters}
        data = self.adq.getdata()  # this will wait, if not yet finished
        times.append(('wait',time.time()))
        freq = self.lastfreq  # data was acquired at this freq
        seadata['frequency'] = freq
        self.lastfreq = self.set_freq()
        times.append(('setf',time.time()))
        self.adq.start()  # start next acq
        times.append(('start',time.time()))
        roilist = [r for r in self.roilist if r.enable]

        gates = self.adq.gates_and_curves(data, freq, self.interval,
                                          [r.interval for r in roilist])
        if self.save:
            tdata, idata, qdata, pdata = self.adq.curves
            seadata['timestep'] = tdata[1] - tdata[0]
            iqdata = np.array((idata, qdata, pdata), dtype='f4')
            ts = seadata['timestamp']
            if ts:
                filename = fname_from_time(ts, '.npz')
                seanp = np.array(list(seadata.items()), dtype=[('name', 'U16'), ('value', 'f8')])
                np.savez(filename, seadata=seanp, iqdata=iqdata)
                # can be load back via
                # content = np.load(filename)
                # content['seadata'], content['iqdata']
        self.pulselen = self.adq.pulselen
        times.append(('ana',time.time()))
        if self.plot:
            # get reduced interval from adq.sinW
            pulseint = (self.interval[0], self.interval[0] + self.pulselen)
            try:
                self.plotter.plot(
                    self.adq.curves,
                    rois=[pulseint] + [r.interval for r in roilist],
                    average=([r.time for r in roilist],
                             [r.i for r in roilist],
                             [r.q for r in roilist]))
            except Exception as e:
                self.log.warning('can not plot %r' % e)
        else:
            self.plotter.close()
        now = time.time()
        times.append(('plot',now))
        # print(' '.join('%s %5.3f' % (label, t - self.starttime) for label, t in times))
        self.starttime = now
        self.value = freq - self.basefreq
        for i, roi in enumerate(roilist):
            roi.i = a = gates[i][0]
            roi.q = b = gates[i][1]
            roi.value = math.sqrt(a ** 2 + b ** 2)
            roi.phase = math.atan2(a,b) * 180 / math.pi
        inphase = self.roilist[0].i
        if self.control:
            newfreq = freq + inphase * self.slope - self.basefreq
            # step = sorted((-self.maxstep, inphase * self.slope, self.maxstep))[1]
        if self.old:
            fdif = freq - self.old[0]
            idif = inphase - self.old[1]
            if abs(fdif) >= self.minstep:
                self.slope = - fdif / idif
        else:
            fdif = 0
            idif = 0
            newfreq = freq + self.minstep
        self.old = (freq, inphase)
        if self.skipctrl > 0:  # do no control for some time after changing frequency
            self.skipctrl -= 1
        elif self.control:
            self.freq = sorted((self.freq - self.maxstep, newfreq, self.freq + self.maxstep))[1]
        return Done
