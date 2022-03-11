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
#   Markus Zolliker <markus.zolliker@psi.ch>
#
# *****************************************************************************
"""Define base classes for real Modules implemented in the server"""


import time
from queue import Queue, Empty
from collections import OrderedDict
from functools import wraps

from secop.datatypes import ArrayOf, BoolType, EnumType, FloatRange, \
    IntRange, StatusType, StringType, TextType, TupleOf, DiscouragedConversion
from secop.errors import BadValueError, ConfigError, \
    ProgrammingError, SECoPError, SilentError, secop_error
from secop.lib import formatException, mkthread, UniqueObject, generalConfig
from secop.lib.enum import Enum
from secop.params import Accessible, Command, Parameter
from secop.properties import HasProperties, Property
from secop.logging import RemoteLogHandler, HasComlog


generalConfig.defaults['disable_value_range_check'] = False  # check for problematic value range by default

Done = UniqueObject('Done')
"""a special return value for a read/write function

indicating that the setter is triggered already"""


class HasAccessibles(HasProperties):
    """base class of Module

    joining the class's properties, parameters and commands dicts with
    those of base classes.
    wrap read_*/write_* methods
    (so the dispatcher will get notified of changed values)
    """
    @classmethod
    def __init_subclass__(cls):  # pylint: disable=too-many-branches
        super().__init_subclass__()
        # merge accessibles from all sub-classes, treat overrides
        # for now, allow to use also the old syntax (parameters/commands dict)
        accessibles = OrderedDict()  # dict of accessibles
        merged_properties = {}  # dict of dict of merged properties
        new_names = []  # list of names of new accessibles
        override_values = {}  # bare values overriding a parameter and methods overriding a command

        for base in reversed(cls.__mro__):
            for key, value in base.__dict__.items():
                if isinstance(value, Accessible):
                    value.updateProperties(merged_properties.setdefault(key, {}))
                    if base == cls and key not in accessibles:
                        new_names.append(key)
                    accessibles[key] = value
                    override_values.pop(key, None)
                elif key in accessibles:
                    override_values[key] = value
        for aname, aobj in list(accessibles.items()):
            if aname in override_values:
                aobj = aobj.copy()
                value = override_values[aname]
                if value is None:
                    accessibles.pop(aname)
                    continue
                aobj.merge(merged_properties[aname])
                aobj.override(value)
                # replace the bare value by the created accessible
                setattr(cls, aname, aobj)
            else:
                aobj.merge(merged_properties[aname])
            accessibles[aname] = aobj

        # rebuild order: (1) inherited items, (2) items from paramOrder, (3) new accessibles
        # move (2) to the end
        paramOrder = cls.__dict__.get('paramOrder', ())
        for aname in paramOrder:
            if aname in accessibles:
                accessibles.move_to_end(aname)
                # ignore unknown names
        # move (3) to the end
        for aname in new_names:
            if aname not in paramOrder:
                accessibles.move_to_end(aname)
        # note: for python < 3.6 the order of inherited items is not ensured between
        # declarations within the same class
        cls.accessibles = accessibles

        # Correct naming of EnumTypes
        # moved to Parameter.__set_name__

        # check validity of Parameter entries
        for pname, pobj in accessibles.items():
            # XXX: create getters for the units of params ??

            # wrap of reading/writing funcs
            if not isinstance(pobj, Parameter):
                # nothing to do for Commands
                continue

            rfunc = getattr(cls, 'read_' + pname, None)
            # TODO: remove handler stuff here
            rfunc_handler = pobj.handler.get_read_func(cls, pname) if pobj.handler else None
            wrapped = getattr(rfunc, 'wrapped', False)  # meaning: wrapped or auto generated
            if rfunc_handler:
                if 'read_' + pname in cls.__dict__:
                    if pname in cls.__dict__:
                        raise ProgrammingError("parameter '%s' can not have a handler "
                                               "and read_%s" % (pname, pname))
                    # read_<pname> overwrites inherited handler
                else:
                    rfunc = rfunc_handler
                wrapped = False

            # create wrapper except when read function is already wrapped
            if not wrapped:

                if rfunc:

                    @wraps(rfunc)  # handles __wrapped__ and __doc__
                    def new_rfunc(self, pname=pname, rfunc=rfunc):
                        try:
                            value = rfunc(self)
                            self.log.debug("read_%s returned %r", pname, value)
                        except Exception as e:
                            self.log.debug("read_%s failed with %r", pname, e)
                            raise
                        if value is Done:
                            return getattr(self, pname)
                        setattr(self, pname, value)  # important! trigger the setter
                        return value

                    new_rfunc.poll = getattr(rfunc, 'poll', True)
                else:

                    def new_rfunc(self, pname=pname):
                        return getattr(self, pname)

                    new_rfunc.poll = False
                    new_rfunc.__doc__ = 'auto generated read method for ' + pname

                new_rfunc.wrapped = True  # indicate to subclasses that no more wrapping is needed
                setattr(cls, 'read_' + pname, new_rfunc)

            if not pobj.readonly:
                wfunc = getattr(cls, 'write_' + pname, None)
                wrapped = getattr(wfunc, 'wrapped', False)  # meaning: wrapped or auto generated
                if (wfunc is None or wrapped) and pobj.handler:
                    # ignore the handler, if a write function is present
                    # TODO: remove handler stuff here
                    wfunc = pobj.handler.get_write_func(pname)
                    wrapped = False

                # create wrapper except when write function is already wrapped
                if not wrapped:

                    if wfunc:

                        @wraps(wfunc)  # handles __wrapped__ and __doc__
                        def new_wfunc(self, value, pname=pname, wfunc=wfunc):
                            pobj = self.accessibles[pname]
                            self.log.debug('validate %r for %r', value, pname)
                            # we do not need to handle errors here, we do not
                            # want to make a parameter invalid, when a write failed
                            new_value = pobj.datatype(value)
                            new_value = wfunc(self, new_value)
                            self.log.debug('write_%s(%r) returned %r', pname, value, new_value)
                            if new_value is Done:
                                # setattr(self, pname, getattr(self, pname))
                                return getattr(self, pname)
                            setattr(self, pname, new_value)  # important! trigger the setter
                            return new_value
                    else:

                        def new_wfunc(self, value, pname=pname):
                            setattr(self, pname, value)
                            return value

                        new_wfunc.__doc__ = 'auto generated write method for ' + pname

                    new_wfunc.wrapped = True  # indicate to subclasses that no more wrapping is needed
                    setattr(cls, 'write_' + pname, new_wfunc)

        # check for programming errors
        for attrname in dir(cls):
            prefix, _, pname = attrname.partition('_')
            if not pname:
                continue
            if prefix == 'do':
                raise ProgrammingError('%r: old style command %r not supported anymore'
                                       % (cls.__name__, attrname))
            if prefix in ('read', 'write') and not getattr(getattr(cls, attrname), 'wrapped', False):
                raise ProgrammingError('%s.%s defined, but %r is no parameter'
                                       % (cls.__name__, attrname, pname))

        res = {}
        # collect info about properties
        for pn, pv in cls.propertyDict.items():
            if pv.settable:
                res[pn] = pv
        # collect info about parameters and their properties
        for param, pobj in cls.accessibles.items():
            res[param] = {}
            for pn, pv in pobj.getProperties().items():
                if pv.settable:
                    res[param][pn] = pv
        cls.configurables = res


class Module(HasAccessibles):
    """basic module

    all SECoP modules derive from this.

    :param name: the modules name
    :param logger: a logger instance
    :param cfgdict: the dict from this modules section in the config file
    :param srv: the server instance

    Notes:

    - the programmer normally should not need to reimplement :meth:`__init__`
    - within modules, parameters should only be addressed as ``self.<pname>``,
      i.e. ``self.value``, ``self.target`` etc...

      - these are accessing the cached version.
      - they can also be written to, generating an async update

    - if you want to 'update from the hardware', call ``self.read_<pname>()`` instead

      - the return value of this method will be used as the new cached value and
        be an async update sent automatically.

    - if you want to 'update the hardware' call ``self.write_<pname>(<new value>)``.

      - The return value of this method will also update the cache.

    """
    # static properties, definitions in derived classes should overwrite earlier ones.
    # note: properties don't change after startup and are usually filled
    #       with data from a cfg file...
    # note: only the properties predefined here are allowed to be set in the cfg file
    export = Property('flag if this module is to be exported', BoolType(), default=True, export=False)
    group = Property('optional group the module belongs to', StringType(), default='', extname='group')
    description = Property('description of the module', TextType(), extname='description', mandatory=True)
    meaning = Property('optional meaning indicator', TupleOf(StringType(), IntRange(0, 50)),
                       default=('', 0), extname='meaning')
    visibility = Property('optional visibility hint', EnumType('visibility', user=1, advanced=2, expert=3),
                          default='user', extname='visibility')
    implementation = Property('internal name of the implementation class of the module', StringType(),
                              extname='implementation')
    interface_classes = Property('offical highest interface-class of the module', ArrayOf(StringType()),
                                 extname='interface_classes')
    pollinterval = Property('poll interval for parameters handled by doPoll', FloatRange(0.1, 120), default=5)
    slowinterval = Property('poll interval for other parameters', FloatRange(0.1, 120), default=15)
    enablePoll = True

    # properties, parameters and commands are auto-merged upon subclassing
    parameters = {}
    commands = {}

    # reference to the dispatcher (used for sending async updates)
    DISPATCHER = None
    attachedModules = None

    def __init__(self, name, logger, cfgdict, srv):
        # remember the dispatcher object (for the async callbacks)
        self.DISPATCHER = srv.dispatcher
        self.omit_unchanged_within = getattr(self.DISPATCHER, 'omit_unchanged_within', 0.1)
        self.log = logger
        self.name = name
        self.valueCallbacks = {}
        self.errorCallbacks = {}
        self.earlyInitDone = False
        self.initModuleDone = False
        self.startModuleDone = False
        self.remoteLogHandler = None
        self.changePollinterval = Queue()  # used for waiting between polls and transmit info to the thread
        errors = []

        # handle module properties
        # 1) make local copies of properties
        super().__init__()

        # 2) check and apply properties specified in cfgdict as
        # '<propertyname> = <propertyvalue>'
        for key in self.propertyDict:
            value = cfgdict.pop(key, None)
            if value is None:
                # legacy cfg: specified as '.<propertyname> = <propertyvalue>'
                value = cfgdict.pop('.' + key, None)
            if value is not None:
                try:
                    self.setProperty(key, value)
                except BadValueError:
                    errors.append('%s: value %r does not match %r!' %
                                  (key, value, self.propertyDict[key].datatype))

        # 3) set automatic properties
        mycls = self.__class__
        myclassname = '%s.%s' % (mycls.__module__, mycls.__name__)
        self.implementation = myclassname
        # list of all 'secop' modules
        # self.interface_classes = [
        #    b.__name__ for b in mycls.__mro__ if b.__module__.startswith('secop.modules')]
        # list of only the 'highest' secop module class
        self.interface_classes = [
            b.__name__ for b in mycls.__mro__ if b.__module__.startswith('secop.modules')][0:1]

        # handle Features
        # XXX: todo

        # handle accessibles
        # 1) make local copies of parameter objects
        #    they need to be individual per instance since we use them also
        #    to cache the current value + qualifiers...
        accessibles = {}
        # conversion from exported names to internal attribute names
        accessiblename2attr = {}
        for aname, aobj in self.accessibles.items():
            # make a copy of the Parameter/Command object
            aobj = aobj.copy()
            if not self.export:  # do not export parameters of a module not exported
                aobj.export = False
            if aobj.export:
                accessiblename2attr[aobj.export] = aname
            accessibles[aname] = aobj
        # do not re-use self.accessibles as this is the same for all instances
        self.accessibles = accessibles
        self.accessiblename2attr = accessiblename2attr
        # provide properties to 'filter' out the parameters/commands
        self.parameters = {k: v for k, v in accessibles.items() if isinstance(v, Parameter)}
        self.commands = {k: v for k, v in accessibles.items() if isinstance(v, Command)}

        # 2) check and apply parameter_properties
        #    specified as '<paramname>.<propertyname> = <propertyvalue>'
        #    this may also be done on commands: e.g. 'stop.visibility = advanced'
        for k, v in list(cfgdict.items()):  # keep list() as dict may change during iter
            if '.' in k[1:]:
                aname, propname = k.split('.', 1)
                propvalue = cfgdict.pop(k)
                aobj = self.accessibles.get(aname, None)
                if aobj:
                    try:
                        aobj.setProperty(propname, propvalue)
                    except KeyError:
                        errors.append("'%s.%s' does not exist" %
                                      (aname, propname))
                    except BadValueError as e:
                        errors.append('%s.%s: %s' %
                                      (aname, propname, str(e)))
                else:
                    errors.append('%r not found' % aname)

        # 3) check config for problems:
        #    only accept remaining config items specified in parameters
        bad = [k for k in cfgdict if k not in self.parameters]
        if bad:
            errors.append(
                '%s does not exist (use one of %s)' %
                (', '.join(bad), ', '.join(list(self.parameters) +
                                                      list(self.propertyDict))))

        # 4) complain if a Parameter entry has no default value and
        #    is not specified in cfgdict and deal with parameters to be written.
        self.writeDict = {}  # values of parameters to be written
        for pname, pobj in self.parameters.items():
            self.valueCallbacks[pname] = []
            self.errorCallbacks[pname] = []

            if not pobj.hasDatatype():
                errors.append('%s needs a datatype' % pname)
                continue

            if pname in cfgdict:
                if not pobj.readonly and pobj.initwrite is not False:
                    # parameters given in cfgdict have to call write_<pname>
                    # TODO: not sure about readonly (why not a parameter which can only be written from config?)
                    try:
                        pobj.value = pobj.datatype(cfgdict[pname])
                        self.writeDict[pname] = pobj.value
                    except BadValueError as e:
                        errors.append('%s: %s' % (pname, e))
            else:
                if pobj.default is None:
                    if pobj.needscfg:
                        errors.append('%r has no default '
                                      'value and was not given in config!' % pname)
                    # we do not want to call the setter for this parameter for now,
                    # this should happen on the first read
                    pobj.readerror = ConfigError('parameter %r not initialized' % pname)
                    # above error will be triggered on activate after startup,
                    # when not all hardware parameters are read because of startup timeout
                    pobj.value = pobj.datatype(pobj.datatype.default)
                else:
                    try:
                        value = pobj.datatype(pobj.default)
                    except BadValueError as e:
                        # this should not happen, as the default is already checked in Parameter
                        raise ProgrammingError('bad default for %s:%s: %s' % (name, pname, e)) from None
                    if pobj.initwrite and not pobj.readonly:
                        # we will need to call write_<pname>
                        # if this is not desired, the default must not be given
                        # TODO: not sure about readonly (why not a parameter which can only be written from config?)
                        pobj.value = value
                        self.writeDict[pname] = value
                    else:
                        cfgdict[pname] = value

        # 5) 'apply' config:
        #    pass values through the datatypes and store as attributes
        for k, v in list(cfgdict.items()):
            try:
                # this checks also for the proper datatype
                # note: this will NOT call write_* methods!
                if k in self.parameters or k in self.propertyDict:
                    setattr(self, k, v)
                    cfgdict.pop(k)
            except (ValueError, TypeError) as e:
                # self.log.exception(formatExtendedStack())
                errors.append('parameter %s: %s' % (k, e))

        # ensure consistency
        for aobj in self.accessibles.values():
            aobj.finish()

        # Modify units AFTER applying the cfgdict
        for pname, pobj in self.parameters.items():
            dt = pobj.datatype
            if '$' in dt.unit:
                dt.setProperty('unit', dt.unit.replace('$', self.parameters['value'].datatype.unit))

        # 6) check complete configuration of * properties
        if not errors:
            try:
                self.checkProperties()
            except ConfigError as e:
                errors.append(str(e))
            for aname, aobj in self.accessibles.items():
                try:
                    aobj.checkProperties()
                except (ConfigError, ProgrammingError) as e:
                    errors.append('%s: %s' % (aname, e))
        if errors:
            raise ConfigError(errors)

    # helper cfg-editor
    def __iter__(self):
        return self.accessibles.__iter__()

    def __getitem__(self, item):
        return self.accessibles.__getitem__(item)

    def announceUpdate(self, pname, value=None, err=None, timestamp=None):
        """announce a changed value or readerror"""

        # TODO: remove readerror 'property' and replace value with exception
        pobj = self.parameters[pname]
        timestamp = timestamp or time.time()
        changed = pobj.value != value
        try:
            # store the value even in case of error
            pobj.value = pobj.datatype(value)
        except Exception as e:
            if isinstance(e, DiscouragedConversion):
                if DiscouragedConversion.log_message:
                    self.log.error(str(e))
                    self.log.error('you may disable this behaviour by running the server with --relaxed')
                    DiscouragedConversion.log_message = False
            if not err:  # do not overwrite given error
                err = e
        if err:
            err = secop_error(err)
            if str(err) == str(pobj.readerror):
                return  # do call updates for repeated errors
        elif not changed and timestamp < (pobj.timestamp or 0) + self.omit_unchanged_within:
            # no change within short time -> omit
            return
        pobj.timestamp = timestamp or time.time()
        pobj.readerror = err
        if pobj.export:
            self.DISPATCHER.announce_update(self.name, pname, pobj)
        if err:
            callbacks = self.errorCallbacks
            arg = err
        else:
            callbacks = self.valueCallbacks
            arg = value
        cblist = callbacks[pname]
        for cb in cblist:
            try:
                cb(arg)
            except Exception:
                # print(formatExtendedTraceback())
                pass

    def registerCallbacks(self, modobj, autoupdate=()):
        """register callbacks to another module <modobj>

        - whenever a self.<param> changes:
          <modobj>.update_<param> is called with the new value as argument.
          If this method raises an exception, <modobj>.<param> gets into an error state.
          If the method does not exist and <param> is in autoupdate,
          <modobj>.<param> is updated to self.<param>
        - whenever <self>.<param> gets into an error state:
          <modobj>.error_update_<param> is called with the exception as argument.
          If this method raises an error, <modobj>.<param> gets into an error state.
          If this method does not exist, and <param> is in autoupdate,
          <modobj>.<param> gets into the same error state as self.<param>
        """
        for pname in self.parameters:
            errfunc = getattr(modobj, 'error_update_' + pname, None)
            if errfunc:
                def errcb(err, p=pname, efunc=errfunc):
                    try:
                        efunc(err)
                    except Exception as e:
                        modobj.announceUpdate(p, err=e)
                self.errorCallbacks[pname].append(errcb)
            else:
                def errcb(err, p=pname):
                    modobj.announceUpdate(p, err=err)
                if pname in autoupdate:
                    self.errorCallbacks[pname].append(errcb)

            updfunc = getattr(modobj, 'update_' + pname, None)
            if updfunc:
                def cb(value, ufunc=updfunc, efunc=errcb):
                    try:
                        ufunc(value)
                    except Exception as e:
                        efunc(e)
                self.valueCallbacks[pname].append(cb)
            elif pname in autoupdate:
                def cb(value, p=pname):
                    modobj.announceUpdate(p, value)
                self.valueCallbacks[pname].append(cb)

    def isBusy(self, status=None):
        """helper function for treating substates of BUSY correctly"""
        # defined even for non drivable (used for dynamic polling)
        return False

    def earlyInit(self):
        """initialise module with stuff to be done before all modules are created"""
        self.earlyInitDone = True

    def initModule(self):
        """initialise module with stuff to be done after all modules are created"""
        self.initModuleDone = True

    def startModule(self, start_events):
        """runs after init of all modules

        when a thread is started, a trigger function may signal that it
        has finished its initial work
        start_events.get_trigger(<timeout>) creates such a trigger and
        registers it in the server for waiting
        <timeout> defaults to 30 seconds
        """
        if self.enablePoll or self.writeDict:
            # enablePoll == False: start poll thread for writing values from writeDict only
            mkthread(self.__pollThread, start_events.get_trigger())
        self.startModuleDone = True

    def doPoll(self):
        """polls important parameters like value and status

        all other parameters are polled automatically
        """

    def setFastPoll(self, pollinterval):
        """change poll interval

        :param pollinterval: a new (typically lower) pollinterval
            special values: True: set to 0.25 (default fast poll interval)
            False: set to self.pollinterval (value for idle)
        """
        if pollinterval is False:
            self.changePollinterval.put(self.pollinterval)
            return
        self.changePollinterval.put(0.25 if pollinterval is True else pollinterval)

    def callPollFunc(self, rfunc):
        """call read method with proper error handling"""
        try:
            rfunc()
        except SilentError:
            pass
        except SECoPError as e:
            self.log.error(str(e))
        except Exception:
            self.log.error(formatException())

    def __pollThread(self, started_callback):
        self.writeInitParams()
        if not self.enablePoll:
            return
        polled_parameters = []
        # collect and call all read functions a first time
        for pname, pobj in self.parameters.items():
            rfunc = getattr(self, 'read_' + pname)
            if rfunc.poll:
                polled_parameters.append((rfunc, pobj))
                self.callPollFunc(rfunc)
        started_callback()
        pollinterval = self.pollinterval
        last_slow = last_main = 0
        last_error = None
        error_count = 0
        to_poll = ()
        while True:
            now = time.time()
            wait_main = last_main + pollinterval - now
            wait_slow = last_slow + self.slowinterval - now
            wait_time = min(wait_main, wait_slow)
            if wait_time > 0:
                try:
                    result = self.changePollinterval.get(timeout=wait_time)
                except Empty:
                    result = None
                if result is not None:
                    pollinterval = result
                continue
            # call doPoll, if due
            if wait_main <= 0:
                last_main = (now // pollinterval) * pollinterval
                try:
                    self.doPoll()
                    if last_error and error_count > 1:
                        self.log.info('recovered after %d calls to doPoll (%r)', error_count, last_error)
                    last_error = None
                except Exception as e:
                    if repr(e) != last_error:
                        error_count = 0
                        self.log.error('error in doPoll: %r', e)
                    error_count += 1
                    last_error = repr(e)
                now = time.time()
            # find ONE due slow poll and call it
            loop = True
            while loop:  # loops max. 2 times, when to_poll is at end
                for rfunc, pobj in to_poll:
                    if now > pobj.timestamp + self.slowinterval * 0.5:
                        self.callPollFunc(rfunc)
                        loop = False
                        break
                else:
                    if now < last_slow + self.slowinterval:
                        break
                    last_slow = (now // self.slowinterval) * self.slowinterval
                    to_poll = iter(polled_parameters)

    def writeInitParams(self, started_callback=None):
        """write values for parameters with configured values

        this must be called at the beginning of the poller thread
        with proper error handling
        """
        for pname in list(self.writeDict):
            value = self.writeDict.pop(pname, Done)
            # in the mean time, a poller or handler might already have done it
            if value is not Done:
                try:
                    self.log.debug('initialize parameter %s', pname)
                    getattr(self, 'write_' + pname)(value)
                except SilentError:
                    pass
                except SECoPError as e:
                    self.log.error(str(e))
                except Exception:
                    self.log.error(formatException())
        if started_callback:
            started_callback()

    def setRemoteLogging(self, conn, level):
        if self.remoteLogHandler is None:
            for handler in self.log.handlers:
                if isinstance(handler, RemoteLogHandler):
                    self.remoteLogHandler = handler
                    break
            else:
                raise ValueError('remote handler not found')
        self.remoteLogHandler.set_conn_level(self, conn, level)


class Readable(Module):
    """basic readable module"""
    # pylint: disable=invalid-name
    Status = Enum('Status',
                  IDLE=100,
                  WARN=200,
                  UNSTABLE=270,
                  ERROR=400,
                  DISABLED=0,
                  UNKNOWN=401,
                  )  #: status codes

    value = Parameter('current value of the module', FloatRange())
    status = Parameter('current status of the module', TupleOf(EnumType(Status), StringType()),
                       default=(Status.IDLE, ''))
    pollinterval = Parameter('default poll interval', FloatRange(0.1, 120),
                             default=5, readonly=False, export=True)

    def earlyInit(self):
        super().earlyInit()
        # trigger a poll interval change when self.pollinterval changes.
        # self.setFastPoll with a float argument does the job here
        self.valueCallbacks['pollinterval'].append(self.setFastPoll)

    def doPoll(self):
        self.read_value()
        self.read_status()


class Writable(Readable):
    """basic writable module"""
    disable_value_range_check = Property('disable value range check', BoolType(), default=False)
    target = Parameter('target value of the module',
                       default=0, readonly=False, datatype=FloatRange(unit='$'))

    def __init__(self, name, logger, cfgdict, srv):
        super().__init__(name, logger, cfgdict, srv)
        value_dt = self.parameters['value'].datatype
        target_dt = self.parameters['target'].datatype
        try:
            # this handles also the cases where the limits on the value are more
            # restrictive than on the target
            target_dt.compatible(value_dt)
        except Exception:
            if type(value_dt) == type(target_dt):
                raise ConfigError('the target range extends beyond the value range') from None
            raise ProgrammingError('the datatypes of target and value are not compatible') from None
        if isinstance(value_dt, FloatRange):
            if (not self.disable_value_range_check and not generalConfig.disable_value_range_check
                    and value_dt.problematic_range(target_dt)):
                self.log.error('the value range must be bigger than the target range!')
                self.log.error('you may disable this error message by running the server with --relaxed')
                self.log.error('or by setting the disable_value_range_check property of the module to True')
                raise ConfigError('the value range must be bigger than the target range')


class Drivable(Writable):
    """basic drivable module"""

    Status = Enum(Readable.Status, BUSY=300)  #: status codes

    status = Parameter(datatype=StatusType(Status))  # override Readable.status

    def isBusy(self, status=None):
        """check for busy, treating substates correctly

        returns True when busy (also when finalizing)
        """
        return 300 <= (status or self.status)[0] < 400

    def isDriving(self, status=None):
        """check for driving, treating status substates correctly

        returns True when busy, but not finalizing
        """
        return 300 <= (status or self.status)[0] < 390

    @Command(None, result=None)
    def stop(self):
        """cease driving, go to IDLE state"""


class Communicator(HasComlog, Module):
    """basic abstract communication module"""

    @Command(StringType(), result=StringType())
    def communicate(self, command):
        """communicate command

        :param command: the command to be sent
        :return: the reply
        """
        raise NotImplementedError()


class Attached(Property):
    """a special property, defining an attached module

    assign a module name to this property in the cfg file,
    and the server will create an attribute with this module
    """
    def __init__(self, basecls=Module, description='attached module', mandatory=True):
        self.basecls = basecls
        super().__init__(description, StringType(), mandatory=mandatory)

    def __get__(self, obj, owner):
        if obj is None:
            return self
        if obj.attachedModules is None:
            # return the name of the module (called from Server on startup)
            return super().__get__(obj, owner)
        # return the module (called after startup)
        return obj.attachedModules.get(self.name)  # return None if not given
