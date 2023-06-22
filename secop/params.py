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
"""Define classes for Parameters/Commands and Overriding them"""


import inspect

from secop.datatypes import BoolType, CommandType, DataType, \
    DataTypeType, EnumType, IntRange, NoneOr, OrType, \
    StringType, StructOf, TextType, TupleOf, ValueType
from secop.errors import BadValueError, ProgrammingError
from secop.properties import HasProperties, Property

UNSET = object()  # an argument not given, not even None


class Accessible(HasProperties):
    """base class for Parameter and Command

    Inheritance mechanism:

    param.propertyValues contains the properties, which will be used when the
      owner class will be instantiated

    param.ownProperties contains the properties to be used for inheritance
    """

    ownProperties = None

    def init(self, kwds):
        # do not use self.propertyValues.update here, as no invalid values should be
        # assigned to properties, even not before checkProperties
        for k, v in kwds.items():
            self.setProperty(k, v)

    def as_dict(self):
        return self.propertyValues

    def override(self, value):
        """override with a bare value"""
        raise NotImplementedError

    def copy(self):
        """return a (deep) copy of ourselfs"""
        raise NotImplementedError

    def updateProperties(self, merged_properties):
        """update merged_properties with our own properties"""
        raise NotImplementedError

    def merge(self, merged_properties):
        """merge with inherited properties

        :param merged_properties: dict of properties to be updated
        note: merged_properties may be modified
        """
        raise NotImplementedError

    def finish(self):
        """ensure consistency"""
        raise NotImplementedError

    def for_export(self):
        """prepare for serialisation"""
        raise NotImplementedError

    def hasDatatype(self):
        return 'datatype' in self.propertyValues

    def __repr__(self):
        props = []
        for k, v in sorted(self.propertyValues.items()):
            props.append('%s=%r' % (k, v))
        return '%s(%s)' % (self.__class__.__name__, ', '.join(props))


class Parameter(Accessible):
    """defines a parameter

    :param description: description
    :param datatype: the datatype
    :param inherit: whether properties not given should be inherited
    :param kwds: optional properties
    """
    # storage for Parameter settings + value + qualifiers

    description = Property(
        'mandatory description of the parameter', TextType(),
        extname='description', mandatory=True, export='always')
    datatype = Property(
        'datatype of the Parameter (SECoP datainfo)', DataTypeType(),
        extname='datainfo', mandatory=True, export='always', default=ValueType())
    readonly = Property(
        'not changeable via SECoP (default True)', BoolType(),
        extname='readonly', default=True, export='always')
    group = Property(
        'optional parameter group this parameter belongs to', StringType(),
        extname='group', default='')
    visibility = Property(
        'optional visibility hint', EnumType('visibility', user=1, advanced=2, expert=3),
        extname='visibility', default=1)
    constant = Property(
        'optional constant value for constant parameters', ValueType(),
        extname='constant', default=None)
    default = Property(
        '''[internal] default (startup) value of this parameter

        if it can not be read from the hardware''', ValueType(),
        export=False, default=None)
    export = Property(
        '''[internal] export settings

          * False: not accessible via SECoP.
          * True: exported, name automatic.
          * a string: exported with custom name''', OrType(BoolType(), StringType()),
        export=False, default=True)
    poll = Property(
        '''[internal] polling indicator

           may be:

             * None (omitted): will be converted to True/False if handler is/is not None
             * False or 0 (never poll this parameter)
             * True or 1 (AUTO), converted to SLOW (readonly=False)
               DYNAMIC (*status* and *value*) or REGULAR (else)
             * 2 (SLOW), polled with lower priority and a multiple of pollinterval
             * 3 (REGULAR), polled with pollperiod
             * 4 (DYNAMIC), if BUSY, with a fraction of pollinterval,
               else polled with pollperiod
           ''', NoneOr(IntRange()),
        export=False, default=None)
    needscfg = Property(
        '[internal] needs value in config', NoneOr(BoolType()),
        export=False, default=None)
    optional = Property(
        '[internal] is this parameter optional?', BoolType(),
        export=False, settable=False, default=False)
    handler = Property(
        '[internal] overload the standard read and write functions', ValueType(),
        export=False, default=None, settable=False)
    initwrite = Property(
        '''[internal] write this parameter on initialization

        default None: write if given in config''', NoneOr(BoolType()),
        export=False, default=None, settable=False)

    # used on the instance copy only
    value = None
    timestamp = 0
    readerror = None

    def __init__(self, description=None, datatype=None, inherit=True, **kwds):
        super().__init__()
        if datatype is None:
            # collect datatype properties. these are not applied, as we have no datatype
            self.ownProperties = {k: kwds.pop(k) for k in list(kwds) if k not in self.propertyDict}
        else:
            self.ownProperties = {}
            if not isinstance(datatype, DataType):
                if isinstance(datatype, type) and issubclass(datatype, DataType):
                    # goodie: make an instance from a class (forgotten ()???)
                    datatype = datatype()
                else:
                    raise ProgrammingError(
                        'datatype MUST be derived from class DataType!')
            self.datatype = datatype
            if 'default' in kwds:
                self.default = datatype(kwds['default'])

        if description is not None:
            kwds['description'] = inspect.cleandoc(description)

        self.init(kwds)

        if inherit:
            self.ownProperties.update(self.propertyValues)
        else:
            self.ownProperties = {k: getattr(self, k) for k in self.propertyDict}

    def __get__(self, instance, owner):
        # not used yet
        if instance is None:
            return self
        return instance.parameters[self.name].value

    def __set__(self, obj, value):
        obj.announceUpdate(self.name, value)

    def __set_name__(self, owner, name):
        self.name = name
        if isinstance(self.datatype, EnumType):
            self.datatype.set_name(name)

        if self.export is True:
            predefined_cls = PREDEFINED_ACCESSIBLES.get(self.name, None)
            if predefined_cls is Parameter:
                self.export = self.name
            elif predefined_cls is None:
                self.export = '_' + self.name
            else:
                raise ProgrammingError('can not use %r as name of a Parameter' % self.name)

    def copy(self):
        """return a (deep) copy of ourselfs"""
        res = type(self)()
        res.name = self.name
        res.init(self.propertyValues)
        if 'datatype' in self.propertyValues:
            res.datatype = res.datatype.copy()
        return res

    def updateProperties(self, merged_properties):
        """update merged_properties with our own properties"""
        datatype = self.ownProperties.get('datatype')
        if datatype is not None:
            # clear datatype properties, as they are overriden by datatype
            for key in list(merged_properties):
                if key not in self.propertyDict:
                    merged_properties.pop(key)
        merged_properties.update(self.ownProperties)

    def override(self, value):
        """override default"""
        self.default = self.datatype(value)

    def merge(self, merged_properties):
        """merge with inherited properties

        :param merged_properties: dict of properties to be updated
        note: merged_properties may be modified
        """
        datatype = merged_properties.pop('datatype', None)
        if datatype is not None:
            self.datatype = datatype.copy()
        self.init(merged_properties)
        self.finish()

    def finish(self):
        """ensure consistency"""

        if self.constant is not None:
            constant = self.datatype(self.constant)
            # The value of the `constant` property should be the
            # serialised version of the constant, or unset
            self.constant = self.datatype.export_value(constant)
            self.readonly = True
        if 'default' in self.propertyValues:
            # fixes in case datatype has changed
            try:
                self.default = self.datatype(self.default)
            except BadValueError:
                # clear default, if it does not match datatype
                self.propertyValues.pop('default')

    def export_value(self):
        return self.datatype.export_value(self.value)

    def for_export(self):
        return dict(self.exportProperties(), readonly=self.readonly)

    def getProperties(self):
        """get also properties of datatype"""
        super_prop = super().getProperties().copy()
        if self.datatype:
            super_prop.update(self.datatype.getProperties())
        return super_prop

    def setProperty(self, key, value):
        """set also properties of datatype"""
        try:
            if key in self.propertyDict:
                super().setProperty(key, value)
            else:
                try:
                    self.datatype.setProperty(key, value)
                except KeyError:
                    raise ProgrammingError('cannot set %s on parameter with datatype %s'
                                           % (key, type(self.datatype).__name__)) from None
        except ValueError as e:
            raise ProgrammingError('property %s: %s' % (key, str(e))) from None

    def checkProperties(self):
        super().checkProperties()
        self.datatype.checkProperties()


class Command(Accessible):
    """decorator to turn a method into a command

    :param argument: the datatype of the argument or None
    :param result: the datatype of the result or None
    :param inherit: whether properties not given should be inherited
    :param kwds: optional properties
    """

    description = Property(
        'description of the Command', TextType(),
        extname='description', export='always', mandatory=True)
    group = Property(
        'optional command group of the command.', StringType(),
        extname='group', export=True, default='')
    visibility = Property(
        'optional visibility hint', EnumType('visibility', user=1, advanced=2, expert=3),
        extname='visibility', export=True, default=1)
    export = Property(
        '''[internal] export settings

          * False: not accessible via SECoP.
          * True: exported, name automatic.
          * a string: exported with custom name''', OrType(BoolType(), StringType()),
        export=False, default=True)
    optional = Property(
        '[internal] is the command optional to implement? (vs. mandatory)', BoolType(),
        export=False, default=False, settable=False)
    datatype = Property(
        "datatype of the command, auto generated from 'argument' and 'result'",
        DataTypeType(), extname='datainfo', export='always')
    argument = Property(
        'datatype of the argument to the command, or None', NoneOr(DataTypeType()),
        export=False, mandatory=True)
    result = Property(
        'datatype of the result from the command, or None', NoneOr(DataTypeType()),
        export=False, mandatory=True)

    func = None

    def __init__(self, argument=False, *, result=None, inherit=True, **kwds):
        super().__init__()
        self.init(kwds)
        if result or kwds or isinstance(argument, DataType) or not callable(argument):
            # normal case
            if argument is False and result:
                argument = None
            if argument is not False:
                if isinstance(argument, (tuple, list)):
                    # goodie: treat as TupleOf
                    argument = TupleOf(*argument)
                self.argument = argument
                self.result = result
        else:
            # goodie: allow @Command instead of @Command()
            self.func = argument  # this is the wrapped method!
            if argument.__doc__:
                self.description = inspect.cleandoc(argument.__doc__)
            self.name = self.func.__name__
        self._inherit = inherit  # save for __set_name__

    def __set_name__(self, owner, name):
        self.name = name
        if self.func is None:
            raise ProgrammingError('Command %s.%s must be used as a method decorator' %
                                   (owner.__name__, name))

        self.datatype = CommandType(self.argument, self.result)
        self.ownProperties = self.propertyValues.copy()
        if self.export is True:
            predefined_cls = PREDEFINED_ACCESSIBLES.get(name, None)
            if predefined_cls is Command:
                self.export = name
            elif predefined_cls is None:
                self.export = '_' + name
            else:
                raise ProgrammingError('can not use %r as name of a Command' % name) from None
        if not self._inherit:
            for key, pobj in self.properties.items():
                if key not in self.propertyValues:
                    self.propertyValues[key] = pobj.default

    def __get__(self, obj, owner=None):
        if obj is None:
            return self
        if not self.func:
            raise ProgrammingError('Command %s not properly configured' % self.name) from None
        return self.func.__get__(obj, owner)

    def __call__(self, func):
        """called when used as decorator"""
        if 'description' not in self.propertyValues and func.__doc__:
            self.description = inspect.cleandoc(func.__doc__)
        self.func = func
        return self

    def copy(self):
        """return a (deep) copy of ourselfs"""
        res = type(self)()
        res.name = self.name
        res.func = self.func
        res.init(self.propertyValues)
        if res.argument:
            res.argument = res.argument.copy()
        if res.result:
            res.result = res.result.copy()
        self.finish()
        return res

    def updateProperties(self, merged_properties):
        """update merged_properties with our own properties"""
        merged_properties.update(self.ownProperties)

    def override(self, value):
        """override method

        this is needed when the @Command is missing on a method overriding a command"""
        if not callable(value):
            raise ProgrammingError('%s = %r is overriding a Command' % (self.name, value))
        self.func = value
        if value.__doc__:
            self.description = inspect.cleandoc(value.__doc__)

    def merge(self, merged_properties):
        """merge with inherited properties

        :param merged_properties: dict of properties to be updated
        """
        self.init(merged_properties)
        self.finish()

    def finish(self):
        """ensure consistency"""
        self.datatype = CommandType(self.argument, self.result)

    def setProperty(self, key, value):
        """special treatment of datatype"""
        try:
            if key == 'datatype':
                command = DataTypeType()(value)
                super().setProperty('argument', command.argument)
                super().setProperty('result', command.result)
            super().setProperty(key, value)
        except ValueError as e:
            raise ProgrammingError('property %s: %s' % (key, str(e))) from None

    def do(self, module_obj, argument):
        """perform function call

        :param module_obj: the module on which the command is to be executed
        :param argument: the argument from the do command
        :returns: the return value converted to the result type

        - when the argument type is TupleOf, the function is called with multiple arguments
        - when the argument type is StructOf, the function is called with keyworded arguments
        - the validity of the argument/s is/are checked
        """
        func = self.__get__(module_obj)
        if self.argument:
            # validate
            argument = self.argument(argument)
            if isinstance(self.argument, TupleOf):
                res = func(*argument)
            elif isinstance(self.argument, StructOf):
                res = func(**argument)
            else:
                res = func(argument)
        else:
            if argument is not None:
                raise BadValueError('%s.%s takes no arguments' % (module_obj.__class__.__name__, self.name))
            res = func()
        if self.result:
            return self.result(res)
        return None  # silently ignore the result from the method

    def for_export(self):
        return self.exportProperties()

    def __repr__(self):
        result = super().__repr__()
        return result[:-1] + ', %r)' % self.func if self.func else result


# list of predefined accessibles with their type
PREDEFINED_ACCESSIBLES = dict(
    value=Parameter,
    status=Parameter,
    target=Parameter,
    pollinterval=Parameter,
    ramp=Parameter,
    user_ramp=Parameter,
    setpoint=Parameter,
    time_to_target=Parameter,
    unit=Parameter,  # reserved name
    loglevel=Parameter,  # reserved name
    mode=Parameter,  # reserved name
    stop=Command,
    reset=Command,
    go=Command,
    abort=Command,
    shutdown=Command,
    communicate=Command,
)
