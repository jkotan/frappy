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
"""generic SEA driver

a object or subobject in sea may be assigned to a SECoP module

Examples:

SECoP       SEA          hipadaba path   mod.obj  mod.sub  par.sub   mod.path
-------------------------------------------------------------------------------
tt:maxwait  tt           /tt/maxwait     tt                maxwait   /tt
tt:ramp     tt set/ramp  /tt/set/ramp    tt                set/ramp  /tt
t1:raw      tt t1/raw    /tt/t1/raw      tt       t1       raw       /tt/t1
rx:bla      rx bla       /some/rx_a/bla  rx                bla       /some/rx_a
"""

import json
import threading
import time
import os
from os.path import expanduser, join, exists

from secop.client import ProxyClient
from secop.datatypes import ArrayOf, BoolType, \
    EnumType, FloatRange, IntRange, StringType
from secop.errors import ConfigError, HardwareError, secop_error, NoSuchModuleError
from secop.lib import getGeneralConfig, mkthread, formatExtendedStack
from secop.lib.asynconn import AsynConn, ConnectionClosed
from secop.modules import Attached, Command, Done, Drivable, \
    Module, Parameter, Property, Readable, Writable
from secop.protocol.dispatcher import make_update


CFG_HEADER = """[NODE]
description = %(nodedescr)s
id = %(config)s.sea.psi.ch

[%(seaconn)s]
class = secop_psi.sea.SeaClient
description = %(service)s sea connection for %(config)s
config = %(config)s
service = %(service)s
"""

CFG_MODULE = """
[%(module)s]
class = secop_psi.sea.%(modcls)s
iodev = %(seaconn)s
sea_object = %(module)s
"""

SERVICE_NAMES = {
    'config': 'main',
    'stick': 'stick',
    'addon': 'addons',
}

SEA_DIR = expanduser('~/sea')
for confdir in getGeneralConfig()['confdir'].split(os.pathsep):
    seaconfdir = join(confdir, 'sea')
    if exists(seaconfdir):
        break
else:
    seaconfdir = os.environ.get('FRAPPY_SEA_DIR')


def get_sea_port(instance):
    for filename in ('sea_%s.tcl' % instance, 'sea.tcl'):
        try:
            with open(join(SEA_DIR, filename)) as f:
                for line in f:
                    linesplit = line.split()
                    if len(linesplit) == 3:
                        _, var, value = line.split()
                        if var == 'serverport':
                            return value
        except FileNotFoundError:
            pass
    return None


class SeaClient(ProxyClient, Module):
    """connection to SEA"""

    uri = Parameter('hostname:portnumber', datatype=StringType(), default='localhost:5000')
    timeout = Parameter('timeout', datatype=FloatRange(0), default=10)
    config = Property("""needed SEA configuration, space separated

                      Example: "ori4.config ori4.stick"
                      """, StringType(), default='')
    service = Property("main/stick/addons", StringType(), default='')
    visibility = 'expert'
    default_json_file = {}

    def __init__(self, name, log, opts, srv):
        instance = srv.node_cfg['name'].rsplit('_', 1)[0]
        if 'uri' not in opts:
            port = get_sea_port(instance)
            if port is None:
                raise ConfigError('missing sea port for %s' % instance)
            opts['uri'] = 'tcp://localhost:%s' % port
        self.objects = []
        self.shutdown = False
        self.path2param = {}
        self._write_lock = threading.Lock()
        config = opts.get('config')
        if config:
            self.default_json_file[name] = config.split()[0] + '.json'
        self.io = None
        self.asyncio = None
        ProxyClient.__init__(self)
        Module.__init__(self, name, log, opts, srv)

    def register_obj(self, module, obj):
        self.objects.append(obj)
        self.path2param.update(module.path2param)
        self.register_callback(module.name, module.updateEvent)

    def startModule(self, started_callback):
        mkthread(self._connect, started_callback)

    def _connect(self, started_callback):
        if '//' not in self.uri:
            self.uri = 'tcp://' + self.uri
        self.asyncio = AsynConn(self.uri)
        assert self.asyncio.readline() == b'OK'
        self.asyncio.writeline(b'Spy 007')
        assert self.asyncio.readline() == b'Login OK'
        self.request('frappy_config %s %s' % (self.service, self.config))

        # frappy_async_client switches to the json protocol (better for updates)
        self.asyncio.writeline(b'frappy_async_client')
        self.asyncio.writeline(('get_all_param ' + ' '.join(self.objects)).encode())
        mkthread(self._rxthread, started_callback)

    def request(self, command):
        """send a request and wait for reply"""
        with self._write_lock:
            if not self.io or not self.io.connection:
                if not self.asyncio or not self.asyncio.connection:
                    self._connect(None)
                self.io = AsynConn(self.uri)
                assert self.io.readline() == b'OK'
                self.io.writeline(b'seauser seaser')
                assert self.io.readline() == b'Login OK'
                print('connected to %s' % self.uri)
            self.io.flush_recv()
            # print('> %s' % command)
            self.io.writeline(('fulltransact %s' % command).encode())
            result = None
            deadline = time.time() + 10
            while time.time() < deadline:
                try:
                    reply = self.io.readline()
                    if reply is None:
                        continue
                except ConnectionClosed:
                    break
                reply = reply.decode()
                # print('< %s' % reply)
                if reply.startswith('TRANSACTIONSTART'):
                    result = []
                    continue
                if reply == 'TRANSACTIONFINISHED':
                    if result is None:
                        print('missing TRANSACTIONSTART on: %s' % command)
                        return ''
                    if not result:
                        return ''
                    return '\n'.join(result)
                if result is None:
                    print('swallow: %s' % reply)
                    continue
                if not result:
                    result = [reply.split('=', 1)[-1]]
                else:
                    result.append(reply)
        raise TimeoutError('no response within 10s')

    def _rxthread(self, started_callback):
        while not self.shutdown:
            try:
                reply = self.asyncio.readline()
                if reply is None:
                    continue
            except ConnectionClosed:
                break
            try:
                msg = json.loads(reply)
            except Exception as e:
                print(repr(e), reply)
                continue
            if isinstance(msg, str):
                if msg.startswith('_E '):
                    try:
                        _, path, readerror = msg.split(None, 2)
                    except ValueError:
                        continue
                else:
                    continue
                data = {'%s.geterror' % path: readerror.replace('ERROR: ', '')}
                obj = None
                flag = 'hdbevent'
            else:
                obj = msg['object']
                flag = msg['flag']
                data = msg['data']
            if flag == 'finish' and obj == 'get_all_param':
                # first updates have finished
                if started_callback:
                    started_callback()
                    started_callback = None
                continue
            if flag != 'hdbevent':
                if obj not in ('frappy_async_client', 'get_all_param'):
                    print('SKIP', msg)
                continue
            if not data:
                continue
            if not isinstance(data, dict):
                print('what means %r' % msg)
                continue
            now = time.time()
            for path, value in data.items():
                readerror = None
                if path.endswith('.geterror'):
                    if value:
                        readerror = HardwareError(value)
                    path = path.rsplit('.', 1)[0]
                    value = None
                try:
                    module, param = self.path2param[path]
                except KeyError:
                    if path.startswith('/device'):
                        if path == '/device/changetime':
                            result = self.request('check_config %s %s' % (self.service, self.config))
                            if result == '1':
                                self.asyncio.writeline(('get_all_param ' + ' '.join(self.objects)).encode())
                            else:
                                self.DISPATCHER.shutdown()
                        elif path.startswith('/device/frappy_%s' % self.service) and value == '':
                            self.DISPATCHER.shutdown()
                    # print('UNUSED', msg)
                    continue  # unused parameter
                oldv, oldt, oldr = self.cache.get((module, param), [None, None, None])
                if value is None:
                    value = oldv
                if value != oldv or str(readerror) != str(oldr) or abs(now - oldt) > 60:
                    # do not update unchanged values within 60 sec
                    self.updateValue(module, param, value, now, readerror)


    @Command(StringType(), result=StringType())
    def communicate(self, command):
        """send a command to SEA"""
        reply = self.request(command)
        return reply

    @Command(StringType(), result=StringType())
    def query(self, cmd):
        """a request checking for errors and accepting 0 or 1 line as result"""
        errors = []
        reply = None
        for line in self.request(cmd).split('\n'):
            if line.strip().startswith('ERROR:'):
                errors.append(line[6:].strip())
            elif reply is None:
                reply = line.strip()
            else:
                self.log.info('SEA: superfluous reply %r to %r', reply, cmd)
        if errors:
            raise HardwareError('; '.join(errors))
        return reply


class SeaConfigCreator(SeaClient):
    def startModule(self, started_callback):
        """save objects (and sub-objects) description and exit"""
        self._connect(lambda: None)
        reply = self.request('describe_all')
        reply = ''.join('' if line.startswith('WARNING') else line for line in reply.split('\n'))
        description, reply = json.loads(reply)
        modules = {}
        modcls = {}
        for filename, obj, descr in reply:
            if filename not in modules:
                modules[filename] = {}
            if descr['params'][0].get('cmd', '').startswith('run '):
                modcls[obj] = 'SeaDrivable'
            elif not descr['params'][0].get('readonly', True):
                modcls[obj] = 'SeaWritable'
            else:
                modcls[obj] = 'SeaReadable'
            modules.setdefault(filename, {})[obj] = descr

        result = []
        for filename, descr in modules.items():
            stripped, _, ext = filename.rpartition('.')
            service = SERVICE_NAMES[ext]
            seaconn = 'sea_' + service
            cfgfile = join(seaconfdir, stripped + '.cfg')
            with open(cfgfile, 'w') as fp:
                fp.write(CFG_HEADER % dict(config=filename, seaconn=seaconn, service=service,
                                           nodedescr=description.get(filename, filename)))
                for obj in descr:
                    fp.write(CFG_MODULE % dict(modcls=modcls[obj], module=obj, seaconn=seaconn))
            content = json.dumps(descr).replace('}, {', '},\n{').replace('[{', '[\n{').replace('}]}, ', '}]},\n\n')
            result.append('%s\n' % cfgfile)
            with open(join(seaconfdir, filename + '.json'), 'w') as fp:
                fp.write(content + '\n')
            result.append('%s: %s' % (filename, ','.join(n for n in descr)))
        raise SystemExit('; '.join(result))

    @Command(StringType(), result=StringType())
    def query(self, cmd):
        """a request checking for errors and accepting 0 or 1 line as result"""
        errors = []
        reply = None
        for line in self.request(cmd).split('\n'):
            if line.strip().startswith('ERROR:'):
                errors.append(line[6:].strip())
            elif reply is None:
                reply = line.strip()
            else:
                self.log.info('SEA: superfluous reply %r to %r', reply, cmd)
        if errors:
            raise HardwareError('; '.join(errors))
        return reply


SEA_TO_SECOPTYPE = {
    'float': FloatRange(),
    'text': StringType(),
    'int': IntRange(-1 << 63, 1 << 63 - 1),
    'bool': BoolType(),
    'none': None,
    'floatvarar': ArrayOf(FloatRange(), 0, 400),  # 400 is the current limit for proper notify events in SEA
}


def get_datatype(paramdesc):
    typ = paramdesc['type']
    result = SEA_TO_SECOPTYPE.get(typ, False)
    if result is not False:  # general case
        return result
    # special cases
    if typ == 'enum':
        return EnumType(paramdesc['enum'])
    raise ValueError('unknown SEA type %r' % typ)


class SeaModule(Module):
    iodev = Attached()

    # pollerClass=None
    path2param = None
    sea_object = None
    hdbpath = None  # hdbpath for main writable

    def __new__(cls, name, logger, cfgdict, srv):
        if hasattr(srv, 'extra_sea_modules'):
            extra_modules = srv.extra_sea_modules
        else:
            extra_modules = {}
            srv.extra_sea_modules = extra_modules
        json_file = cfgdict.pop('json_file', None) or SeaClient.default_json_file[cfgdict['iodev']]
        visibility_level = cfgdict.pop('visibility_level', 2)

        single_module = cfgdict.pop('single_module', None)
        if single_module:
            sea_object, base, paramdesc = extra_modules[single_module]
            params = [paramdesc]
            paramdesc['key'] = 'value'
            if issubclass(cls, SeaWritable):
                if paramdesc.get('readonly', True):
                    raise ConfigError('%s/%s is not writable' % (sea_object, paramdesc['path']))
                paramdesc['key'] = 'target'
                paramdesc['readonly'] = False
            extra_module_set = ()
            if 'description' not in cfgdict:
                cfgdict['description'] = '%s@%s' % (single_module, json_file)
        else:
            sea_object = cfgdict.pop('sea_object')
            rel_paths = cfgdict.pop('rel_paths', '.')
            if 'description' not in cfgdict:
                cfgdict['description'] = '%s@%s%s' % (
                    name, json_file, '' if rel_paths == '.' else ' (rel_paths=%s)' % rel_paths)

            # with open(join(seaconfdir, json_file + '.json')) as fp:
            #    sea_object, descr = json.load(fp)
            with open(join(seaconfdir, json_file)) as fp:
                descr = json.load(fp)[sea_object]
            if rel_paths == '*' or not rel_paths:
                # take all
                main = descr['params'][0]
                if issubclass(cls, Readable):
                    # assert main['path'] == ''  # TODO: check cases where this fails
                    main['key'] = 'value'
                else:
                    descr['params'].pop(0)
            else:
                # filter by relative paths
                rel_paths = rel_paths.split()
                result = []
                for rpath in rel_paths:
                    include = True
                    for paramdesc in descr['params']:
                        path = paramdesc['path']
                        if path.endswith('is_running'):
                            # take this always
                            result.append(paramdesc)
                            continue
                        if paramdesc.get('visibility', 1) > visibility_level:
                            continue
                        sub = path.split('/', 1)
                        if rpath == '.':  # take all except subpaths with readonly node at top
                            if len(sub) == 1:
                                include = paramdesc.get('kids', 0) == 0 or not paramdesc.get('readonly', True)
                            if include or path == '':
                                result.append(paramdesc)
                        elif sub[0] == rpath:
                            result.append(paramdesc)
                descr['params'] = result
                rel0 = '' if rel_paths[0] == '.' else rel_paths[0]
                if result[0]['path'] == rel0:
                    if issubclass(cls, Readable):
                        result[0]['key'] = 'value'
                    else:
                        result.pop(0)
                else:
                    logger.error('%s: no value found', name)
                # logger.info('PARAMS %s %r', name, result)
            base = descr['base']
            params = descr['params']
            if issubclass(cls, SeaWritable):
                paramdesc = params[0]
                assert paramdesc['key'] == 'value'
                if paramdesc.get('readonly', True):
                    raise ConfigError('%s/%s is not writable' % (sea_object, paramdesc['path']))
                paramdesc['key'] = 'target'
                paramdesc['readonly'] = False
            extra_module_set = cfgdict.pop('extra_modules', ())
            if extra_module_set:
                extra_module_set = set(extra_module_set.replace(',', ' ').split())
        path2param = {}
        attributes = dict(sea_object=sea_object, path2param=path2param)

        # some guesses about visibility (may be overriden in *.cfg):
        if sea_object in ('table', 'cc'):
            attributes['visibility'] = 2
        elif base.count('/') > 1:
            attributes['visibility'] = 2
        for paramdesc in params:
            path = paramdesc['path']
            readonly = paramdesc.get('readonly', True)
            dt = get_datatype(paramdesc)
            kwds = dict(description=paramdesc.get('description', path),
                        datatype=dt,
                        visibility=paramdesc.get('visibility', 1),
                        needscfg=False, poll=False, readonly=readonly)
            if kwds['datatype'] is None:
                kwds.update(visibility=3, default='', datatype=StringType())
            pathlist = path.split('/') if path else []
            key = paramdesc.get('key')  # None, 'value' or 'target'
            if key is None:
                if len(pathlist) > 0:
                    if len(pathlist) == 1:
                        if issubclass(cls, Readable):
                            kwds['group'] = 'more'
                    else:
                        kwds['group'] = pathlist[-2]
                # flatten path to parameter name
                for i in reversed(range(len(pathlist))):
                    key = '_'.join(pathlist[i:])
                    if not key in cls.accessibles:
                        break
                if key == 'is_running':
                    kwds['export'] = False
            if key == 'target' and kwds.get('group') == 'more':
                kwds.pop('group')
            if key in cls.accessibles:
                if key == 'target':
                    kwds['readonly'] = False
                pobj = cls.accessibles[key]
                pobj.init(kwds)
                datatype = kwds.get('datatype', cls.accessibles[key].datatype)
            else:
                pobj = Parameter(**kwds)
                datatype = pobj.datatype
            if issubclass(cls, SeaWritable) and key == 'target':
                kwds['readonly'] = False
                attributes['value'] = Parameter(**kwds)

            hdbpath = '/'.join([base] + pathlist)
            if key in extra_module_set:
                extra_modules[name + '.' + key] = sea_object, base, paramdesc
                continue  # skip this parameter
            path2param[hdbpath] = (name, key)
            attributes[key] = pobj
            # if hasattr(cls, 'read_' + key):
            #     print('override %s.read_%s' % (cls.__name__, key))

            def rfunc(self, cmd='hval %s/%s' % (base, path)):
                reply = self._iodev.query(cmd)
                try:
                    reply = float(reply)
                except ValueError:
                    pass
                # an updateEvent will be handled before above returns
                return reply

            attributes['read_' + key] = rfunc

            if not readonly:
                # if hasattr(cls, 'write_' + key):
                #     print('override %s.write_%s' % (cls.__name__, key))

                def wfunc(self, value, datatype=datatype, command=paramdesc['cmd']):
                    value = datatype.export_value(value)
                    if isinstance(value, bool):
                        value = int(value)
                        # TODO: check if more has to be done for valid tcl data (strings?)
                    cmd = "%s %s" % (command, value)
                    self._iodev.query(cmd)
                    return Done

                attributes['write_' + key] = wfunc

        # create standard parameters like value and status, if not yet there
        for pname, pobj in cls.accessibles.items():
            if pname == 'pollinterval':
                pobj.export = False
                attributes[pname] = pobj
                pobj.__set_name__(cls, pname)
            elif pname not in attributes and isinstance(pobj, Parameter):
                pobj.poll = False
                pobj.needscfg = False
                attributes[pname] = pobj
                pobj.__set_name__(cls, pname)

        classname = '%s_%s' % (cls.__name__, sea_object)
        attributes['pollerClass'] = None
        newcls = type(classname, (cls,), attributes)
        return Module.__new__(newcls)

    def updateEvent(self, module, parameter, value, timestamp, readerror):
        upd = getattr(self, 'update_' + parameter, None)
        if upd:
            upd(value, timestamp, readerror)
            return
        try:
            pobj = self.parameters[parameter]
        except KeyError:
            print(self.name, parameter)
            raise
        pobj.timestamp = timestamp
        #if not pobj.readonly and pobj.value != value:
        #    print('UPDATE', module, parameter, value)
        # should be done here: deal with clock differences
        if not readerror:
            try:
                pobj.value = value  # store the value even in case of a validation error
                pobj.value = pobj.datatype(value)
            except Exception as e:
                readerror = secop_error(e)
        pobj.readerror = readerror
        self.DISPATCHER.broadcast_event(make_update(self.name, pobj))

    def initModule(self):
        self._iodev.register_obj(self, self.sea_object)
        super().initModule()


class SeaReadable(SeaModule, Readable):

    def update_status(self, value, timestamp, readerror):
        if readerror:
            value = repr(readerror)
        if value == '':
            self.status = (self.Status.IDLE, '')
        else:
            self.status = (self.Status.ERROR, value)

    def read_status(self):
        return self.status


class SeaWritable(SeaModule, Writable):
    def read_value(self):
        return self.target

    def update_target(self, value, timestamp, readerror):
        self.target = value
        if not readerror:
            self.value = value


class SeaDrivable(SeaModule, Drivable):
    _sea_status = ''
    _is_running = 0

    def read_status(self):
        return self.status

    # def read_target(self):
    #    return self.target

    def write_target(self, value):
        self._iodev.query('run %s %s' % (self.sea_object, value))
        #self.status = [self.Status.BUSY, 'driving']
        return value

    def update_status(self, value, timestamp, readerror):
        if not readerror:
            self._sea_status = value
            self.updateStatus()

    def update_is_running(self, value, timestamp, readerror):
        if not readerror:
            self._is_running = value
            self.updateStatus()

    def updateStatus(self):
        if self._sea_status:
            self.status = (self.Status.ERROR, self._sea_status)
        elif self._is_running:
            self.status = (self.Status.BUSY, 'driving')
        else:
            self.status = (self.Status.IDLE, '')

    def updateTarget(self, module, parameter, value, timestamp, readerror):
        if value is not None:
            self.target = value

    @Command()
    def stop(self):
        self._iodev.query('%s is_running 0' % self.sea_object)
