Reference
---------

Module Base Classes
...................

.. autoclass:: secop.modules.Module
    :members: earlyInit, initModule, startModule, pollerClass

.. autoclass:: secop.modules.Readable
    :members: Status

.. autoclass:: secop.modules.Writable

.. autoclass:: secop.modules.Drivable
    :members: Status, isBusy, isDriving, stop


Parameters, Commands and Properties
...................................

.. autoclass:: secop.params.Parameter
.. autoclass:: secop.params.Command
.. autoclass:: secop.properties.Property
.. autoclass:: secop.modules.Attached
    :show-inheritance:


Datatypes
.........

.. autoclass:: secop.datatypes.FloatRange
.. autoclass:: secop.datatypes.IntRange
.. autoclass:: secop.datatypes.BoolType
.. autoclass:: secop.datatypes.ScaledInteger
.. autoclass:: secop.datatypes.EnumType
.. autoclass:: secop.datatypes.StringType
.. autoclass:: secop.datatypes.TupleOf
.. autoclass:: secop.datatypes.ArrayOf
.. autoclass:: secop.datatypes.StructOf
.. autoclass:: secop.datatypes.BLOBType



Communication
.............

.. autoclass:: secop.modules.Communicator
    :show-inheritance:
    :members: communicate

.. autoclass:: secop.stringio.StringIO
    :show-inheritance:
    :members: communicate, multicomm

.. autoclass:: secop.stringio.HasIodev
    :show-inheritance:

.. autoclass:: secop.iohandler.IOHandlerBase
    :show-inheritance:
    :members:

.. autoclass:: secop.iohandler.IOHandler
    :show-inheritance:
    :members:


Exception classes
.....................--

.. automodule:: secop.errors
    :members:

.. include:: server.rst

