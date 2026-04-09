#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Core stream definitions. """

from amaranth            import Elaboratable, Signal, Module
from amaranth.lib.wiring import Signature, In, Out

from ..utils.compat      import LunaInterface


class StreamInterface(LunaInterface):
    """ Simple record implementing a unidirectional data stream.

    This class is similar to LiteX's streams; but instances may be optimized for
    interaction with USB PHYs. Accordingly, some uses may add restrictions; this
    is typically indicated by subclassing this interface.

    Attributes
    -----------
    valid: Signal(), from originator
        Indicates that the current payload bytes are valid pieces of the current transaction.
    first: Signal(), from originator
        Indicates that the payload byte is the first byte of a new packet.
    last: Signal(), from originator
        Indicates that the payload byte is the last byte of the current packet.
    payload: Signal(payload_width), from originator
        The data payload to be transmitted.

    ready: Signal(), from receiver
        Indicates that the receiver will accept the payload byte at the next active
        clock edge. Can be de-asserted to put backpressure on the transmitter.

    Parameters
    ----------
    payload_width: int
        The width of the stream's payload, in bits.
    extra_fields: list of tuples, optional
        A flat (non-nested) list of tuples indicating any extra fields present.
        Similar to a record's layout field; but cannot be nested.
    """

    def __init__(self, payload_width=8, valid_width=1, extra_fields=None):
        """
        Parameter:
            payload_width -- The width of the payload packets.
        """

        # If we don't have any extra fields, use an empty list in its place.
        if extra_fields is None:
            extra_fields = []

        # ... store our extra fields...
        self._extra_fields = extra_fields

        # Build signature dynamically based on parameters.
        members = {
            'valid':   Out(valid_width),
            'ready':   Out(1),
            'first':   Out(1),
            'last':    Out(1),
            'payload': Out(payload_width),
        }
        for field_def in extra_fields:
            name = field_def[0]
            width = field_def[1]
            members[name] = Out(width)

        super().__init__(Signature(members))


    def attach(self, interface, omit=None):
        # Create lists of fields to be copied -to- the interface (RHS fields),
        # and lists of fields to be copied -from- the interface (LHS fields).
        extra_field_names = [f[0] for f in self._extra_fields]
        rhs_fields = ['valid', 'first', 'last', 'payload', *extra_field_names]
        lhs_fields = ['ready']
        assignments = []

        if omit:
            rhs_fields = [field for field in rhs_fields if field not in omit]
            lhs_fields = [field for field in lhs_fields if field not in omit]


        # Create each of our assignments.
        for field in rhs_fields:
            assignment = getattr(interface, field).eq(getattr(self, field))
            assignments.append(assignment)
        for field in lhs_fields:
            assignment = getattr(self, field).eq(getattr(interface, field))
            assignments.append(assignment)

        return assignments

    def connect(self, interface, omit=None):
        return self.attach(interface, omit=omit)


    def stream_eq(self, interface, *, omit=None):
        """ A hopefully more clear version of .connect() that more clearly indicates data_flow direction.

        This will either solve a common footgun or introduce a new one. We'll see and adapt accordingly.
        """
        return interface.attach(self, omit=omit)


    def tap(self, interface, *, tap_ready=False, **kwargs):
        """ Simple extension to stream_eq() that captures a read-only view of the stream.

        This connects all signals from ``interface`` to their equivalents in this stream.
        """
        core = self.stream_eq(interface, omit={"ready"}, **kwargs)

        if tap_ready:
            core.append(self.ready.eq(interface.ready))

        return core




    def __getattr__(self, name):

        # Allow "data" to be a semantic alias for payload.
        # In some cases, this makes more sense to write; so we'll allow either.
        # Individual sections of the code base should stick to one or the other (please).
        if name == 'data':
            return self.payload

        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
