#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause
""" Request components shared between USB2 and USB3. """

from amaranth       import *
from amaranth.lib.wiring import Signature, Out

from ...utils.compat import LunaInterface


class SetupPacket(LunaInterface):
    """ Record capturing the content of a setup packet.

    Components (O = output from setup parser; read-only input to others):
        O: received      -- Strobe; indicates that a new setup packet has been received,
                            and thus this data has been updated.

        O: is_in_request -- High if the current request is an 'in' request.
        O: type[2]       -- Request type for the current request.
        O: recipient[5]  -- Recipient of the relevant request.

        O: request[8]    -- Request number.
        O: value[16]     -- Value argument for the setup request.
        O: index[16]     -- Index argument for the setup request.
        O: length[16]    -- Length of the relevant setup request.
    """

    signature = Signature({
        # Byte 1
        'recipient':      Out(5),
        'type':           Out(2),
        'is_in_request':  Out(1),

        # Byte 2
        'request':        Out(8),

        # Byte 3/4
        'value':          Out(16),

        # Byte 5/6
        'index':          Out(16),

        # Byte 7/8
        'length':         Out(16),

        # Control signaling.
        'received':       Out(1),
    })

    def __init__(self):
        super().__init__()
