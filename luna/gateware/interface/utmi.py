#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" UTMI interfacing. """

from enum import IntEnum

from amaranth            import Elaboratable, Signal, Module
from amaranth.lib.wiring import Signature, In, Out

from ..utils.bus    import OneHotMultiplexer
from ..utils.compat import LunaInterface

class UTMIOperatingMode:
    """ Enumeration that specifies the modes a UTMI transceiver can use. """

    NORMAL                    = 0
    NON_DRIVING               = 1

    RAW_DRIVE                 = 2
    DISABLE_BITSTUFF_AND_NRZI = 2
    CHIRP                     = 2

    NO_SYNC_OR_EOP            = 3


class UTMITerminationSelect:
    """ Enumeration that specifies meanings of the UTMI TermSelect bit. """

    HS_NORMAL    = 0
    HS_CHIRP     = 1
    LS_FS_NORMAL = 1


class UTMITransmitInterface(LunaInterface):
    """ Interface present on hardware that transmits onto a UTMI bus. """

    signature = Signature({
        # Indicates when the data on tx_data is valid.
        'valid': Out(1),
        # The data to be transmitted.
        'data':  Out(8),
        # Pulsed by the UTMI bus when the given data byte will be accepted
        # at the next clock edge.
        'ready': In(1),
    })

    def __init__(self):
        super().__init__()


    def attach(self, utmi_bus):
        """ Returns a list of connection fragments connecting this interface to the provided bus.

        A typical usage might look like:
            m.d.comb += interface_object.attach(utmi_bus)
        """

        return [
            utmi_bus.tx_data   .eq(self.data),
            utmi_bus.tx_valid  .eq(self.valid),

            self.ready          .eq(utmi_bus.tx_ready),
        ]


class UTMIInterfaceMultiplexer(OneHotMultiplexer):
    """ Gateware that merges a collection of UTMITransmitInterfaces into a single interface.

    Assumes that only one transmitter will be communicating at once.

    I/O port:
        O*: output -- Our output interface; has all of the active busses merged together.
    """

    def __init__(self):
        super().__init__(
            interface_type=UTMITransmitInterface,
            mux_signals= ('data',),
            or_signals=  ('valid',),
            pass_signals=('ready',)
        )




class UTMIInterface(LunaInterface):
    """ UTMI+-standardized interface. Intended mostly as a simulation aid."""

    signature = Signature({
        # Core signals.
        'rx_data':                     Out(8),
        'rx_active':                   Out(1),
        'rx_valid':                    Out(1),

        'tx_data':                     Out(8),
        'tx_valid':                    Out(1),
        'tx_ready':                    Out(1),

        # Control signals.
        'xcvr_select':                 Out(2),
        'term_select':                 Out(1),
        'op_mode':                     Out(2),
        'suspend':                     Out(1),
        'id_pullup':                   Out(1),
        'dm_pulldown':                 Out(1),
        'dp_pulldown':                 Out(1),
        'chrg_vbus':                   Out(1),
        'dischrg_vbus':                Out(1),
        'use_external_vbus_indicator': Out(1),

        # Event signals.
        'line_state':                  Out(2),
        'vbus_valid':                  Out(1),
        'session_valid':               Out(1),
        'session_end':                 Out(1),
        'rx_error':                    Out(1),
        'host_disconnect':             Out(1),
        'id_digital':                  Out(1),
    })

    def __init__(self):
        super().__init__()
