#
# This file is part of LUNA.
#
# Compatibility utilities for migrating from amaranth Record to
# amaranth.lib.wiring (Signature/PureInterface) and amaranth.lib.data (Struct).
#

from amaranth            import Elaboratable, Signal, Module, Cat
from amaranth.lib.wiring import Signature, In, Out, Flow


class LunaInterface:
    """Base class replacing Record for LUNA interfaces.

    Uses amaranth.lib.wiring.Signature to define interface members with
    proper In/Out flow annotations. Each instance creates its own set of
    signals via Signature.create().

    Subclasses should set the class attribute ``signature`` to a
    ``Signature(...)`` object, or override ``__init_signature__`` to
    build one dynamically.

    Provides a ``.connect()`` method that reproduces the old Record.connect()
    semantics: Out members are copied from self→other, In members from other→self.
    """

    signature = None  # Override in subclass or pass to __init__

    def __init__(self, signature=None):
        sig = signature or self.__class__.signature
        if sig is None:
            raise TypeError(f"{self.__class__.__name__} has no signature defined")
        self._signature = sig
        self._intf = sig.create()
        self._field_names = list(sig.members.keys())
        for name in self._field_names:
            setattr(self, name, getattr(self._intf, name))

    def connect(self, other):
        """Connect fields from self→other (fanout) / other→self (fanin).

        Returns a list of combinational assignment statements, matching
        the old ``Record.connect()`` behaviour.

        Out members: other.field ← self.field
        In  members: self.field  ← other.field
        """
        stmts = []
        for name, member in self._signature.members.items():
            src = getattr(self, name)
            dst = getattr(other, name)
            if member.flow == Flow.Out:
                stmts.append(dst.eq(src))
            else:  # In
                stmts.append(src.eq(dst))
        return stmts

    def eq(self, other):
        """Assign every field from *other* into *self* (unconditional copy)."""
        stmts = []
        for name in self._field_names:
            stmts.append(getattr(self, name).eq(getattr(other, name)))
        return stmts

    def __getitem__(self, key):
        """Support record[field_name] access for compatibility."""
        return getattr(self, key)


class Encoder(Elaboratable):
    """Priority encoder replacement for amaranth.lib.coding.Encoder.

    Given a one-hot input, produces the index of the active bit.
    If no bits (or multiple bits) are set, the ``n`` (invalid) output is asserted.
    """

    def __init__(self, width):
        self.width = width
        self.i = Signal(width)
        self.o = Signal(range(width))
        self.n = Signal()

    def elaborate(self, platform):
        m = Module()

        with m.Switch(self.i):
            for j in range(self.width):
                with m.Case(1 << j):
                    m.d.comb += self.o.eq(j)
            with m.Default():
                m.d.comb += self.n.eq(1)

        return m
