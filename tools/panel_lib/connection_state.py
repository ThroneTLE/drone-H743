"""Host receive provenance; monotonic seconds, never target uptime or UI time.

Boundary: immutable queue messages and pure snapshot assembly state. No I/O/Tk.
The source object and connection generation together identify a device session.
"""

from dataclasses import dataclass, replace
import time


@dataclass(frozen=True)
class ReceiveContext:
    source: object
    generation: int
    received_at: float

    def is_current(self, transport):
        return (self.source is transport
                and self.generation == getattr(transport, "connection_generation", 0)
                and transport.is_connected)


@dataclass(frozen=True)
class ReceivedMessage:
    payload: object
    context: ReceiveContext
    diagnostic: bool = False


def receive_context(transport, *, received_at=None, generation=None):
    return ReceiveContext(
        transport,
        getattr(transport, "connection_generation", 0) if generation is None else generation,
        time.monotonic() if received_at is None else received_at,
    )


def snapshot_receipt(panel, sequence):
    """Start/merge only one sequence in one session; age uses oldest fragment.

    Direct line consumers (offline tools/tests) may omit a receive context. Live
    transport messages always carry one, installed by the queue dispatcher.
    """
    context = getattr(panel, "_rx_context", None)
    panel._snapshot_live = context is not None
    if context is None:
        context = receive_context(panel.transport)
    previous = getattr(panel, "_snapshot_receipt", None)
    changed_session = previous is not None and (
        previous.source is not context.source or previous.generation != context.generation
    )
    if changed_session:
        panel.validation_latest_sequence = None
        panel.validation_latest_timestamp_ms = None
    if changed_session or getattr(panel, "_snapshot_fragment_sequence", None) != sequence:
        panel.validation_latest_values = {"seq": str(sequence)}
        panel.validation_latest_host_time = 0.0
        panel.validation_latest_transport_generation = None
    elif previous is not None:
        context = replace(context, received_at=min(context.received_at, previous.received_at))
    panel._snapshot_fragment_sequence = sequence
    panel._snapshot_receipt = context
    return context


def snapshot_is_current(panel):
    if not getattr(panel, "_snapshot_live", False):
        return True  # Explicit offline/direct line consumers have no live queue.
    receipt = getattr(panel, "_snapshot_receipt", None)
    return receipt is not None and receipt.is_current(panel.transport)
