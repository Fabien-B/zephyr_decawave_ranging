# UART and USB serial commands

This application exposes the same binary protocol and Zephyr shell on the physical
UART and on the USB CDC ACM virtual serial port. Both ports start in binary mode
and keep independent modes, parsers and shell sessions. For example, the USB shell
can be used while the physical UART continues exchanging binary frames.

Command responses return on the port that received the command, including deferred
ranging results. By default, unsolicited binary events follow the most recent
command port: receiving a binary command on USB suppresses UART reports, and a
subsequent command on UART transfers reports to UART. The selected port must be
in binary mode. Before the first command, events go to both binary ports.
Application text goes to the active shells. Radio settings and monitor resources
remain shared between sessions.

Use the shell command `uart reports` to change routing at runtime:

```text
uart reports active    # Follow the latest binary-command port (default).
uart reports all       # Broadcast to both binary ports.
uart reports status    # Show the current policy; `uart reports` also works.
```

The setting is shared by both ports and resets to `active` after reboot. Shell
commands do not select a report port; return to binary mode with `uart binary`
and send a binary command to select that port. Commands received while the policy
is `all` still update the latest port, used if you switch back to `active`.
Selection happens for a complete,
nonempty frame with a valid checksum, before dispatch; unsupported commands also
select their port. Partial frames, checksum errors, empty frames and shell input
do not change report routing. Ranging, data and TDOA reports all use this policy.
Already queued USB reports or a transmission in progress may still finish on the
previous port. Disconnecting USB does not automatically transfer routing to UART;
send a binary command on UART to select it. Reboot clears the selection.

## USB connection

Connect the board's native USB data port and open its virtual serial device
(typically `/dev/ttyACM0` on Linux or a COM port on Windows). The host must assert
DTR, as pyserial does by default. The serial baud rate does not affect USB speed.
The device starts normally with no USB host connected.

The existing `PdecUart` Python client can use the USB device path unchanged.
Press Enter twice within one second to enter the USB shell, and use `uart binary`
to return that port to binary mode. These commands also remain available on UART.

USB transmission is buffered and nonblocking. If the host stops reading and the
buffer fills, new binary frames are dropped whole; they are never partly queued.
Output while DTR is deasserted is discarded.

## RX-to-TX timing instrumentation

Enable measurements on the initiator and on each responder using its shell:

```text
ranging timing reset
ranging timing on
uart binary
```

Run the host ranging test, stop it and close its serial connection. Enter the
shell again (two Enters within one second), then inspect the accumulated data:

```text
ranging timing off
ranging timing status
```

`ranging timing` also displays status. Instrumentation is off after reboot;
`reset` clears counters without changing the enabled setting. Both paths are
reported separately: RESPONSE is POLL reception to RESPONSE request on a
responder; FINAL is RESPONSE reception to FINAL request on the initiator.

Each path reports attempts, calls reaching `dwt_starttx()` (`samples`), failed
starts, deadlines already past before setup (`expired`), and buffer-write
failures. Per-operation min/average/max elapsed times identify where time is
spent:

- RX callback to TX function entry, including frame processing and token generation.
- TX validation and the existing `SYS_TIME` read.
- `forcetrxoff` and status clear when needed, TX buffer write and frame-control programming when changed.
- Delayed-TX time programming.
- `dwt_starttx()`, including its internal status checks.

**Instrumentation adds zero SPI transactions.** Zephyr's high-resolution timing
API uses the nRF52833's free-running TIMER2 at 16 MHz, which continues through SPI
and thread waits. Each stage boundary captures that local timer. Conversions and
statistics updates occur after TX arming, or after an early failure. No per-frame
printing occurs. The timer is initialized once during radio startup and remains
running; measurements are off after reboot.

The RX timestamp and the single `SYS_TIME` read already required by TX setup
provide the DW-clock anchor. The local timer brackets that existing read and
`dwt_starttx()`, producing estimated lower/upper bounds on RX-to-TX-command time
and remaining TX lead. These bounds account for uncertainty in where the DW
clock was sampled within the read and where the fast command occurs inside the
vendor driver. They include a 2-us rounding allowance; small cross-clock drift
is not calibrated. The vendor driver remains unmodified. These are bounds on
command delivery, not RF transmission timestamps. All output is in real us,
independent of the protocol's configurable delay conversion constant.

The two sides also report **observed peer turnaround**, reusing timestamps
already read by ranging: the ranger measures POLL TX to RESPONSE RX; the rangee
measures RESPONSE TX to FINAL RX. These on-air intervals include propagation in
both directions and use one device clock, so the devices need no clock
synchronization. They should be close to the peer's configured reply delay at
short distances. They describe received replies only, and cannot identify a
missing peer transmission. RESPONSE preparation is measured locally on the
rangee; FINAL preparation is measured locally on the ranger. Collect both outputs
for a full exchange, keeping delay, distance and traffic settings identical.

`n` is the number of measurements contributing to each metric. Early failures
only contribute to stages completed before failure. The peer-turnaround count
may differ from the local attempt count, since it measures the other device.
`off` retains statistics; `reset` clears them. An attempt already in progress
may complete after either command.

Compare TX lead bounds with the approximately 158-us advance requirement for
128-symbol preamble/8-symbol SFD. If the bounds cross that threshold, the exact
command delivery instant remains uncertain. The per-operation timings show which
setup operations to optimize before trying shorter reply delays.

RX-after-TX delay and hardware frame-wait timeout are fixed at zero during
initialization/recovery, rather than rewritten for each frame. Manual
single-buffer RX is explicitly configured: a successful RX leaves the receiver
stopped, and the driver acknowledges status before the application callback.
Replies from that callback skip redundant TRX-off and status cleanup. Queued TX
from LISTENING and error recovery retain them. Frame-control settings are cached
by length and ranging flag (offset is always zero), invalidated on recovery.
Skipped operations may therefore show zero in the timing output. Frame length
comes from the driver's callback metadata, avoiding another RX_FINFO read.

## Binary protocol

Binary frames use this envelope:

```text
0xff 0xff <len> <payload...> <checksum>
```

`len` is the number of payload bytes. The checksum is the XOR of `len` and every
payload byte. Multi-byte integer fields are little-endian.

The first payload byte is the command, response, or event ID.

### Binary commands

| ID | Payload | Description |
| --- | --- | --- |
| `0x01` | `<dst_id:u16>` | Range once with a target device. |
| `0x02` | `<dst_id:u16>` | Get the last known distance to a target device. |
| `0x03` | none | List known device IDs. |
| `0x04` | `<dst_id:u16> <data...>` | Send a data payload to a target device. |
| `0x05` | `<enabled:u8>` | Set outgoing discovery: `0` off, `1` on. |

For example, `ranging range 0x4567` is encoded as payload bytes:

```text
01 67 45
```

### Binary responses

| ID | Payload | Description |
| --- | --- | --- |
| `0x7f` | `<command:u8> <status:u8>` | Error response. |
| `0x81` | `<dst_id:u16> <status:u8> <distance:f32>` | Range result. |
| `0x82` | `<dst_id:u16> <status:u8> <distance:f32>` | Last known distance result. |
| `0x83` | `<count:u8> <device_id:u16>...` | Known device list. |
| `0x84` | `<dst_id:u16> <status:u8> <sent_len:u8>` | Data-send result. |
| `0x85` | `<status:u8> <enabled:u8>` | Discovery setting after command `0x05`. |

Discovery control affects the connected module only, using the same volatile
setting as `ranging discovery off|on`. It suppresses periodic and queued outgoing
HELLOs; an emission already started may finish. Incoming discovery is still
processed. Reboot restores discovery to on. Invalid lengths return error `0x02`;
enabled values other than `0` or `1` return generic error `0x01`, without changing
the setting. Replies return only on the requesting serial port.

Complete disable frame: `ff ff 02 05 00 07`. Successful reply:
`ff ff 03 85 00 00 86`. Enable frame: `ff ff 02 05 01 06`.

With the port in binary mode and no other application using it:

```python
from pdec_uart import PdecUart

with PdecUart("/dev/ttyACM0") as device:
    device.set_discovery(False)  # Returns False after acknowledgement.
    # device.set_discovery(True) re-enables outgoing discovery.
```

The Python client also accepts `device.command("discovery off")` and
`device.command("discovery on")`.

Status values:

| Value | Meaning |
| --- | --- |
| `0x00` | OK |
| `0x01` | Generic error |
| `0x02` | Invalid payload length |
| `0x03` | Distance not found |
| `0x04` | Range failed |
| `0x05` | Message too long |

Distances are 32-bit floating-point values in meters.

### Binary ranging events

Whenever a ranging result is produced, the device sends an unsolicited event
frame according to the report routing policy above:

| ID | Payload |
| --- | --- |
| `0x90` | `<kind:u8> <src_id:u16> <dst_id:u16> <distance:f32>` |

Event kinds:

| Value | Meaning |
| --- | --- |
| `0x00` | Single-sided ranging |
| `0x01` | Double-sided ranging |
| `0x02` | Indirect/passive ranging |

## Entering shell mode

To switch from binary mode to the shell, send two newline characters within
1 second. In a serial terminal, this usually means pressing Enter twice.

Once the shell is active, the prompt is:

```text
pdec>
```

To return to binary mode:

```text
uart binary
```

The device prints `Binary mode enabled` after switching back.

## Shell ranging commands

Device IDs are 16-bit values. Hexadecimal notation is accepted, for example
`0x4567`. Distances are reported in meters.

### Single range

Perform a ranging exchange with one target device:

```text
ranging range 0x4567
```

Example successful output:

```text
0x4567: 1.23m
```

Example failure output:

```text
Ranging with 0x4567 failed !
```

### Last known distance

Print the last stored distance to a device:

```text
ranging get 0x4567
```

Example output:

```text
Last distance to 0x4567: 1.23m
```

If no distance is known:

```text
No distance known for 0x4567
```

### Known devices

List the devices currently known by the local database:

```text
ranging list
```

Example output:

```text
Known devices:
 [0] 0x1234 (me)
 [1] 0x4567
```

### Monitor ranging results

Monitor ranging events involving this device:

```text
ranging monitor
```

This is equivalent to:

```text
ranging monitor local
```

Monitor all ranging events heard by this device:

```text
ranging monitor all
```

Monitor output uses the event type, the two device IDs, and the distance in
meters:

```text
[DS] 0x1234 <-> 0x4567: 1.23m
```

Press Enter to stop monitoring and return to the shell prompt.

## UART commands

Send an ASCII data message to another device:

```text
uart send 0x4567 hello world
```

Example output:

```text
Sent 11 byte(s) to 0x4567
```

Return the UART protocol to binary mode:

```text
uart binary
```

## Discovery transmissions during ranging tests

Each module transmits a discovery HELLO every two seconds by default. From the
shell of each module, inspect or change that module's setting:

```text
ranging discovery status
ranging discovery off
ranging discovery on
```

Disabling affects outgoing discovery only. Received frames still update the
local database. Discoveries queued before `off` are discarded when dispatched;
a command already being dispatched is allowed to finish. The setting is volatile and
returns to `on` at reboot. Enabling resumes periodic transmissions.
