# UART and USB serial commands

This application exposes the same binary protocol and Zephyr shell on the physical
UART and on the USB CDC ACM virtual serial port. Both ports start in binary mode
and keep independent modes, parsers and shell sessions. For example, the USB shell
can be used while the physical UART continues exchanging binary frames.

Command responses return on the port that received the command, including deferred
ranging results. Unsolicited binary events go to each port currently in binary
mode; application text goes to the active shells. Radio settings and monitor
resources remain shared between sessions.

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

Binary monitor mode is always on. Whenever a ranging result is produced, the
device sends an unsolicited event frame on each port in binary mode:

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
