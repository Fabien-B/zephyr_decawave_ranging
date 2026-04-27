# Zephyr Shell Commands

This application exposes a Zephyr shell over the UART protocol. The UART starts
in binary mode by default.

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

## Ranging commands

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

## Continuous ranging commands

Continuous ranging periodically ranges one or more target devices.

Start continuous ranging:

```text
ranging continuous start
```

Stop continuous ranging:

```text
ranging continuous stop
```

Set the full-cycle period in milliseconds:

```text
ranging continuous freq 500
```

With multiple targets, the period is spread across all targets in the active
target list.

Add a target to the custom target list:

```text
ranging continuous add 0x4567
```

Remove a target from the custom target list:

```text
ranging continuous remove 0x4567
```

Reset the target list back to all known devices except this device:

```text
ranging continuous reset
```

Show continuous ranging status and targets:

```text
ranging continuous list
```

Example output:

```text
Continuous ranging: running
Mode: all
Full-cycle period: 500 ms
Targets (1):
 0x4567
```

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
