
# Decawave ranging demo using DWM3001C with Zephyr

Perform double sided two way ranging both as initiator and responder.
It also listens to others's DS TWR to get the single-sided TWR from the other modules.

**Example:**

In a setup with 3 modules: Alice, Bob and Dave.
When Alice initiate a double-sided TWR with Bob, Bob compute the DS TWR, while Alice compute the single-sided TWR.
Dave listen passively and compute the SS TWR between Alice and Bob.


## Getting started

Create a Zephyr workspace directory:

```
mkdir zephyrWorkspace
cd zephyrWorkspace
```

Create a python venv, and install `west`:

```
python3 -m venv zephEnv
source zephEnv/bin/activate
pip install west
```

Get this repository:

```
west init -m https://github.com/Fabien-B/zephyr_decawave_ranging.git
cd app
```
Get dependencies:

```
west update
```

Install the other required python modules:

```
pip install -r requirements.txt
```

Install the compilation toolchain:

```
west sdk install --toolchains arm-zephyr-eabi
```

Build:

```
west build -b decawave_dwm3001c ./
```

Flash:

```
west flash -r jlink
west flash -r blackmagicprobe
```

Note: If you are using a JLink probe, install the J-Link software first: [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/).


