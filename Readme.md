
# Init

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
west init -m git@github.com:lmapii/practical-zephyr-t2-empty-ws.git
cd app
```

Install the other required python modules:

```
pip install -r requirements.txt
```

Install the compilation toolchain:

```
west sdk install --toolchains arm-zephyr-eabi
```

Get dependencies:

```
west update
```

Build:

```
west build -b decawave_dwm3001cdk ./
```

Flash:

```
west flash -r jlink
west flash -r blackmagicprobe
```
