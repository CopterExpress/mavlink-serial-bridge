# mavlink-serial-bridge

**mavlink-serial-bridge** is a **MAVLink** messages forwarder from the **serial device** (**UART**, **RS-232**, some **USB devices**, etc.) to **UDP** packets and vice versa.

## Installation

Requirments:

- `libcyaml`

```console
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

## Configuration

Create a copy of the `example.yaml` from `/etc/mavlink-serial-bridge/`, save it in the same directory and edit, according to your requirements.

## Start the application

Run `sudo systemctl start mavlink-serial-bridge@<configuration>` to start the tool, where **configuration** is the configuration file name (without the file extension).

Run `sudo systemctl enable mavlink-serial-bridge@<configuration>` to start the tool, where **configuration** is the configuration file name (without the file extension).
