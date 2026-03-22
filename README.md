# vesc-can-cpp-driver

C++ interface for controlling and monitoring VESC 6 motor controllers over Linux SocketCAN.

## Overview

This package provides a complete implementation of the VESC 6 CAN command and telemetry protocol, as described in the [official VESC CAN documentation](https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf). It allows you to control and monitor one or more VESC controllers on a shared CAN bus directly from c++ code.

## Features

- Complete VESC 6 CAN frame parsing and construction
- Support for multiple VESCs on a single bus (identified by VESC ID)

## License

See [LICENSE](LICENSE) for terms.
