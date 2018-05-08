# INS-Board Firmware

This is a minimalistic autopilot targeted for fixed wing gliders.
_It is still work in progress and not usable for now._


## Cloning the Repository

```
git clone https://github.com/Stapelzeiger/INS-board-fw.git
git submodule update --init --recursive
```

## Hardware Targets

### INS-Board

A compact board with IMU, SD-card and lots of connectors. [See Repo](https://github.com/stapelzeiger/ins-board)

to build, run:

```
cd target/INS-board-v1
make # run multiple times until it finds everything, there is an issue with the Makefile
```


## unit-tests

To build and run all test:
```
mkdir -p build/tests
cd build/tests
cmake ../..
make check
```

## Python setup

It is recommended that you install the required python packages in a virtualenv:
```
virtualenv --python=python3 env
```
And load it (this is shell specific):
```
source env/bin/activate.fish
```

After making a change in a python module, you should update the installation:
```
pip install -r requirements.txt --upgrade
```
