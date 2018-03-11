# INS-Board Firmware

This is a minimalistic autopilot targeted for fixed wing gliders.
It is still work in progress and not usable for now.

## Hardware Targets

 - [INS-Board](https://github.com/stapelzeiger/ins-board)

more targets are planned

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
