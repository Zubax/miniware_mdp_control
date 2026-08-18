# Miniware MDP-M01 Python control

[![PyPI](https://img.shields.io/pypi/v/miniware-mdp-control)](https://pypi.org/project/miniware-mdp-control)

Control paired Miniware MDP-P906 power supplies through a USB-connected MDP-M01.

```sh
python3 -m pip install miniware-mdp-control
```

## Command line

```sh
mdp-control status
mdp-control ch1 9V 750mA on
mdp-control ch1 off
mdp-control ch1 9V 750mA on ch2 5V 1A on ch3 off
```

Actions are case-insensitive and may appear in any order within a channel clause. A command containing `on` may
energize connected hardware.

## Python API

```python
from mdp_control import ChannelCommand, MDPController

with MDPController() as mdp:
    status = mdp.apply(ChannelCommand(1, voltage=9.0, current=0.75, output=True))
    print(status.channels[0])
```

The complete documentation is maintained in the `mdp_control` module docstring. View it from the command line or
through Python's documentation tool:

```sh
mdp-control --help
python3 -m pydoc mdp_control
```
