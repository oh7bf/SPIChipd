# SPIChipd
Read chips with Serial Peripheral Interface on Raspberry Pi.
Chips working with some issues: MAX31865.

Compile code with *make*.

Test the code with

```bash
./spichipd 2> /dev/null
```

The standard error is used for systemd-journald logging.

To create class documentation run *doxygen* without options on command line.

