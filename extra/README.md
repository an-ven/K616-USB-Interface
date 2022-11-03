## keithley616.py

A simple command line utility and python library to read and parse measurement data from the interface board.

    usage: keithley616.py [-h] [-z | -n] [-a | -r {0,1,2,3,4,5}] [--hold-on | --hold-off] [-s] [-v] [-t] portname

    Simple command line utility for K616 USB interface

    positional arguments:
    portname              Serial port connecting to the K616 interface

    options:
    -h, --help            show this help message and exit
    -z, --zero            Enable remote zero check
    -n, --normal          Switch to normal mode (disable zero check)
    -a, --auto            Enable automatic sesnitivity range selection
    -r {0,1,2,3,4,5}, --range {0,1,2,3,4,5}
                            Manual sensitivity range selection
    --hold-on             Enable instrument display hold
    --hold-off            Disable instrument display hold
    -s, --status          Print instrument status
    -v, --verbose         Print additional informations on program execution
    -t, --stream          Stram measurement values (continuously print new measurements when available)

## k616log.py

A small example logger utility that makes use of the `keithley616.py` script as a library.

    usage: k616log.py [-h] [-p PERIOD] [-f FILE] portname

    Simple logger utility for K616 USB Interface board. By default, if there are no filename and sampling period options specified, the output file name will be based on date and time of
    starting this utility, while the log output rate will be equal to the instrument sampling rate

    positional arguments:
    portname              Serial port connecting to the K616 interface

    options:
    -h, --help            show this help message and exit
    -p PERIOD, --period PERIOD
                            Time period in seconds between log entries
    -f FILE, --file FILE  Output log file name