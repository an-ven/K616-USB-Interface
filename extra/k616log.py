#!/bin/python3

import sys
import time
import argparse
import keithley616

DEFAULT_FILENAME=time.strftime('K616_%Y-%m-%dT%H-%M-%S.csv')

parser = argparse.ArgumentParser(
    description='''Simple logger utility for K616 USB Interface board. By default, if there are no filename
    and sampling period options specified, the output file name will be based on date and time of starting
    this utility, while the log output rate will be equal to the instrument sampling rate'''    
)

parser.add_argument('-p', '--period', help='Time period in seconds between log entries', type=float, default=0)
parser.add_argument('-f', '--file', help='Output log file name', type=str, default=DEFAULT_FILENAME)
parser.add_argument('portname', help='Serial port connecting to the K616 interface', default='/dev/ttyACM0')

args = parser.parse_args()

log_period = args.period

# Check if period parameter is a sensible value
if not log_period==0 and log_period<0.2:
    print('Instrument sampling rate is about 0.2sec, specified period is too short.')
    sys.exit(0)

# Instantiate instrument object and connect to the K616 USB interface
isntrument = keithley616.Keithley616(args.portname)

print('Writing to log file: ' + args.file)

with open(args.file, 'a') as output_file:
    
    start_time = time.time()

    # Log output function (auto update callback function )
    def write_output():
        # Generate log entry string with following format: "<timestamp>, <value>, <units>, <status>\n"
        status = 'Overflow' if isntrument.overflow else ('Zero Check' if isntrument.zero_check else 'Normal')
        log_entry = f'{time.time()-start_time:8.2f}, {isntrument.value:9.3e}, {isntrument.uint}, {status}\n'
        print(log_entry, end='')
        output_file.write(log_entry)

    if args.period:
        # If log period is specified, start while loop with specified period
        try:
            while True:
                sample_time = time.time()
                isntrument.read()
                write_output()
                if time.time()-sample_time>log_period:
                    print('Can\'t write fast enough')
                else:
                    time.sleep(log_period-(time.time()-sample_time))
        except KeyboardInterrupt:
            print('Keyboard interrupt received, exiting.')
    
    else:
        # Start k616 library auto update thread and pass write_output() as callback function
        isntrument.start_auto_update(callback=write_output)
        try:
            # Loop while the auto update thread is alive
            while isntrument.autoupdate_is_alive():
                pass
        except KeyboardInterrupt:
            print('Keyboard interrupt received, stopping autoupdate thread.')
            isntrument.stop_auto_update()
            while isntrument.autoupdate_is_alive(): pass
            print('Exiting.')
