#!/bin/python3

import serial
import threading

class Keithley616:

    ID_STRING = 'K616_USB_IF'   # Interface ID string
    SENSITIVITY_STRINGS = ['AUTO', '0.01', '0.1', '1', '10', '100']

    def __init__(self, serial_port, verbose=False):
        
        self._verbose = verbose

        # Open serial port
        if self._verbose: print('Opening port: ' + serial_port)
        self._port = serial.Serial(port=serial_port, baudrate=115200, timeout=0.25, write_timeout=0.1)

        # Try to get interface ID string (clear buffer, send 'i' command and read response line)
        if self._verbose: print('Sending ID request')
        self._port.reset_input_buffer()
        self._port.write(b'i')
        id_resp = self._port.readline().decode().strip()

        # Check interface ID string        
        if self._verbose: print('Checking ID response ... ', end='')
        if self.ID_STRING in id_resp:
            # Set the interface to continuously output in compact format and get first reading
            if self._verbose: print('OK')
            self._port.write(b'Su')
            self.read()
        else:
            # If ID check fails, error out by raising an exception
            if self._verbose: print('Error')
            raise Exception('Interface ID check failure')

    def read(self):
        # Clear input buffer and get new line
        self._port.reset_input_buffer()
        self._get_new_data()
        return self.value

    def start_auto_update(self, callback=None):
        # Check if callback parameter is valid
        if not callback==None and not callable(callback):
            raise TypeError('Callback parameter is not callable')
        # Create and start auto update thread
        if self._verbose: print('Starting auto update thread')
        self._stop_event = threading.Event()
        self._update_thread = threading.Thread(target=self._update_task, args=(callback,))
        self.autoupdate_is_alive = self._update_thread.is_alive
        self._update_thread.start()

    def stop_auto_update(self):
        self._stop_event.set()

    def set_range(self, range):
        if type(range)==int:
            if range>=0 and range<=5:
                if self._verbose: print('Set sensitivity range: ' + str(range))
                self._port.write(('r'+str(range)).encode('utf-8'))
            else:
                raise ValueError('Range parameter must be an integer between 0 and 5')
        else:
            raise TypeError('Range parameter must be an integer value')

    def set_zero(self, zero):
        if type(zero)==bool:
            if self._verbose: print('Set remote zero check: ' + str(zero))
            self._port.write(b'Z' if zero else b'z')
        else:
            raise TypeError('Zero check parameter must be a boolean')

    def set_hold(self, hold):
        if type(hold)==bool:
            if self._verbose: print('Set display hold: ' + str(hold))
            self._port.write(b'H' if hold else b'h')
        else:
            raise TypeError('Display hold parameter must be a boolean')

    def _get_new_data(self):
        # Read new line from serial port
        rx_line = self._port.readline().decode()
        # Check if a full line has been received
        if '\n' in rx_line:
            rx_line = rx_line.strip().split(' ')
            # Extract current measurement value and units
            self.value = float(rx_line[0])
            self.uint = rx_line[1]
            # Process instrument state indicator characters
            self.overflow = True if 'O' in rx_line[2] else False
            self.zero_check = True if 'Z' in rx_line[2] else False
            self.remote_zero = True if 'R' in rx_line[2] else False
            self.manual_sens = True if 'M' in rx_line[2] else False
            self.disp_hold = True if 'H' in rx_line[2] else False
            self.count_error = True if 'E' in rx_line[2] else False
            # Extract sensitivity number
            self.sensitivity = 0
            for c in '12345':
                if c in rx_line[2]: self.sensitivity = int(c)
        else:
            # readline() probably timed out before receiving a full line
            pass
    
    def _update_task(self, cb):
        # Purge old data from serial input buffer
        self._port.reset_input_buffer()
        # Run untill stop event is set
        while not self._stop_event.is_set():
            self._get_new_data()
            # If set, call callback funnction
            if callable(cb): cb()
        
        if self._verbose: print('Auto update task exiting')

    def __del__(self):
        try:
            self._stop_event.set()
            if self._verbose: print('Closing port ... ', end='')
            self._port.close()
            if self._verbose: print('closed')
        except:
            pass


if __name__=="__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Simple command line utility for K616 USB interface')

    parser.add_argument('portname', help='Serial port connecting to the K616 interface', default='/dev/ttyACM0')
    pg_zero = parser.add_mutually_exclusive_group()
    pg_zero.add_argument('-z', '--zero', help='Enable remote zero check', action='store_true')
    pg_zero.add_argument('-n', '--normal', help='Switch to normal mode (disable zero check)', action='store_true')
    pg_range = parser.add_mutually_exclusive_group()
    pg_range.add_argument('-a', '--auto', help='Enable automatic sesnitivity range selection', action='store_true')
    pg_range.add_argument('-r', '--range', help='Manual sensitivity range selection', type=int, choices=[0,1,2,3,4,5])
    pg_hold = parser.add_mutually_exclusive_group()
    pg_hold.add_argument('--hold-on', help='Enable instrument display hold', action='store_true')
    pg_hold.add_argument('--hold-off', help='Disable instrument display hold', action='store_true')
    parser.add_argument('-s', '--status', help='Print instrument status', action='store_true')
    parser.add_argument('-v', '--verbose', help='Print additional informations on program execution', default=False, action='store_true')
    parser.add_argument('-t', '--stream', help='Stram measurement values (continuously print new measurements when available)', action='store_true')

    args = parser.parse_args()

    # Initialize instrument object (connect to instrument interface)
    instrument = Keithley616(serial_port=args.portname, verbose=args.verbose)
    # Process optionall controll arguments
    if args.zero: instrument.set_zero(True)
    if args.normal: instrument.set_zero(False)
    if args.auto: instrument.set_range(0)
    if args.range: instrument.set_range(args.range)
    if args.hold_on: instrument.set_hold(True)
    if args.hold_off: instrument.set_hold(False)

    # Measurement printing function (also used as callback function in auto updating mode)
    def print_measurement():
        print(str(instrument.value)+' ' + str(instrument.uint), end='')
        print((' (OVERFLOW)' if instrument.overflow else (' (COUNT ERROR)' if instrument.count_error else '')))

    # Read latest measurement and display value and apropriate units
    instrument.read()

    # Instrument status printout
    if args.status:
        print('INSTRUMENT STATUS')
        print('      Measurement: ' + str(instrument.value)+' ' + str(instrument.uint), end='')
        print((' (OVERFLOW)' if instrument.overflow else (' (COUNT ERROR)' if instrument.count_error else '')))
        print('       Zero check: ' + ('ON' if instrument.zero_check else 'OFF') + (' (REMOTE)' if instrument.remote_zero else ''))
        print('Sensitivity range: ' + str(instrument.sensitivity) + (' MANUAL' if instrument.manual_sens else ' AUTO'))
        print('     Display Hold: ' + ('ON' if instrument.disp_hold else 'OFF'))
    else:
        if args.stream:
            instrument.start_auto_update(print_measurement)
            try:
                while instrument.autoupdate_is_alive():
                    pass
            except KeyboardInterrupt:
                instrument.stop_auto_update()
                if args.verbose: print('Keyboard interrupt received, exiting')
        else:
            print_measurement()
