#!/usr/bin/env python
#
# Redirect data from a TCP/IP connection to a serial port and vice versa.
#
# (C) 2002-2016 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause

import sys
import socket
import serial
import serial.threaded
import time

class OppoSerialToNet(serial.threaded.Packetizer):
    """
    Read and write (Unicode) lines from/to OPPO Player serial port.
    """

    TERMINATOR = b'\r'
    
    custom_cmd = ''
    query_cmd = ''
    power_state_checking = False

    def __call__(self):
        return self

    def handle_packet(self, packet):
        
        if (packet != '@OK'):
            sys.stderr.write('handle packet %s (%s)\n' % (packet, ','.join([str(hex(c)) for c in packet])))
            
        if self.custom_cmd != '':
            if not self.power_state_checking:
                self.handle_custom_cmd(packet)
            else:
                self.check_custom_power_cmd_result(self.custom_cmd,packet)
        else:
            if self.socket is not None:
                self.socket.sendall(packet + '\n')

    def write_serial(self, text):
        """
        Write text to the transport and the carrage return (CR) is append.
        """

        text = text.strip('\r\n').strip('\r').strip('\n').strip()
        if len(text) > 1:        
            sys.stderr.write('write serial %s, %d\n' % (text,ord(text[-1])))
        # handle custome command: need to check oppo current status, 
        # then send the suitable command in class function handle_custom_cmd
        if text in ['#POWON', '#POWOFF']:
            # custom command: POWON and POWOFF
            # POWON : make sure oppo will be ture on
            # POWOFF : make user oppo will be turn off
            self.custom_cmd = text
            self.query_cmd = '#QPW'
            self.transport.write(self.query_cmd + self.TERMINATOR)
        else:
            self.custom_cmd = ''
            self.query_cmd = ''
            # + is not the best choice but bytes does not support % or .format in py3 and we want a single write call
            self.transport.write(text + self.TERMINATOR)

    def set_verbose_mode_off(self):
        '''
        trun off the oppo verbose mode
        '''
        self.transport.write('SVM 0' + self.TERMINATOR)
        
    def handle_custom_cmd(self, packetText):
        if not 'OK' in packetText:
            # check query_cmd result is OK
            # re-send query_cmd if result is NOT OK
            self.transport.write(self.query_cmd + self.TERMINATOR)
        else:
            if self.custom_cmd in ['#POWON', '#POWOFF']:
                self.handle_custom_power_cmd(self.custom_cmd, packetText)
            else:
                sys.stderr.write('unknow custom cmd %s\n' % self.custom_cmd)
                self.reset_custom_cmd_meta()
        
    def handle_custom_power_cmd(self, pwdCmd, pwdState):
        # check oppo power state and send oppo power cmd if need
        if (pwdCmd == '#POWON' and pwdState.find('@OK OFF') >= 0) or \
            (pwdCmd == '#POWOFF' and pwdState.find('@OK ON') >= 0):
            self.transport.write('#POW' + self.TERMINATOR)
            self.power_state_checking = True
            time.sleep(3)
        else:
            if (pwdCmd == '#POWON' and pwdState.find('@OK ON') >= 0) or \
                (pwdCmd == '#POWOFF' and pwdState.find('@OK OFF') >= 0):
                # clean up variables for custom cmd handling
                sys.stderr.write('power state %s, skip sending power cmd %s\n' % (pwdState, pwdCmd))
                self.reset_custom_cmd_meta()
            else:
                sys.stderr.write('power state %s unknow, reset query cmd\n' % pwdState)
                self.transport.write(self.query_cmd + self.TERMINATOR)
        
    def check_custom_power_cmd_result(self, pwdCmd, pwdState):
        # check power cmd result and re-send if fail
        if (pwdCmd == '#POWON' and pwdState.find('@OK ON') >= 0) or \
            (pwdCmd == '#POWOFF' and pwdState.find('@OK OFF') >= 0):
            sys.stderr.write('power cmd %s finished\n' % pwdCmd)
            self.reset_custom_cmd_meta()
            if self.socket is not None:
                self.socket.sendall(pwdState + '\n')
        else:
            sys.stderr.write('power cmd %s retry\n' % pwdCmd)
            self.transport.write('#POW' + self.TERMINATOR)
            time.sleep(1)
    
    def reset_custom_cmd_meta(self):
        self.custom_cmd = ''
        self.query_cmd = ''
        self.power_state_checking = False
        
    def write_socket(self, data):
        self.socket.sendall(data + '\n')
        
    
class SerialToNet(serial.threaded.Protocol):
    """serial->socket"""

    def __init__(self):
        self.socket = None

    def __call__(self):
        return self

    def data_received(self, data):
        if self.socket is not None:
            self.socket.sendall(data)


if __name__ == '__main__':  # noqa
    import argparse

    parser = argparse.ArgumentParser(
        description='Simple Serial to Network (TCP/IP) redirector.',
        epilog="""\
NOTE: no security measures are implemented. Anyone can remotely connect
to this service over the network.

Only one connection at once is supported. When the connection is terminated
it waits for the next connect.
""")

    parser.add_argument(
        'SERIALPORT',
        help="serial port name")

    parser.add_argument(
        'BAUDRATE',
        type=int,
        nargs='?',
        help='set baud rate, default: %(default)s',
        default=9600)

    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='suppress non error messages',
        default=False)

    parser.add_argument(
        '--develop',
        action='store_true',
        help='Development mode, prints Python internals on errors',
        default=False)

    group = parser.add_argument_group('serial port')

    group.add_argument(
        "--parity",
        choices=['N', 'E', 'O', 'S', 'M'],
        type=lambda c: c.upper(),
        help="set parity, one of {N E O S M}, default: N",
        default='N')

    group.add_argument(
        '--rtscts',
        action='store_true',
        help='enable RTS/CTS flow control (default off)',
        default=False)

    group.add_argument(
        '--xonxoff',
        action='store_true',
        help='enable software flow control (default off)',
        default=False)

    group.add_argument(
        '--rts',
        type=int,
        help='set initial RTS line state (possible values: 0, 1)',
        default=None)

    group.add_argument(
        '--dtr',
        type=int,
        help='set initial DTR line state (possible values: 0, 1)',
        default=None)

    group = parser.add_argument_group('network settings')

    exclusive_group = group.add_mutually_exclusive_group()

    exclusive_group.add_argument(
        '-P', '--localport',
        type=int,
        help='local TCP port',
        default=7777)

    exclusive_group.add_argument(
        '-c', '--client',
        metavar='HOST:PORT',
        help='make the connection as a client, instead of running a server',
        default=False)

    args = parser.parse_args()

    # connect to serial port
    ser = serial.serial_for_url(args.SERIALPORT, do_not_open=True)
    ser.baudrate = args.BAUDRATE
    ser.parity = args.parity
    ser.rtscts = args.rtscts
    ser.xonxoff = args.xonxoff

    if args.rts is not None:
        ser.rts = args.rts

    if args.dtr is not None:
        ser.dtr = args.dtr

    if not args.quiet:
        sys.stderr.write(
            '--- TCP/IP to Serial redirect on {p.name}  {p.baudrate},{p.bytesize},{p.parity},{p.stopbits} ---\n'
            '--- type Ctrl-C / BREAK to quit\n'.format(p=ser))

    try:
        ser.open()
    except serial.SerialException as e:
        sys.stderr.write('Could not open serial port {}: {}\n'.format(ser.name, e))
        sys.exit(1)

    ser_to_net = OppoSerialToNet()
    serial_worker = serial.threaded.ReaderThread(ser, ser_to_net)
    serial_worker.start()

    if not args.client:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('', args.localport))
        srv.listen(1)
    try:
        intentional_exit = False
        while True:
            if args.client:
                host, port = args.client.split(':')
                sys.stderr.write("Opening connection to {}:{}...\n".format(host, port))
                client_socket = socket.socket()
                try:
                    client_socket.connect((host, int(port)))
                except socket.error as msg:
                    sys.stderr.write('WARNING: {}\n'.format(msg))
                    time.sleep(5)  # intentional delay on reconnection as client
                    continue
                sys.stderr.write('Connected\n')
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                #~ client_socket.settimeout(5)
            else:
                sys.stderr.write('Waiting for connection on {}...\n'.format(args.localport))
                client_socket, addr = srv.accept()
                sys.stderr.write('Connected by {}\n'.format(addr))
                # More quickly detect bad clients who quit without closing the
                # connection: After 1 second of idle, start sending TCP keep-alive
                # packets every 1 second. If 3 consecutive keep-alive packets
                # fail, assume the client is gone and close the connection.
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                ser_to_net.socket = client_socket
                # enter network <-> serial loop
                ser_to_net.set_verbose_mode_off()
                while True:
                    try:
                        data = client_socket.recv(1024)
                        if not data:
                            break
                        #ser.write(data)                 # get a bunch of bytes and send them
                        if ser_to_net.power_state_checking:
                            ser_to_net.write_socket('@BUSY')
                        else:
                            ser_to_net.write_serial(data)
                    except socket.error as msg:
                        if args.develop:
                            raise
                        sys.stderr.write('ERROR: {}\n'.format(msg))
                        # probably got disconnected
                        break
            except KeyboardInterrupt:
                intentional_exit = True
                raise
            except socket.error as msg:
                if args.develop:
                    raise
                sys.stderr.write('ERROR: {}\n'.format(msg))
            finally:
                ser_to_net.socket = None
                sys.stderr.write('Disconnected\n')
                client_socket.close()
                if args.client and not intentional_exit:
                    time.sleep(5)  # intentional delay on reconnection as client
    except KeyboardInterrupt:
        pass

    sys.stderr.write('\n--- exit ---\n')
    serial_worker.stop()
