import serial
test_string = 'hello world'
port_list = ['/dev/ttyS0']
for port in port_list:
    try:
        serialPort = serial.Serial(port, 115200, timeout=1)
        print('serial port: {0}'.format(port))
        bytes_sent = serialPort.write(test_string)
        print('bytes sent: {0} '.format(bytes_sent))
        loopback = serialPort.read(bytes_sent)
        if loopback == test_string:
            print('succeeded, received the following message: {0}'.format(loopback))
        else:
            print('failed')
    except IOError:
        print('io error, port ', port)

