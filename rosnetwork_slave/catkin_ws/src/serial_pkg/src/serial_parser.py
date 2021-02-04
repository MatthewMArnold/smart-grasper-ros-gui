#!/usr/bin/env python

import time
import crc
import serial
import struct

WAIT_HEAD_BYTE_H = 0
WAIT_HEAD_BYTE_L = 1
WAIT_HEADER = 2
WAIT_BODY = 3

class SerialSocket:
    def __init__(self, rxcallback, errorcallback=None, portName='/dev/ttyS0'):
        """
        Initializes the socket

        Args:
            - rxcallback (function pointer): Callback called each time the
              parser receives a message
            - errorcallback (function pointer, optional): Callback called if a
              parsing error occurs. Defaults to None.
            - portName (str, optional): The hardware port you would like to
              connect the socket to. Defaults to '/dev/ttyS0'.
        """
        self.HEADER_SIZE    = 3
        self.HEADER_BYTE_H  = 0X32
        self.HEADER_BYTE_L  = 0X54
        self.BAUDERATE      = 115200
        self.PORT_NAME      = portName
        self.MAX_BUFFERED_BYTES = 4000  # The max number of bytes that can be in
                                        # the serial buffer before an error is
                                        # logged
        self.MAX_MESSAGES_TO_PROCESS = 10
        self.port = serial.Serial(self.PORT_NAME, self.BAUDERATE, timeout=0)
        if not self.port.is_open:
            self.port.open()
        self.rxState = WAIT_HEAD_BYTE_H
        self.rxCallback = rxcallback
        self.errorCallback = errorcallback

    def listen(self):
        """
        Call repeatedly to parse messages being received on the serial port
        associated with the socket. This socket is meant to run in a ROS node.
        """
        remaining = self.port.in_waiting
        if remaining > self.MAX_BUFFERED_BYTES:
            rospy.logerr('serial node has >{0} bytes buffered'.format(self.MAX_BUFFERED_BYTES))
        # Only process so many messages each time listen is called
        messageCount = 0
        while messageCount < self.MAX_MESSAGES_TO_PROCESS:
            if self.rxState == WAIT_HEAD_BYTE_H:
                if self.port.in_waiting <= 0:
                    return

                byte = struct.unpack('B', self.port.read())[0]
                if byte == self.HEADER_BYTE_H:
                    self.rxState = WAIT_HEAD_BYTE_L

            if self.rxState == WAIT_HEAD_BYTE_L:
                if self.port.in_waiting <= 0:
                    return

                byte = struct.unpack('B', self.port.read())[0]
                if byte == self.HEADER_BYTE_L:
                    self.rxState = WAIT_HEADER
                else:
                    self.rxState = WAIT_HEAD_BYTE_H
                    rospy.logerr('invalid head byte l received, expected {0} received {1}'.format(self.HEADER_BYTE_L, byte))
                    if self.errorCallback is not None:
                        self.errorCallback()

            if self.rxState == WAIT_HEADER:
                if self.port.in_waiting < self.HEADER_SIZE:
                    return

                self.header = struct.unpack('HB', self.port.read(self.HEADER_SIZE))
                if len(self.header) == 2:
                    self.messageLength = self.header[0]
                    self.messageType = self.header[1]
                    self.rxState = WAIT_BODY
                else:
                    rospy.logerr('invalid header length {0}'.format(len(self.header)))
                    self.rxState = WAIT_HEAD_BYTE_H
                    if self.errorCallback is not None:
                        self.errorCallback()

            if self.rxState == WAIT_BODY:
                if self.port.in_waiting < self.messageLength + 2:
                    return
                messageBody = self.port.read(self.messageLength)

                checksum = struct.unpack('<H', self.port.read(2))[0]
                checksumExpected = crc.crc16(self.__generateInfo(self.messageLength, self.messageType) + messageBody)
                if checksumExpected != checksum:
                    rospy.logerr('checksum failed, got {0} expected {1}'.format(checksum, checksumExpected))
                    if self.errorCallback is not None:
                        self.errorCallback()
                else:
                    self.rxCallback(self.messageType, messageBody) 
                messageCount += 1
                self.rxState = WAIT_HEAD_BYTE_H

            else:
                rospy.logerr('invalid state')

    def __generateHeader(self):
        """
        Packages the header in preperation for sending.

        Returns:
            byte array: The packaged header array
        """
        return struct.pack('<BB', self.HEADER_BYTE_H, self.HEADER_BYTE_L)

    def __generateInfo(self, length, msgType):
        """
        Packages the length and msgType into an appropriate format for sending via serial.

        Args:
            - length (int): The length of the message being sent
            - msgType (int): The message type associated with the message

        Returns:
            byte array: Packaged data, in the form of an array
        """
        return struct.pack('<HB', length, msgType)

    def sendMessage(self, message, msgType):
        """
        Packages and sends the specified message.

        Args:
            - message (byte array): The message body
            - msgType (int): The message type associated with the message
        """
        header = self.__generateHeader()
        info = self.__generateInfo(len(message), msgType)
        txMsg = header + info + message
        checksum = crc.crc16(unicode(info + message))
        txMsg = txMsg + struct.pack('<H', checksum)
        bytes_sent = self.port.write(txMsg)


def handleHelloWorld(type, msg):
    print('received message: type={0}, msg={1}'.format(type, msg))

if __name__ == '__main__':
    """
    Test program to run the serial parser in a standalone environment.
    """
    s = SerialSocket(handleHelloWorld)
    c = 0
    while True:
        c += 1
        if c % 10 == 1:
            s.sendMessage("hello world", 2)
        s.listen()
        time.sleep(0.01)
