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
    def __init__(self, callback):
        self.HEADER_SIZE = 3
        self.HEADER_BYTE_H = 0X32
        self.HEADER_BYTE_L = 0X54
        self.BAUDERATE =  115200
        self.PORT_NAME = '/dev/ttyS0'
        self.MAX_MESSAGES_TO_PROCESS = 10
        self.port = serial.Serial(self.PORT_NAME, self.BAUDERATE, timeout=0)
        if not self.port.is_open:
            self.port.open()
        self.rxState = WAIT_HEAD_BYTE_H
        self.rxCallback = callback

    def listen(self):
        remaining = self.port.in_waiting
        if remaining > 4000:
            rospy.logerr('serial node has >4000 bytes buffered')
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
            if self.rxState == WAIT_BODY:
                if self.port.in_waiting < self.messageLength + 2:
                    return
                messageBody = self.port.read(self.messageLength)
                checksum = struct.unpack('<H', self.port.read(2))[0]
                checksumExpected = crc.crc16(self.generateInfo(self.messageLength, self.messageType) + messageBody)
                if checksumExpected != checksum:
                    rospy.logerr('checksum failed, got {0} expected {1}'.format(checksum, checksumExpected))
                else:
                    self.rxCallback(self.messageType, messageBody) 
                messageCount += 1
                self.rxState = WAIT_HEAD_BYTE_H
            else:
                rospy.logerr('invalid state')

    def readBytes(self, numBytes):
        return self.port.read(numBytes)

    def generateHeader(self):
        return struct.pack('<BB', self.HEADER_BYTE_H, self.HEADER_BYTE_L)

    def generateInfo(self, length, msgType):
        return struct.pack('<HB', length, msgType)

    def sendMessage(self, message, msgType):
        header = self.generateHeader()
        info = self.generateInfo(len(message), msgType)
        txMsg = header + info + message
        checksum = crc.crc16(unicode(info + message))
        txMsg = txMsg + struct.pack('<H', checksum)
        bytes_sent = self.port.write(txMsg)


def handleHelloWorld(type, msg):
    print('received message: type={0}, msg={1}'.format(type, msg))

if __name__ == '__main__':
    s = SerialSocket(handleHelloWorld)
    c = 0
    while True:
        c += 1
        if c % 10 == 1:
            s.sendMessage("hello world", 2)
        s.listen()
        time.sleep(0.01)
