from ctypes import WinError
from sys import argv
from threading import Thread
import socket as sockets
import typing
import time
from utils import (
    Signals,
    StatusCode,
    cvt_size,
    get_size
)


class ComChannel(Thread):

    def __init__(self, name: str, address: typing.Tuple[str, int],
                 connectCallback=None,
                 dataTunnel=None,
                 errorCallback=None,
                 disconnectCallback=None):
        # Setup channel thread
        super().__init__(name=name, target=self.__startup_operation)
        # Set runtime variables
        self.name = name
        self.address = address
        self.isOpened = False
        self.reconnect_timeout = 2
        # Setup callbacks
        self.connectCallback = connectCallback
        self.dataTunnel = dataTunnel
        self.errorCallback = errorCallback
        self.disconnectCallback = disconnectCallback
        # Setup channel socket
        self.socket = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)
        self.socket.settimeout(3)

    def __startup_operation(self):
        # Check if channel is closed
        while not self.isOpened:
            # Open the channel
            try:
                # Connect to Robot
                self.socket.connect(self.address)
                # Update runtime
                self.isOpened = True
                break
            # except TimeoutError as e:
            #     print(f'Timed out')
                time.sleep(self.reconnect_timeout)
            except ConnectionRefusedError as e:
                print(f'Robot has refused the connection. MSG[{e}]')
                time.sleep(self.reconnect_timeout)
            finally:
                # Fire the connection callback
                if self.isOpened and self.connectCallback:
                    self.connectCallback(self)

    def open(self):
        # Start the closed channel
        if not self.isOpened:
            self.start()

    def close(self):
        # Check if channel is opened
        if self.isOpened:
            # Close the channel socket
            try:
                self.socket.close()
                # Join the thread
                self.join(5)
                # Fire the disconnect callback
                if self.disconnectCallback:
                    self.disconnectCallback(self)
                # Update runtime
                self.isOpened = False
                # TODO Reinitialize the channel runtime
            except Exception as e:
                print('Can\'t close channel.')
                self.isOpened = False

    def send(self, data: bytes) -> int:
        if self.isOpened and data:
            # Try to send given data
            try:
                return self.socket.send(data)
            except Exception as error:
                # Fire error callback
                if self.errorCallback:
                    self.errorCallback(error)
        # Return 0 as socket hasn't sent any data
        return 0

    def receive(self, bfSize: int) -> int:
        try:
            packet = self.socket.recv(bfSize)
            # Check if packet is empty or contains close signal
            if not packet:
                return -1
            if packet.decode('utf-8') == Signals.SIGNAL_BYE:
                return -1
            # Pass packet through data tunnel
            if self.dataTunnel:
                self.dataTunnel(packet)
            return 1
        except TimeoutError:
            return 0


def connectCallback(channel: 'ComChannel'):
    print(f'Channel [{channel.name}] is connected.')
    # Receive data from server
    while channel.isOpened:
        # Receive data packet with given buffer size
        resuleCode = channel.receive(2048)
        if resuleCode == -1:
            print('Channel closed successfully')
            break


def dataTunnel(data: bytes):
    print(f'Received data of length: {len(data)}')


if __name__ == '__main__':
    channel = ComChannel('test', ('localhost', 2001), connectCallback, dataTunnel, None)
    channel.open()
