#!/usr/bin/python3

import logging
logging.basicConfig(format='%(levelname)s, %(asctime)s, %(name)s, line %(lineno)d: %(message)s')

import struct
import select

from a314d import A314d

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

SERVICE_NAME = 'echo'

class EchoService(object):
    def __init__(self):
        self.a314d = A314d(SERVICE_NAME)
        self.rbuf = b''
        self.current_stream_id = None

    def process_stream_data(self, stream_id: int, data: bytes):
        logger.info('stream data from ' + stream_id + ' bytes ' + data.len)

    def process_a314d_msg(self, stream_id, ptype, payload):
        if ptype == self.a314d.MSG_CONNECT:
            if payload == SERVICE_NAME.encode() and self.current_stream_id is None:
                logger.info('Amiga connected to "' + SERVICE_NAME + '" service')
                self.current_stream_id = stream_id
                self.a314d.send_connect_response(stream_id, 0)
            else:
                self.a314d.send_connect_response(stream_id, 3)
        elif self.current_stream_id == stream_id:
            if ptype == self.a314d.MSG_DATA:
                self.process_stream_data(stream_id, payload)
                pass
            elif ptype == self.a314d.MSG_EOS:
                raise AssertionError('EOS is not supposed to be in use')
                pass
            elif ptype == self.a314d.MSG_RESET:
                self.current_stream_id = None
                logger.info('Amiga disconnected from "' + SERVICE_NAME + '" service')

    def handle_a314d_readable(self):
        buf = self.a314d.drv.recv(1024)
        if not buf:
            logger.error('Connection to a314d was closed, shutting down')
            exit(-1)

        self.rbuf += buf
        while True:
            if len(self.rbuf) < 9:
                break

            (plen, stream_id, ptype) = struct.unpack('=IIB', self.rbuf[:9])
            if len(self.rbuf) < 9 + plen:
                break

            self.rbuf = self.rbuf[9:]
            payload = self.rbuf[:plen]
            self.rbuf = self.rbuf[plen:]

            self.process_a314d_msg(stream_id, ptype, payload)

    def run(self):
        logger.info('ECHO service is running')

        while True:
            try:
                rl = [self.a314d]
                rl, _, _ = select.select(rl, [], [], 10.0)
            except KeyboardInterrupt:
                break

            for s in rl:
                if s == self.a314d:
                    self.handle_a314d_readable()

        self.a314d.close()

        logger.info('ECHO service stopped')

if __name__ == '__main__':
    service = EchoService()
    service.run()
