#!/usr/bin/python3

import logging
logging.basicConfig(format='%(levelname)s, %(asctime)s, %(name)s, line %(lineno)d: %(message)s')

from a314d import A314d

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

SERVICE_NAME = 'echo'

class EchoService(object):
    def __init__(self):
        self.a314d = A314d(SERVICE_NAME)


    def run(self):
        logger.info('ECHO service is running')

if __name__ == '__main__':
    service = EchoService()
    service.run()
