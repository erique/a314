FROM dtcooper/raspberrypi-os:bookworm

RUN apt-get update && apt-get install -y build-essential libraspberrypi-dev raspberrypi-kernel-headers

