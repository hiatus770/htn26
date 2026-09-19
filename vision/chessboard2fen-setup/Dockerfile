FROM python:3.7-slim-buster

# TensorFlow 2.2 is only compatible with older CPython releases.  This image
# keeps those libraries isolated from the host's Python installation.
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    TF_CPP_MIN_LOG_LEVEL=2

# Debian Buster reached end of life and its package indexes were archived.
RUN sed -i \
        -e 's|deb.debian.org/debian|archive.debian.org/debian|g' \
        -e '/buster-updates/d' \
        /etc/apt/sources.list \
    && apt-get -o Acquire::Check-Valid-Until=false update \
    && apt-get install -y --no-install-recommends \
        libcairo2 libglib2.0-0 libgl1 libsm6 libxext6 libxrender1 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN python -m pip install --no-cache-dir --upgrade 'pip<24' \
    && python -m pip install --no-cache-dir -r /tmp/requirements.txt

COPY smoke_test.py /opt/chessboard2fen-setup/smoke_test.py
WORKDIR /repo
