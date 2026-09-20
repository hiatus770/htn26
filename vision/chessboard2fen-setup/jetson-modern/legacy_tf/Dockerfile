# Select an NVIDIA TensorFlow image that matches the Jetson's installed
# JetPack/L4T release. Do not use the x86 image used for legacy verification.
ARG BASE_IMAGE=nvcr.io/nvidia/tensorflow:25.02-tf2-py3-igpu
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    TF_CPP_MIN_LOG_LEVEL=2 \
    TF_USE_LEGACY_KERAS=1

WORKDIR /app
COPY requirements-jetson.txt .
RUN python3 -m pip install --no-cache-dir -r requirements-jetson.txt

COPY app/ /app/app/
ENTRYPOINT ["python3"]
