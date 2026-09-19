# Select an NVIDIA TensorFlow image that matches the Jetson's installed
# JetPack/L4T release. Do not use the x86 image used for legacy verification.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    TF_CPP_MIN_LOG_LEVEL=2

WORKDIR /app
COPY requirements-jetson.txt .
RUN python3 -m pip install --no-cache-dir -r requirements-jetson.txt

COPY app/ /app/
ENTRYPOINT ["python3", "run_images.py"]
