# SPDX-License-Identifier: Apache-2.0
FROM python:3.10-slim-bookworm
ENV PYTHONUNBUFFERED=1
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN python -m pip install "pip>=23,<26" "setuptools>=64,<80" "packaging>=24.2"
COPY src/plc_simulator /opt/plc-simulator
RUN python -m pip install --no-build-isolation /opt/plc-simulator
EXPOSE 1502
USER 65534:65534
CMD ["plc-simulator", "--host", "0.0.0.0", "--port", "1502", "--disable-ownership"]
