#!/usr/bin/env bash
# Install the DQN / machine-learning dependencies inside the container.
# The image build installs these best-effort; run this if it was skipped
# (e.g. a flaky network) or to refresh them.
set -euo pipefail

pip3 install --break-system-packages --timeout 1000 --retries 20 \
  --index-url https://download.pytorch.org/whl/cpu torch

pip3 install --break-system-packages --timeout 1000 --retries 20 \
  gymnasium stable-baselines3 transforms3d

echo ">> ML dependencies installed."
