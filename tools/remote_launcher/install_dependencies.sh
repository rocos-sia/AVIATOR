#!/usr/bin/env bash
# Run once on a new Ubuntu control computer. Its sudo password may be different.
set -euo pipefail
sudo apt-get update
sudo apt-get install -y --no-install-recommends python3 python3-tk openssh-client rsync xauth
