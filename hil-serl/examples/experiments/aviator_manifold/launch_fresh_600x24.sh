#!/usr/bin/env bash
# Compatibility entry point for the 600-trajectory, 24-env run.
exec bash "$(dirname "$0")/launch_fresh_rlpd.sh" "$@"
