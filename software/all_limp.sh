#!/bin/bash
#
# Release torque on every servo, one at a time.
#
# Leg 3 was missing from this list until 2026-10-12, so the emergency
# stop left a whole leg clamped and loaded.
#
# utils/all_limp.py does the same job in one pass on one open port, and
# verifies each servo actually released. Prefer it; this is kept because
# it needs nothing but go_limp.py.
#
python3 utils/go_limp.py 11
python3 utils/go_limp.py 12
python3 utils/go_limp.py 13

python3 utils/go_limp.py 21
python3 utils/go_limp.py 22
python3 utils/go_limp.py 23

python3 utils/go_limp.py 31
python3 utils/go_limp.py 32
python3 utils/go_limp.py 33

python3 utils/go_limp.py 41
python3 utils/go_limp.py 42
python3 utils/go_limp.py 43

python3 utils/go_limp.py 51
python3 utils/go_limp.py 52
python3 utils/go_limp.py 53

python3 utils/go_limp.py 61
python3 utils/go_limp.py 62
python3 utils/go_limp.py 63
