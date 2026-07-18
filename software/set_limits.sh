#!/bin/bash

echo "set limits on coxa servo to 20"
python set_limits.py 11 20
python set_limits.py 21 20
python set_limits.py 31 20
python set_limits.py 41 20
python set_limits.py 51 20
python set_limits.py 61 20

echo "set limits on femur servo to 45"
python set_limits.py 12 45
python set_limits.py 22 45
python set_limits.py 32 45
python set_limits.py 42 45
python set_limits.py 52 45
python set_limits.py 62 45

echo "set limits on tibia servo to 90"
python set_limits.py 13 90
python set_limits.py 23 90
python set_limits.py 33 90
python set_limits.py 43 90
python set_limits.py 53 90
python set_limits.py 63 90
