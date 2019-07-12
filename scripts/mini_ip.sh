#!/bin/bash
# Mini script that returns the last digits of the robot's IP address
# Make sure to place these lines in .bashrc

OUTPUT="$(ip -o addr show dev "wlp2s0" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')"
export IP_DIGITS="${OUTPUT}"
