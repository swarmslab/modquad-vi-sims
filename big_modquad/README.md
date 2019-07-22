Before getting started, make sure to add the following script to your MAVs' .bashrc:

```
#!/bin/bash
# Mini script that returns the last digits of the robot's IP address
# Make sure to place these lines in .bashrc

OUTPUT="$(hostname -I | sed -r 's!/.*!!; s!.*\.!!' | tr -d '[:space:]')"
export IP_DIGITS="${OUTPUT}"
```
