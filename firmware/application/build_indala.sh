#!/bin/bash
export PATH="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin:/c/ProgramData/chocolatey/bin:$PATH"
cd /c/Users/Animeboynz/Documents/GitHub/ChameleonUltra/firmware/application
make -j
echo EXIT:$?
