#!/usr/bin/env bash
# Pre build script that connects generated C code with
# user written C++ code. Project is configured to run
# this script automatically before each build.

FILEPATH=Core/Src/main.c
INCLUDE='#include "user-code\/main_cpp.h"'
MAINCALL='MainCpp();'

# Add call to MainCpp() in main.c if it's missing
if grep -q ${MAINCALL} ${FILEPATH}
then
  :
else
  sed -i "/USER CODE BEGIN WHILE/s/$/\n  ${MAINCALL}/" ${FILEPATH}
fi

# Add include of main_cpp.h in main.c if it's missing
if grep -q "${INCLUDE}" ${FILEPATH}
then
  :
else
  sed -i "/USER CODE BEGIN Includes/s/$/\n${INCLUDE}/" ${FILEPATH}
fi
