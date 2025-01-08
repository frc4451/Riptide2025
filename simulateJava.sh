#!/bin/sh

# Source me!

# See: https://github.com/frc4451/frc-nix/blob/a544814e087bc86624a06d645684180e77a5c8a7/README.md#why-doesnt-my-simulation-gui-work
export HALSIM_EXTENSIONS="$PWD"/build/jni/release/libhalsim_gui.so
export JAVA_HOME="$HOME"/wpilib/2025/jdk/
