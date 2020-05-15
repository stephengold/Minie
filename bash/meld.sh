#!/bin/bash

set -e

S1=/home/sgold/Git/Libbulletjme/src/main/java
D1=/home/sgold/Git/Minie/MinieLibrary/src/main/java

S2=/home/sgold/Git/jmonkeyengine/jme3-examples/src/main/java
D2=/home/sgold/Git/Minie/Jme3Examples/src/main/java

/usr/bin/meld --diff $S1 $D1 --diff $S2 $D2
