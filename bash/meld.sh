#!/bin/bash

set -e

GitDir=~/NetBeansProjects

S1=$GitDir/Libbulletjme/src/main/java
D1=$GitDir/Minie/MinieLibrary/src/main/java

S2=$GitDir/jmonkeyengine/jme3-examples/src/main/java
D2=$GitDir/Minie/Jme3Examples/src/main/java

/usr/bin/meld --diff $S1 $D1 --diff $S2 $D2
