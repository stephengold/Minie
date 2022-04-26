#!/bin/bash

set -e

GitDir=~/NetBeansProjects

S1=$GitDir/Libbulletjme/src/main/java
D1=$GitDir/Minie/MinieLibrary/src/main/java

S2=$GitDir/jmonkeyengine/jme3-examples/src/main/java
D2=$GitDir/Minie/Jme3Examples/src/main/java

S3=$GitDir/LbjExamples/apps/src/main/java/com/github/stephengold/lbjexamples/apps
D3=$GitDir/Minie/TutorialApps/src/main/java/jme3utilities/tutorial

/usr/bin/meld --diff $S1 $D1 --diff $S2 $D2 --diff $S3 $D3
