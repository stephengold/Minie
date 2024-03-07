#!/bin/bash

set -e

GitDir=~/NetBeansProjects
#GitDir="/c/Users/sgold/My Documents/NetBeansProjects"

S1="$GitDir/Libbulletjme/src/main/java"
D1="$GitDir/Minie/MinieLibrary/src/main/java"

S2="$GitDir/jmonkeyengine/jme3-examples/src/main/java"
D2="$GitDir/Minie/Jme3Examples/src/main/java"

S3="$GitDir/LbjExamples/apps/src/main/java/com/github/stephengold/lbjexamples/apps"
D3="$GitDir/Minie/TutorialApps/src/main/java/jme3utilities/tutorial"

S4="$GitDir/LbjExamples/docs/en/modules/ROOT"
D4="$GitDir/Minie/MinieLibrary/src/site/antora/tutorials/modules/minie-library-tutorials"

S5="$GitDir/LbjExamples/docs/en/modules/ROOT"
D5="$GitDir/Minie/src/site/antora/minie-project/modules/ROOT"

S6="$GitDir/Acorus/docs/en"
D6="$GitDir/Minie/src/site/antora/minie-project"

S7="$GitDir/Minie-site-it"
D7="$GitDir/Minie"

S8="$GitDir/LbjExamples/apps/src/main/java"
D8="$GitDir/Minie/MinieExamples/src/main/java"

Meld="/usr/bin/meld"
#Meld="/c/Program Files/Meld/meld"

"$Meld" --diff "$S1" "$D1" --diff "$S2" "$D2" --diff "$S3" "$D3" --diff "$S4" "$D4" \
        --diff "$S5" "$D5" --diff "$S6" "$D6" --diff "$S7" "$D7" --diff "$S8" "$D8"
