#!/bin/bash

set -e

export JAVA_HOME=/usr/lib/jvm/Temurin/jdk-17.0.4+8

V=7.3.0+for36
D=/home/sgold/Releases/Minie/$V
mkdir $D

./gradlew clean release
cp MinieLibrary/build/libs/*.{jar,module,pom} $D
# Note: the -javadoc.jar and -sources.jar are copied just once!

./gradlew clean release -Pbare
cp MinieLibrary/build/libs/*bare+for36.{jar,module,pom} $D

./gradlew clean release -Pbig3
cp MinieLibrary/build/libs/*big3+for36.{jar,module,pom} $D

./gradlew clean release -Pbtdebug
cp MinieLibrary/build/libs/*debug+for36.{jar,module,pom} $D

./gradlew clean release -Pbtdebug -Pdp
cp MinieLibrary/build/libs/*debugdp+for36.{jar,module,pom} $D

./gradlew clean release -Pdp
cp MinieLibrary/build/libs/*dp+for36.{jar,module,pom} $D

./gradlew clean release -Pdroid
cp MinieLibrary/build/libs/*droid+for36.{jar,module,pom} $D

./gradlew clean release -Pmt
cp MinieLibrary/build/libs/*mt+for36.{jar,module,pom} $D

chmod 444 $D/*
chmod 555 $D
