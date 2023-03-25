#!/bin/bash

set -e

export JAVA_HOME=/usr/lib/jvm/Temurin/jdk-17.0.6+10

V=7.4.0
D=/home/sgold/Releases/Minie/$V
mkdir $D

./gradlew clean release
cp MinieLibrary/build/libs/*.{jar,module,pom} $D
# Note: the -javadoc.jar and -sources.jar are copied just once!

./gradlew clean release -Pbare
cp MinieLibrary/build/libs/*bare.{jar,module,pom} $D

./gradlew clean release -Pbig3
cp MinieLibrary/build/libs/*big3.{jar,module,pom} $D

./gradlew clean release -Pbtdebug
cp MinieLibrary/build/libs/*debug.{jar,module,pom} $D

./gradlew clean release -Pbtdebug -Pdp
cp MinieLibrary/build/libs/*debugdp.{jar,module,pom} $D

./gradlew clean release -Pdp
cp MinieLibrary/build/libs/*dp.{jar,module,pom} $D

./gradlew clean release -Pdroid
cp MinieLibrary/build/libs/*droid.{jar,module,pom} $D

./gradlew clean release -Pmt
cp MinieLibrary/build/libs/*mt.{jar,module,pom} $D

chmod 444 $D/*
chmod 555 $D
