#!/bin/bash

set -e

V=4.6.1
D=/home/sgold/Releases/Minie/$V
mkdir $D

./gradlew clean release
cp MinieLibrary/build/libs/*.{jar,pom} $D

./gradlew clean release -Pbare
cp MinieLibrary/build/libs/*bare.{jar,pom} $D

./gradlew clean release -Pbig3
cp MinieLibrary/build/libs/*big3.{jar,pom} $D

./gradlew clean release -Pbtdebug
cp MinieLibrary/build/libs/*debug.{jar,pom} $D

./gradlew clean release -Pbtdebug -Pdp
cp MinieLibrary/build/libs/*debugdp.{jar,pom} $D

./gradlew clean release -Pdp
cp MinieLibrary/build/libs/*dp.{jar,pom} $D

./gradlew clean release -Pmt
cp MinieLibrary/build/libs/*mt.{jar,pom} $D

chmod 444 $D/*
chmod 555 $D
