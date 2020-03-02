#!/bin/bash -x

# Copy physics-related jme3-examples source files
# from the SRC directory to the DST directory.

set -e

SRC=/home/sgold/Git/jmonkeyengine/jme3-examples/src/main/java/jme3test
DST=/home/sgold/Git/Minie/Jme3Examples/src/main/java/jme3test

cd $SRC

cp --no-clobber --recursive bullet $DST
rm --force \
   $DST/bullet/TestBoneRagdoll.java \
   $DST/bullet/TestIK.java \
   $DST/bullet/TestIssue931.java \
   $DST/bullet/TestIssue1004.java \
   $DST/bullet/TestRagdollCharacter.java

[ -d $DST/batching ] || mkdir $DST/batching
cp batching/TestBatchNodeTower.java $DST/batching

[ -d $DST/games ] || mkdir $DST/games
cp games/RollingTheMonkey.java games/WorldOfInception.java $DST/games

[ -d $DST/helloworld ] || mkdir $DST/helloworld
cp helloworld/HelloCollision.java \
   helloworld/HelloPhysics.java \
   helloworld/HelloTerrainCollision.java $DST/helloworld

[ -d $DST/terrain ] || mkdir $DST/terrain
cp terrain/TerrainFractalGridTest.java \
   terrain/TerrainGridAlphaMapTest.java \
   terrain/TerrainGridSerializationTest.java \
   terrain/TerrainGridTest.java \
   terrain/TerrainGridTileLoaderTest.java \
   terrain/TerrainTestCollision.java $DST/terrain

cd $DST/../../../..
git status --short
