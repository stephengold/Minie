# Release log for the Minie library, DacWizard, MinieExamples, and VhacdTuner

## Version 7.4.0 released on 24 March 2023

+ Fixed bugs:
  + `BetterCharacterControl` oscillates while unducking near an overhang
  + `BetterCharacterController.isOnGround()` is unreliable
  + `TestIssue18Heightfield` application throws an `IllegalArgumentException`
  + excessive logging from the `TestIssue13` application
+ Implemented kinematic modes for `BetterCharacterControl`
  and `JoinedBodyControl`.
+ Deprecated 7 library methods for obtaining native IDs:
  + `CollisionShape.getObjectId()`
  + `CollisionSpace.getSpaceId()`
  + `PhysicsCollisionObject.getObjectId()`
  + `PhysicsJoint.getObjectId()`
  + `RigidBodyMotionState.getObjectId()`
  + `RotationalLimitMotor.getMotor()`
  + `TranslationalLimitMotor.getMotor()`
+ Based on:
  + v3.6.0-stable of JMonkeyEngine,
  + v3.0.0 of the jME-TTF library,
  + v8.3.2 of the Heart library,
  + v1.0.0 of the Acorus library,
  + v0.7.5 of the Wes library, and
  + v0.9.35 of the jme3-utilities-nifty library.

## Version 7.3.0+for36 released on 3 March 2023

+ Fixed bugs:
  + ignored collision objects may be garbage-collected prematurely
  + adding a `DynamicAnimControl` to a spatial resets its armature
  + `PhysicsCollisionObject.cloneIgnoreList()` throws a `NullPointerException`
  + `BetterCharacterControl.setPhysicsLocation()` never updates the field
  + tasks passed to `PhysicsSpace.enqueueOnThisThread()` never execute
  + `TestAttachDriver` example doesn't reset properly (JME issue 1975)
+ Added the `JointedBodyControl` class.
+ Added 7 methods to the library:
  + `CollisionSpace.isForceUpdateAllAabbs()`
  + `CollisionSpace.setForceUpdateAllAabbs()`
  + `DynamicAnimControl.blendToKinematicMode(KinematicSubmode, float, Transform)`
  + `DynamicAnimControl.saveCurrentPose()`
  + `DynamicAnimControl.setKinematicMode(KinematicSubmode)`
  + `PhysicsCollisionObject.listIgnoredPcos()`
  + `PhysicsCollisionObject.setIgnoreList(PhysicsCollisionObject[])`
+ Added the "reset" kinematic submode to `DynamicAnimControl`.
+ Publicized 3 library methods:
  + a constructor for the `IndexedMesh` class
  + `DacLinks.getTransformer()`
  + `RagUtils.findMainJoint()`
+ Deprecated 2 library methods:
  + `PhysicsCollisionObject.listIgnoredIds()`
  + `PhysicsCollisionObject.setIgnoreList(long[])`
+ Added some runtime checks and strengthened others.
+ Made improvements to the DacWizard application:
  + added the "torso" screen to select the torso's main bone
  + added posing capability to the "load" and "test" screens
  + fixed a bug that caused generation of syntactically incorrect Java code
  + fixed a bug where the wrong main bone was used to estimate ranges of motion
  + when generating Java code, provide a `configure()` method
  + generate practical classnames for Java code
+ Based on:
  + v3.6.0-beta3 of JMonkeyEngine,
  + v1.6.0 of the SimMath library,
  + v8.3.1+for36 of the Heart library,
  + v0.9.18+for36 of the Acorus library,
  + v0.7.3+for36 of the Wes library, and
  + v0.9.34+for36 of the jme3-utilities-nifty library.
+ Built using Gradle v8.0.2 .
+ Updated the native libraries to v18.1.0 of Libbulletjme.

## Version 7.2.0 released on 24 January 2023

+ Bugfix: `PhysicsCollisionObject.findInstance()`
  creates a weak global reference that can never be deleted
+ Bugfix:  `DynamicAnimControl.totalMass()` returns `NaN` even when
  the control is added to a `Spatial`
+ Bugfix: weak global references in `PhysicsCollisionObject` and `MultiBody`
  can never be deleted
+ Added 2 methods to the library:
  + `CollisionSpace.jniEnvId()`
  + `NativeLibrary.jniEnvId()`
+ Updated the native libraries to v17.5.4 of Libbulletjme.

## Version 7.1.0 released on 16 January 2023

+ Added the capability to generate cylinder shapes in a ragdoll.
+ Publicized the `RagUtils.makeRectangularSolid()` method.
+ Added `DynamicAnimControl` tunings for a Mixamo rig.
+ Added the capability to display angles in degrees or radians in DacWizard.
+ Updated `DacWizard` and `VhacdTuner` to use v0.9.33
  of the jme3-utilities-nifty library.

## Version 7.0.2 released on 2 January 2023

+ Bugfix:  `NullPointerException` in `rebuildRigidBody()` while de-serializing
  an old model
+ Bugfix:  DacWizard doesn't write rotation orders to Java source code

## Version 7.0.1 released on 1 January 2023

Bugfix:  `NullPointerException` in `PhysicsCollisionObject.readPcoProperties()`

## Version 7.0.0 released on 24 December 2022

+ API changes:
  + Privatized `PhysicsCollisionObject.getCollisionFlags()` (a native method)
  + Added the `static` qualifier to `PersistentManifolds.listPointIds()`
  + Renamed the public logger in the `ConvexShape` class to avoid conflict.
  + Added a 2nd argument to `PhysicsBody.cloneJoints()`
  + Corrected the return type of `CharacterController.jmeClone()`
  + Finalized 3 classes:
    + `NativeSoftBodyUtil`
    + `PhysicsRayTestResult`
    + `PhysicsSweepTestResult`

+ Bug fixes:
  + `DynamicAnimControl` may pass illegal arguments to `MyMath.slerp()`
  + assertion failure when `toString()` is invoked on a collision object
    or physics joint with no native object assigned
  + out-of-range exception upon re-entering DacWizard's "bones" screen
    with a different model
  + transforms are not updated for the `getCalculatedOriginA()` and
    `getCalculatedOriginB()` methods in the `New6Dof` class
  + `getPhysicsLocationDp()` and `getPhysicsRotationDp()` return incorrect
    values for a soft body
  + cloning bugs:
    + physics joints are cloned inaccurately
    + ignore lists are cloned inaccurately
    + cloning or rebuilding a collision object results in
      different collision flags
    + the feedback parameter of a `Constraint` isn't cloned
    + `DacLinks` incompletely cloned
    + `BoneLink.tmpMatrix` is shared between clones
  + serialization bugs:
    + `NullPointerException` in `PreComposer.read()`
    + `RigidBodyMotionState` is never serialized
    + the pivot offsets of single-ended constraints
      are de-serialized incorrectly
    + `PhysicsLink.density` is never saved or loaded
    + the feedback parameter of a `Constraint` is never saved or loaded
    + the `bindTransform` and `preComposer` fields of a `DacLinks`
      are never saved or loaded

+ Publicized the `PhysicsLink.velocity()` method.
+ Added 9 double-precision setters:
  + `CharacterController.warpDp()`
  + `MultiBodyCollider.setPhysicsLocationDp()`
  + `MultiBodyCollider.setPhysicsRotationDp(Matrix3d)`
  + `PhysicsCharacter.setPhysicsLocationDp()`
  + `PhysicsGhostObject.setPhysicsLocationDp()`
  + `PhysicsGhostObject.setPhysicsRotationDp()` (2 signatures)
  + `PhysicsRigidBody.setPhysicsRotationDp(Matrix3d)`
  + `PhysicsSoftBody.setPhysicsLocationDp()`
+ Added 8 other double-precision methods:
  + `CollisionShape.getScaleDp()`
  + `CollisionSpace.rayTestDp()`
  + `ManifoldPoints.getPositionWorldOnADp()`
  + `ManifoldPoints.getPositionWorldOnBDp()`
  + `PhysicsCollisionObject.getPhysicsRotationMatrixDp()`
  + `RigidBodyMotionState.getLocationDp()`
  + `RigidBodyMotionState.getOrientationMatrixDp()`
  + `RigidBodyMotionState.getOrientationQuaternionDp()`
+ Added 6 other methods:
  + `New6Dof.calculatedBasisA()`
  + `New6Dof.calculatedBasisB()`
  + `New6Dof.getRotationMatrix()`
  + `PhysicsCollisionObject.collisionFlags()`
  + `PhysicsDescriber.describeMatrix()`
  + `PhysicsDumper.dump(PhysicsJoint, String)`

+ Added `INFO`-level log messages
  to the `New6Dof` and `PhysicsRigidBody` classes.
+ Made incremental improvements to the `PhysicsDumper` output format.
+ Strengthened argument validation.
+ Updated the native libraries to v17.4.0 of Libbulletjme.
+ Updated `DacWizard` and `VhacdTuner` to use v0.9.32
  of the jme3-utilities-nifty library.
+ Built using Gradle v7.6 .

## Version 6.2.0 released on 13 November 2022

+ Added 3 methods:
  + `PhysicsSpace.isCcdWithStaticOnly()`
  + `PhysicsSpace.setCcdWithStaticOnly()`
  + `NativeLibrary.countClampedCcdMotions()`
+ Updated the native libraries to v17.0.0 of Libbulletjme.
+ Based on v0.9.17 of the Acorus library.

## Version 6.1.0 released on 1 October 2022

+ Bugfix:  `PreComposer` isn't properly cloned or serialized
+ Added library support for dynamic collision-shape splitting:
  + Added 8 public constructors:
    + `ChildCollisionShape(Vector3f, CollisionShape)`
    + `CompoundMesh(CompoundMesh)`
    + `CompoundMesh(CompoundMesh, Vector3f)`
    + `GImpactCollisionShape(CompoundMesh, Vector3f)`
    + `HullCollisionShape(Vector3f...)`
    + `IndexedMesh(FloatBuffer)`
    + `MeshCollisionShape(boolean, CompoundMesh)`
    + `MultiSphere(Vector3f[], float...)`
  + Added 17 other public methods:
    + `ChildCollisionShape.split()`
    + `CollisionShape.aabbCenter()`
    + `CollisionShape.canSplit()`
    + `CollisionShape.scaledVolume()`
    + `CollisionShape.toSplittableShape()`
    + `CompoundCollisionShape.connectivityMatrix()`
    + `CompoundCollisionShape.countGroups()`
    + `CompoundCollisionShape.split()`
    + `CompoundMesh.maxMin()`
    + `CompoundMesh.split()`
    + `ConvexShape.toHullShape()`
    + `GImpactCollisionShape.split()`
    + `HullCollisionShape.split()`
    + `IndexedMesh.copyTriangle()`
    + `IndexedMesh.maxMin()`
    + `IndexedMesh.split()`
    + `MeshCollisionShape.split()`
+ Other library enhancements:
   + Added the `createGImpactShape()` method
     to the `CollisionShapeFactory` class.
   + Added the `pairTest()` method to the `CollisionSpace` class.
   + Added the `countMeshTriangles()` to the `GImpactCollisionShape` class.
+ Added the `SplitDemo` and `SweepDemo` applications.
+ Enhanced the `ShapeGenerator` class to use diverse axes when generating
  capsule, cone, and cylinder shapes.
+ Added the "teapotGi" collision shape to the MinieAssets project.
+ Based on v8.2.0 of the Heart library.
+ Updated the native libraries to v16.3.0 of Libbulletjme.

## Version 6.0.1 released on 29 August 2022

+ Bugfix:  `DacLinks` never re-enables hardware skinning (performance issue)
+ VhacdTuner GUI tweaks:
  + Removed the "ACDMode" buttons.
  + Added buttons to toggle the "async" setting.
  + Added a button to quit ranking the newest test result.
+ Added glTF loading capability to DacWizard.

## Version 6.0.0 released on 25 August 2022

+ Protected the no-arg constructors of 6 abstract classes. (API changes)
+ Bugfix:  issue #30 (`NullPointerException` after removing
  `DynamicAnimControl` from a `Spatial`)
+ Bugfix:  I/O resources not safely closed in `VHACDParameters`.
+ Added V-HACD version 4 including the `Vhacd4`, `Vhacd4Parameters, `FillMode`,
  and `Vhacd4Hull` classes plus a new `HullCollisionShape` constructor and a
  new `CollisionShapeFactory` method.
+ Added the VhacdTuner sub-project.
+ Added the `toMap()` method to the `VHACDParameters` class.
+ Updated the native libraries to v16.1.0 of Libbulletjme.

## Version 5.1.0 released on 6 August 2022

+ Bugfix:  `PhysicsRigidBody.setInverseInertiaLocal()` and
  `PhysicsRigidBody.updateMassProps()` don't update the world inertia tensor
+ Bugfix:  when rebuilding a rigid body, many properties are lost
+ Added a `DynamicAnimControl` mechanism to report the completion
  of a blend-to-kinematic operation.
+ Added the `CompletionListener` interface.
+ Added the `RigidBodySnapshot` class.
+ Added the `setIgnoreList()` method to the `PhysicsCollisionObject` class.
+ Added a simpler constructor to `PhysicsSoftSpace`.
+ Publicized the `rebuildRigidBody()` method.
+ Updated the native libraries to v16.0.0 of Libbulletjme.
+ Built using Gradle v7.5.1 .

## Version 5.0.1 released on 2 August 2022

+ Bugfix:  static rigid body misbehaves after being assigned a positive mass
+ Bugfix:  `DynamicAnimControl` with `AnimComposer` exhibits glitches during
  blends to kinematic mode
+ Bugfix:  when rebuilding a rigid body, its ignore list is lost
+ Based on v8.1.0 of the Heart library.
+ Built using Gradle v7.5 .

## Version 5.0.0 released on 11 July 2022

+ Replaced the "ano" build with "droid" build that includes Java classes.
+ Changes to the library API:
  + Replaced the `DebugMeshNormals` enum with `MeshNormals` (from Heart).
  + Protected the no-arg constructors of the `DacConfiguration`
    and `DacLinks` classes.
  + Protected the public constructors of 3 classes:
    + `AbstractPhysicsDebugControl`
    + `IKController`
    + `SoftPhysicsJoint`
  + Qualified 17 utility classes as `final`.
  + Removed the `static` qualifier from the protected `createWireMaterial()`
    method of the `BulletDebugAppState` class.
  + Deleted the deprecated `setContractCalcArea3Points()`
    method of the `PhysicsCollisionEvent` class.

+ Library bugfixes:
  + `PhysicsCharacter.onGround()` is unreliable (stephengold/Libbulletjme#18)
  + `TorsoLink` continues writing the model transform
     after a blend to kinematic completes
  + `DynamicAnimControl` rebuilds the ragdoll for minute changes to bone scaling
  + `TorsoLink` plays some bone animations, even in dynamic mode
  + outdated constant values in `ConfigFlag`
  + `DynamicAnimControl` is still marked "ready"
    after removal from the `PhysicsSpace`
  + `DebugMeshCallback.maxDistance()` modifies the vertex list
  + `ConfigFlag.describe()` ignores 3 flags

+ Other library improvements:
  + Warn if the native library version differs from the expected version.
  + Throw an exception in `AbstractPhysicsControl.jmeClone()`
    if the control is added to a `PhysicsSpace`.
  + Add capability to specify the main bone of a ragdoll,
    which needn't be a root bone.
  + Added accessors for global deactivation settings:
    + `PhysicsBody.getDeactivationDeadline()`
    + `PhysicsBody.isDeactivationEnabled()`
    + `PhysicsBody.setDeactivationDeadline()`
    + `PhysicsBody.setDeactivationEnabled()`
  + Added debug visualizations of rigid-body angular velocities
    and soft-body wind velocities.
  + Added a "relative tolerance" parameter to `DynamicAnimControl`.
  + Reimplemented `BulletDebugAppState` using `BaseAppState`.
  + Added the `DeformableSpace` class that supports both multibodies
    and soft bodies.
  + Added `SDF_MDF` and `SDF_RDN` bitmasks to `ConfigFlag`.

+ Added the `Pachinko` and `Windlass` apps to MinieExamples.
+ Added the `HelloGhost` and `HelloWind` apps to TutorialApps.
+ Updated the project URL in the POM.
+ Added 10 "package-info.java" files.
+ Based on:
  + v8.0.0 of the Heart library,
  + v0.7.2 of the Wes library,
  + v0.9.16 of the Acorus library, and
  + v0.9.30 of the jme3-utilities-nifty library.
+ Updated the native libraries to v15.2.1 of Libbulletjme.
+ Added the "checkstyle" plugin to the build.

## Version 4.9.0 released on 2 May 2022

+ Eliminated the last dependency on JCenter!
+ Bugfix:  `IllegalArgumentException` caused by slerps in `TorsoLink`
+ Changed `CollisionShapeFactory` to skip meshes without triangles.
+ Added the `GearJoint` class.
+ Changes to the apps:
  + Bugfix:  rigid body becomes deactivated in `HelloContactReponse`
  + Bugfix:  `get(Limits.TextureAnisotropy)` returns `null` on some platforms
  + Renamed `TestDebugToPost` and moved it from MinieExamples to TutorialApps.
  + Added `TestGearJoint` to MinieExamples.
  + Added 3rd body to `HelloDeactivation` for a visual reference point.
  + Added monkey-head test to `TestSoftBodyControl`.
  + Added display-settings editors to `NewtonsCradle` and `RopeDemo`.
  + Enabled window resizing for `DacWizard` and all apps in MinieExamples.
+ Based on:
  + v3.5.2-stable of JMonkeyEngine,
  + v1.5.0 of the SimMath library,
  + v7.6.0 of the Heart library,
  + v0.9.15 of the Acorus library, and
  + v0.9.29 of the jme3-utilities-nifty library.
+ Built using Gradle v7.4.2 .
+ Updated the native libraries to v14.3.0 of Libbulletjme.

## Version 4.8.1 released on 29 March 2022

+ Bugfix:  issue #23 (access violations on 64-bit Windows)
+ Split off "TutorialApps" from the "MinieExamples" sub-project.
+ Based on:
  + v0.9.11 of the Acorus library and
  + v0.9.28 of the jme3-utilities-nifty library.
+ Updated the native libraries to v14.2.0 of Libbulletjme.

## Version 4.8.0 released on 20 March 2022

+ Added native libraries for the MacOSX_ARM64 platform,
  so that Minie applications can run on "Apple Silicon" Macs.
+ Created the "+ano" (Android natives only) build, for use in
  Android signed bundles (for distribution via Google Store).
+ Added the `contactErp()`, `jointErp()`, `setContactErp()`,
  and `setJointErp()` methods to the `SolverInfo` class,
  to give applications better control over joint elasticity.
+ Based on v3.5.1-stable of JMonkeyEngine.
+ Updated the native libraries to v14.1.0 of Libbulletjme.

## Version 4.7.1 released on 11 March 2022

+ Bugfix:  `PhysicsCollisionEvent.getNormalWorldOnB()` returns wrong value
+ Bugfix:  issue #20 (`btAssert` fails after `HingeJoint.setAngularOnly(true)`)
+ Added the `needsCollision()` method to the `CollisionSpace` class.
  This method can be overridden for dynamic collision filtering.
+ Added the `ContactListener` interface for immediate handling of rigid-body
  contacts.
+ Added the `ManifoldPoints` utility class to access the properties
  of a contact point without instantiating a `PhysicsCollisionEvent`.
+ Added the `PersistentManifolds` utility class to access the properties
   of a contact manifold.
+ Added 8 methods to the `PhysicsSpace` class:
  + `addContactListener(ContactListener)`
  + `countManifolds()`
  + `listManifolds()`
  + `onContactEnded(PhysicsCollisionObject, PhysicsCollisionObject, long)`
  + `onContactProcessed(PhysicsCollisionObject, PhysicsCollisionObject, long)`
  + `onContactStarted(PhysicsCollisionObject, PhysicsCollisionObject, long)`
  + `removeContactListener(ContactListener)`
  + `update(float, int, boolean, boolean, boolean)`
    to enable callbacks to specific `ContactListener` methods
+ Deprecated the `PhysicsCollisionEvent.setContactCalcArea3Points()` method
  in favor of the corresponding `ManifoldPoints` method.
+ Deleted placeholder files from class jars.
+ Added the `ConveyorDemo` and `JointElasticity` apps to MinieExamples.
+ Based on:
  + v7.4.1 of the Heart library,
  + v0.7.1 of the Wes library,
  + v0.9.10 of the Acorus library, and
  + v0.9.27 of the jme3-utilities-nifty library.
+ Built using Gradle v7.4.1 .
+ Updated the native libraries to v14.0.0 of Libbulletjme.

## Version 4.6.1 released on 24 January 2022

+ Restored support for the MacOSX32 platform.
+ Based on:
  + v3.5.0-stable of JMonkeyEngine,
  + v7.2.0 of the Heart library,
  + v0.6.8 of the Wes library,
  + v0.9.6 of the jme3-utilities-ui library, and
  + v0.9.24 of the jme3-utilities-nifty library.
+ Built using Gradle v7.3.3 .
+ Updated the native libraries to v12.7.1 of Libbulletjme, which
  include the fix for bullet3 issue 4117.

## Version 4.6.0 released on 4 December 2021

+ Bugfix: issue #19 (zero-thickness mesh shape crashes the JRE)
+ Dropped support for the MacOSX32 platform.
+ Based on v3.4.1-stable of JMonkeyEngine.
+ Changes to the examples:
  + Solved 2 null-pointer exceptions in Jme3Examples.
  + Added the `TestInsideTriangle` and `TestIssue19` apps to MinieExamples.
  + Split off the `PhysicsDemo` class from `AbstractDemo`.
  + Built Jme3Examples using Java 8 .
+ Added the `isInsideTriangle()` method to the `NativeLibrary` class.
+ Guarded some low-level logging code to improve efficiency.
+ Built using Gradle v7.3.1 .
+ Updated the native libraries to v12.6.0 of Libbulletjme, which includes
  Bullet updates through 13 November 2021.

## Version 4.5.0 released on 19 November 2021

+ Cached the methods that free native objects, to improve performance.
+ Added the `setPivotInB()` method to the `Anchor` class.
+ Added a mesh customization API to the `DebugShapeFactory` class,
  to support (for example) debug materials that need barycentric coordinates.
+ Publicized the `worldMax()` and `worldMin()` methods
  in the `BulletAppState` class.
+ Improvements to examples:
  + Solved `UnsatisfiedLinkError` crashes in 5 apps.
  + Added the `TestIssue18GImpact` app.
  + Addressed JME issue 1630 in `TestBetterCharacter`.
+ Updated the native libraries to v12.5.0 of Libbulletjme,
  which includes contact filtering for GImpact shapes.
+ Built using Gradle v7.3 .

## Version 4.5.0-test1 released on 25 October 2021

+ Bugfix: invalid contact points for heightfield/mesh shapes (issue #18)
+ Added a flag to disable contact filtering on a per-shape basis.
+ Added accessors to the `CollisionSpace` class for the collision dispatcher's
  "deterministic" option.
+ Made all native physics objects `Comparable` (for use in collections).
+ Improvements to examples:
  + Added 2 tutorial apps for character physics:
    `HelloWalkOtoBcc` and `HelloWalkOtoCc`.
  + Added 2 tests for issue #18:
    `TestIssue18Heightfield` and `TestIssue18Mesh`.
  + Made some tutorial apps more comparable.
  + Created a merged mesh shape for the table in `PoolDemo` (more efficient).
  + Reduced the camera zoom speed in `PoolDemo` (to avoid overshooting).
+ Updated the native libraries to v12.4.1 of Libbulletjme.

## Version 4.4.0 released on 1 October 2021

+ Added 10 double-precision accessors and created an API dependency
  on v1.4.1 of Paul's SimMath library.
+ Added more flexible customization of chassis axes in `VehicleController`.
+ Bugfix: incompatibility with Java v7
+ Added the `NegativeAppDataFilter` class (for obstructions).
+ Added the `listInternalJoints()` method to the `RagUtils` class.
+ Added the example app `HelloServo`.
+ Improved diagnostic messages in `DacWizard`.
+ Based on v7.1.0 of the Heart library and v0.9.23 of the
  jme3-utilities-nifty library.
+ Updated the native libraries to v12.2.2 of Libbulletjme, which includes
  Bullet updates through 20 September 2021.

## Version 4.3.0 released on 22 August 2021

+ Bugfix: contact tests report events with positive separation distance
+ Bugfix: `NullPointerException` while previewing an erroneous model during
  DacWizard step #2 (the "load" step)
+ Bugfix: pure virtual call in native libraries (issue #17)
+ Added support for Quickprof profiling of native code.
+ Added support for multithreaded physics spaces.
+ Added the `TowerPerformance` application for performance testing.
+ Tweaked the behavior of `PhysicsRigidBody.setKinematic()`.
+ Bypass the asset cache when loading models into DacWizard.
+ Updated the native libraries to v12.0.0 of Libbulletjme
  (includes performance enhancements).
+ Based on:
  + LWJGL v3 (to improve support for non-US keyboards),
  + v7.0.0 of the Heart library,
  + v0.6.7 of the Wes library,
  + v0.9.5 of the jme3-utilities-ui library, and
  + v0.9.22 of the jme3-utilities-nifty library.
+ Built using Gradle v7.2 .

## Version 4.2.0 released on 24 June 2021

+ Bugfix: Libbulletjme issue #7 (GImpact contact tests always fail)
+ Dump the bounds and ignore list of each `PhysicsGhostObject`.
+ Added `hasClosest()` and `hasContact()` methods
  to the `CollisionSpace` class.
+ Added a public `getShapeType()` method to the `CollisionShape` class.
+ Document the lack of collision detection between 3 concave shapes.
+ Use a migrated Jaime model in MinieExamples.
+ Updated the native libraries to v10.5.0 of Libbulletjme, which includes
  Bullet v2 updates through 12 May 2021.
+ Built using Gradle v7.1 .

## Version 4.1.1 released on 1 June 2021

+ Bugfix: issue #16 (MinieExamples uses deprecated classes)
+ Based on:
  + v3.4.0-stable of JMonkeyEngine,
  + v2.2.4 of the jme-ttf library,
  + v6.4.4 of the Heart library,
  + v0.6.6 of the Wes library,
  + v0.9.4 of the jme3-utilities-ui library, and
  + v0.9.20 of the jme3-utilities-nifty library.
+ Built using Gradle v7.0.2 .

## Version 4.1.0+for34 released on 25 April 2021

+ Bugfix: `RagUtils.ignoreCollisions()` doesn't handle cycles correctly
+ Bugfix: `NullPointerException` while reading a `DacLinks` from an XML file
+ Added `setPivotInA()` and `setPivotInB()` methods to `Point2PointJoint`.
+ Added `HelloDoor` application to the MinieExamples subproject.
+ Updated the native libraries to v10.3.1 of Libbulletjme, which includes
  Bullet v2 updates through 21 April 2021.
+ Based on:
  + v3.4.0-beta1 of JMonkeyEngine,
  + v6.4.3+for34 of the Heart library,
  + v0.6.3+for34 of the Wes library,
  + v0.9.3+for34 of the jme3-utilities-ui library, and
  + v0.9.19+for34 of the jme3-utilities-nifty library.
+ Built using Gradle v7.0 .

## Version 4.0.2 released on 23 February 2021

+ Bugfix: debug visualization not updated for kinematic rigid bodies
+ Updated the native libraries to v10.2.0 of Libbulletjme.

## Version 4.0.1 released on 16 February 2021

+ Bugfix: stack overflow while cloning a `DynamicAnimControl`
+ Upgraded to JUnit v4.13.2 .

## Version 4.0.0 released on 11 February 2021

+ Bug fixes:
  + debug visualization of a body jitters relative to its `PhysicsControl`.
  + `DacWizard` generates syntactically incorrect source code for some locales
  + issue #14 (suspension lengths of a `PhysicsVehicle` are not initialized)
  + `IllegalArgumentException` while cloning a `PhysicsVehicle`
    in a `PhysicsSpace`
+ A change to the library API:
  + Delete the `angularMomentum()` and `kineticEnergy()` methods
    from the `MultiBody` class.
+ Published to MavenCentral instead of JCenter.
+ Added a `render()` method to the `AbstractPhysicsControl` class.
+ Added tutorial app `HelloMotor`.
+ Built using Gradle v6.8.2 .
+ Based on:
  + v6.4.2 of the Heart library,
  + v0.6.2 of the Wes library,
  + v0.9.2 of the jme3-utilities-ui library, and
  + v0.9.18 of the jme3-utilities-nifty library.
+ Updated the native libraries to v10.1.0 of Libbulletjme, which includes
  Bullet v2 updates through 23 January 2021.

## Version 3.1.0 released on 9 January 2021

+ Re-implemented `CollisionSpace.destroy()` to prepare a space for re-use.
+ Dump 7 more properties of each `VehicleWheel`.
+ Added the `TestIssue13` application.
+ Built using Gradle v6.8 .
+ Updated the native libraries to v9.3.2 of Libbulletjme, which includes
  fixes for issues #12 and #13.

## Version 3.1.0-test4 released on 6 January 2021

+ Bugfix: `CollisionShapeFactory.createBoxShape()`
  positions the box incorrectly.
+ Added (experimental) support for the Linux_ARM32 platform with
  "hf" (hardware floating-point support).
+ Added a `createMergedBoxShape()` method to the `CollisionShapeFactory` class.
+ Updated the native libraries to v9.3.1 of Libbulletjme.

## Version 3.1.0-test3 released on 5 January 2021

+ Bugfix: off-by-one in validation of wheel indices
+ Added (experimental) support for the Linux_ARM32 platform.
+ Added a `createMergedHullShape()` method
  to the `CollisionShapeFactory` class.
+ Publicized the `addMultiBody()` and `removeMultiBody()` methods
  in the `MultiBodySpace` class.
+ Minimized usage of `PhysicsSpace.add()` and `PhysicsSpace.remove()`.
+ In `HelloPoi`, put the indicator in the `Translucent` bucket.
+ Built using Gradle v6.7.1 .
+ Based on:
  + v6.2.0 of the Heart Library,
  + v0.9.1 of the jme3-utilities-ui library,
  + v0.9.17 of the jme3-utilities-nifty library.
+ Updated the native libraries to v9.3.0 of Libbulletjme.

## Version 3.1.0-test2 released on 14 November 2020

+ Updated the native libraries to v9.2.3 of Libbulletjme (minSdkVersion=22).
+ Upgraded to JUnit v4.13.1 .

## Version 3.1.0-test1 released on 14 November 2020

+ Updated the native libraries to v9.2.3 of Libbulletjme (Android SDK v30).
+ Added 3 compatibility methods:
  + `BetterCharacterControl.getVelocity()`
  + `BetterCharacterControl.getViewDirection()`
  + `PhysicRayTestResult.getHitNormalLocal()`
+ Added tutorial app `HelloPoi`.
+ Built using Gradle v6.7 .
+ Based on v0.6.1 of the Wes Library.

## Version 3.0.0 released on 31 August 2020

+ Bug fixes:
  + collision-group checks are ineffective due to missing parentheses
  + `ConcurrentModificationException` thrown by the "Physics Cleaner" thread
  + assertion failures while tracking the ID of a soft-body anchor
  + JVM crash while reading the collision flags of a static rigid body
  + assertion failure during `PhysicsLink.postTick()`
+ Changes to the library API:
  + changed the semantics of the `countJoints()` and `listJoints()` methods
    in the `PhysicsBody` class
  + changed the return type of the `rayTestRaw()` method
    in the `CollisionSpace` class
+ New `DynamicAnimControl` features:
  + a mechanism to ignore collisions between physics links
    that aren't directly joined
  + a mechanism to apply bone rotations (from an `AnimClip`, for instance)
    to linked bones in dynamic mode
  + `setLocalTransform()` methods for the managed bones of physics links
  + a `fixToWorld()` method to lock a `PhysicsLink` into position
  + an adjustable `pinToWorld()` method
+ Improvements to `New6Dof` constraints:
  + enable springs in `RangeOfMotion` for more effective locking of axes
  + a `NewHinge` subclass inspired by Bullet's `btHinge2Constraint`
  + getters for the calculated transforms
+ More convenience:
  + added a factory method to construct a satisfied, double-ended `New6Dof`
    constraint using physics-space coordinates (instead of local ones)
  + added a `clearIgnoreList()` method to the `PhysicsCollisionObject` class
  + added a `CcdFilter` class to select rigid bodies with CCD active
  + added a `DacUserFilter` class to select physics objects belonging to
    a particular `DynamicAnimControl`
  + added `findEnd()` and `findOtherBody()` methods to the `PhysicsJoint` class
  + publicized the `boneIndex()` methods of `BoneLink` and `TorsoLink`
  + construct a `RangeOfMotion` (for a fixed joint) using Euler angles
+ Improvements to `PhysicsDumper`:
  + dump the positions of locked DoFs in a `New6Dof`
  + an option to dump the ignored PCOs of a `PhysicsCollisionObject`
  + dump the application data of a `PhysicsCollisionObject`
+ Built using Gradle v6.6.1 .
+ Based on:
  + v6.0.0 of the Heart Library,
  + v0.9.0 of the jme3-utilities-ui library,
  + v0.6.0 of the Wes Library, and
  + v0.9.15 of the jme3-utilities-nifty library.
+ Updated the native libraries to v9.2.2 of Libbulletjme.

## Version 2.0.1 released on 8 August 2020

Bugfix: characters and ghosts ignore their own ignore lists!

## Version 2.0.0 released on 22 July 2020

+ Bugfix: issue #9 (native crashes caused by invoking `finalizeNative()`
  outside of the "Physics Cleaner" thread)
+ Bugfix: race condition during the removal of an `NpoTracker`.
+ Bugfix: issue #10 (native IDs of soft physics joints shouldn't be tracked)
+ Bugfix: location not initialized when creating a `PhysicsCharacter`.
+ Added 2 new constructors to the `HeightfieldCollisionShape` class.
+ Added argument validation to the `setMaxSlope()` method
  in the `PhysicsCharacter` class.
+ Made more progress constructing the website.
+ Added 10 more tutorial apps.
+ Enabled gamma correction in tutorial apps that use lighting.
+ Updated the native libraries to v8.4.0 of Libbulletjme.

## Version 2.0.0-test2 released on 4 July 2020

+ Added collision listeners for ongoing contacts, to the `PhysicsSpace` class.
+ Added `totalAppliedForce()` and `totalAppliedTorque()` methods
  to the `PhysicsRigidBody` class.
+ Added a `setContactCalcArea3Points()` method
  to the `PhysicsCollisionEvent` class.
+ Added 3 compatibility methods to the `PhysicsCollisionEvent` class.
+ Made progress building the website.
+ Added 7 more tutorial apps.
+ Dump additional information for rigid bodies.
+ Improved descriptions of user objects.
+ Built using Gradle v6.5.1 .
+ Updated the native libraries to v8.3.0 of Libbulletjme.

## Version 2.0.0-test1 released on 19 June 2020

+ Changes to the library API:
  + Deleted the `MinieCharacterControl` and `VHACDResults` classes.
  + Replaced inner class `PhysicsSoftBody.Material` with `SoftBodyMaterial`.
  + Deleted deprecated methods and constructors.
  + De-publicized 7 debug-control classes.
  + Changed arguments of the `PhysicsCharacter`, `CharacterControl`,
    and `Convex2dShape` constructors from `CollisionShape` to `ConvexShape`.
  + Changed an argument of a `HullCollisionShape` constructor to `float...`.
  + Changed the `shape` arguments of `sweepTest()` methods to `ConvexShape`.
  + Changed an argument of the `volumeConvex()` method to `ConvexShape`.
  + Privatized 6 fields:
    + `CollisionSpace.physicsSpaceTL`
    + `CompoundMesh.scale`
    + `PhysicsSpace.physicsJoints`
    + `PhysicsSpace.pQueue`
    + `VHACD.indices`
    + `VHACD.vertices`
  + Changed 7 returned collections to unmodifiable collections.
  + Based the `PhysicsCollisionObject` and `PhysicsCollisionEvent` classes
    on the `NativePhysicsObject` class.
  + Added a private constructor to the `VHACD` class.

+ Eliminated all `finalize()` methods by implementing a cleaner thread and
  adding 4 classes:
  + `BoundingValueHierarchy`
  + `CharacterController`
  + `NpoTracker`
  + `VehicleController`
+ Added a GitHub Pages website, including javadoc and 3 new tutorial pages.
+ Implemented debug visualization for local physics.
+ Added 8 more tutorial apps.
+ Added the `TestEmptyShape` automated test.
+ Added accessors for the speculative contact restitution flag
  of a `PhysicsSpace`.
+ Upgraded to Gradle v6.4.1, Libbulletjme v8.1.0, and JUnit v4.13 .

## Version 1.7.0 released on 31 May 2020

+ Fixed bugs in the library:
  + kinematic `PhysicsRigidBody` cloned as a dynamic one
  + `CompoundCollisionShape.correctAxes()` yields incorrect results when
    multiple children reference the same shape
  + native objects of `SoftBodyWorldInfo` and `VehicleTuning` are never freed
  + `worldInfo` field of `PhysicsSoftBody` not cloned, loaded, or saved
  + `FINE` logging of collision spaces reports `nativeId=0` in `create()`

+ Fixed bugs in applications:
  + `SliderJoint` destroyed multiple times in `TestAttachDriver`
  + `NullPointerException` thrown in `TestRbc`
  + "childColoring" action not handled in `DropTest`

+ Deprecated many obsolete methods slated to be removed from v2.

+ Added debug-visualization features:
  + rebuild the debug shape of a `CompoundCollisionShape` ONLY when it changes
  + arrows to visualize cluster/node/rigid body velocity vectors
  + markers to visualize pinned soft-body nodes
  + arrows to visualize rigid/soft body gravity vectors
  + add texture coordinates when visualizing a `PlaneCollisionShape`
  + specify line widths for `PhysicsJoint` debug arrows
  + configure the shadow mode of the debug root node

+ Additional `PhysicsDumper` output:
  + restitution of each rigid body
  + angular velocity of each dynamic rigid body
  + split-impulse parameters of each `SolverInfo`
  + `isGravityProtected` for rigid bodies and `isWorldInfoProtected`
    for soft bodies
  + native ID for each `SoftBodyWorldInfo`

+ Other added library features:
  + an application-specific data reference for each `PhysicsCollisionObject`
  + an ignore list for each `PhysicsCollisionObject`
  + contact tests for collision spaces
  + an option to protect the gravity of a `PhysicsRigidBody` from modification
    by a `PhysicsSpace`
  + an option to protect the world info of a `PhysicsSoftBody` from replacement
    by a `PhysicsSoftSpace`
  + methods to rotate/translate a `CompoundCollisionShape`
  + a method to activate all collision objects in a `PhysicsSpace`
  + publicized the `addJoint()` and `removeJoint()` methods of `PhysicsSpace`
  + keep track of the `PhysicsSpace` (if any) to which each `PhysicsJoint`
    is added
  + construct an `IndexedMesh` from the debug mesh of a `CollisionShape`
  + construct a `MeshCollisionShape` from a collection of native meshes
  + a method to copy the cluster velocities of a `PhysicsSoftBody`
  + getters for the combined rolling/spinning friction
    of a `PhysicsCollisionEvent`
  + access the split-impulse parameters of a `SolverInfo`
  + `nativeId()` methods for `PhysicsCollisionEvent` and
    `PhysicsCollisionObject`, to prepare for v2
  + getters for the proxy group and proxy mask of a `PhysicsCollisionObject`
  + construct a `CompoundCollisionShape` with specified initial capacity

+ New applications added:
  + `TargetDemo`, a shooting demo
  + `PoolDemo`, an eight-ball pool simulation with moody lighting
  + `NewtonsCradle`, a Newton's cradle simulation
  + `TestScaleChange`
  + tests for JME issues 1283 and 1351
  + 3 apps that were missing from the Jme3Examples subproject

+ Improvements to the `DropTest` application:
  + use the PgUp key to "pop" the selected drop (if any)
  + use the slash key to toggle between lit and wireframe materials
  + when adding a Drop, avoid contact with existing rigid bodies
  + added soft-body drop types (cloth and squishyBall)
  + added jointed drop types (breakableRod, chain, diptych, flail, and ragdoll)
  + added fixed-shape drop types (ankh, banana, barrel, bowl, bowlingPin,
    horseshoe, iBeam, lidlessBox, link, snowman, table, thumbtack,
    triangularFrame, trident, and washer)
  + added "corner" and "square" platforms
  + made gravity configurable
  + applied a repeating texture to the "plane" platform visualization

+ Other improvements to existing applications:
  + minimized the hotkey help node initially
  + added the "BaseMesh" model to various demos
  + bound the "G" key to `System.gc()` in various demos
  + disabled audio rendering in all apps that use `AppSettings`

+ Major refactoring efforts:
  + many classes based on a new `NativePhysicsObject` class
  + many demo apps based on a new `AbstractDemo` class
  + 4 debug controls based on a new `CollisionShapeDebugControl` class
  + physics appstate configuration using a new `DebugConfiguration` class

+ Added 5 more models with CC0 licenses ("Ankh", "Banana", "Barrel",
  "BowlingPin", and "Horseshoe").
+ Updated the native libraries to v6.4.0 of Libbulletjme.
+ Based on:
  + the 3.3.2-stable release of jMonkeyEngine,
  + v5.5.0 of the Heart Library,
  + v0.8.3 of the jme3-utilities-ui library, and
  + v0.5.0 of the Wes Library.
+ Built using Gradle v6.4.1 .

## Version 1.6.1 released on 28 April 2020

Fixed JME issue 1351 (crash during garbage collection)

## Version 1.6.0 released on 12 April 2020

+ Fixed bugs:
  + `UnsatisfiedLinkError` on older Linux systems
    "libstdc++.so.6: version `CXXABI_1.3.8’ not found"
  + issue #2: certain soft-body methods cause access violations
    under Java 9+ on Windows systems
  + `NullPointerException` when re-adding a `PhysicsControl` that's already
    added to a `Spatial`
  + `rebuildRigidBody()` relied on static per-thread references to determine
    the body's `PhysicsSpace`
  + `NullPointerException` in `DacWizard` while editing the shape scale
    of a `PhysicsLink`

+ Added library features:
  + support for 4 Android platforms (arm64-v8a, armeabi-v7a, x86, and x86_64)
  + support for the 64-bit Linux-on-ARM platform (aarch64)
  + `DynamicAnimControl` ignores spatials tagged with "JmePhysicsIgnore"
  + `getCollisionSpace()` and `spaceId()` methods
    to the `PhysicsCollisionObject` class
  + `getRotationAngle()`, `setRotationAngle()`, `getSuspensionLength()`,
    and `setSuspensionLength()` methods to the `VehicleWheel` class
  + a `listDacMeshes()` method to the `RagUtils` class
  + an `appendFromNativeMesh()` method to the `NativeSoftBodyUtil` class
  + a `PcoType` class
  + a `getFlags()` method to the `PhysicsCollisionEvent` class
    and also a `ContactPointFlag` class
  + `copyIndices()` and `copyVertexPositions()` methods
    to the `IndexedMesh` class
  + a `serializeBvh()` method to the `MeshCollisionShape` class and also a
    constructor that takes serialized BVH

+ Improvements to the `DacWizard` application:
  + highlight the selected `PhysicsLink' in the Test screen
  + click RMB to pick a `PhysicsLink` in the Test screen
  + button in the Test screen to visualize the axes of the selected `BoneLink`
  + button in the Test screen to save the model to a J3O file
  + button in the Links screen to configure the `RotationOrder` of a `BoneLink`
  + button in the Bones screen to bypass RoM estimation if the model already
    has a DAC with the exact same linked bones
  + dialog in the Test screen to adjust collision margins
  + the "B"/PgUp and "N"/PgDn keys navigate between screens
  + buttons in the Load and Test screens to visualize skeletons
  + warn if there are multiple DACs in the model
  + dark grey background

+ Added most of the physics examples from `jme3-examples`
  to the Jme3Examples subproject.
+ Added a `TestDebugToPost` application to the MinieExamples subproject.
+ Added build-command options for double-precision and debug-ready versions
  of the library.
+ Reduced memory usage by reimplementing `IndexedMesh` using an `IndexBuffer`.
+ Customized the `RotationOrder` parameters of the sample DAC tunings.
+ Eliminated some non-standard collision margins
  from the MinieExamples subproject.
+ Removed all references to the CesiumMan model.
+ Updated the native libraries to version 5.5.7 of `Libbulletjme`.
+ Based on:
  + the 3.3.0-stable release of jMonkeyEngine,
  + v5.2.1 of the `Heart` library,
  + v0.8.2 of the `jme3-utilities-ui` library,
  + v0.9.14 of the `jme3-utilities-nifty` library, and
  + v0.4.9 of the `Wes` library.
+ Built using Gradle v6.3 .

## Version 1.5.0for33 released on 12 March 2020

+ Fixed bugs:
  + `NullPointerException` in the `DacWizard` application
  + compound shapes read from J3O assets always get the default margin
  + meshes returned by `DebugShapeFactory.getDebugMesh()` have incorrect bounds
  + Minie issue #3: `btAssert()` crash at the peak of a character's jump
    (only with a debug library)

+ Added library features:
  + `CharacterControl` class (for compatibility with jme3-bullet)
  + 2 more `PhysicsSpace` constructors (for compatibility with jme3-bullet)
  + new option for physics links: use `New6Dof` instead of `SixDofJoint`
  + `CollisionSpace` class (for collision detection without dynamics)
  + 4 more contact-and-constraint solvers for `PhysicsSpace`
  + 3 more solver parameters: global CFM, minimum batch, and mode flags
  + (experimental) support for multibody physics objects
  + `ConvexShape` abstract subclass of `CollisionShape`
  + new option for debug-mesh normals: sphere (radial) normals
  + new option to dump child collision shapes in detail
  + native IDs are now optional in physics dumps
  + dump a single `CollisionShape`
  + miscellaneous methods:
    + `BulletAppState.isRunning()`
    + `CollisionShapeFactory.createMergedMeshShape()`
    + `DebugShapeFactory.getDebugTriangles()`
    + `PhysicsSpace.countCollisionListeners()`
    + `PhysicsSpace.countTickListeners()`
    + `RagUtils.validate(Armature)`
    + `VHACDHull.clonePositions()`
    + `VHACDParameters.hashCode()`

+ Added more detail to `PhysicsCharacter` dumps.
+ Added validation for the angular limits of a `SixDofJoint`.
+ Updated the native libraries to version 5.0.0 of `Libbulletjme`.
+ Based on version 5.1 of the `Heart` library.
+ Built using Gradle v6.2.2 .
+ Continuous integration at TravisCI and GitHub.

## Version 1.4.1for33 released on 12 February 2020

Fixed JME issue 1283 (CCD doesn't respect collision groups)

## Version 1.4.0for33 released on 7 February 2020

+ Fixed bugs:
  + scaling bugs in `CompoundShape` and `Convex2dShape`
  + `MyShape.height()` returns wrong value for a `CylinderCollisionShape`
  + `DebugShapeFactory` cache should use a `WeakHashMap` for better garbage
    collection
  + issues with `NativeSoftBodyUtil`
  + soft-body debug geometries can't receive shadows

+ Added library features:
  + a V-HACD interface with progress listeners, based on JNI
    (eliminates the dependency on v-hacd-java-bindings)
  + array-based constructor for `IndexedMesh`
  + ray tests and sweep tests return a part index and/or triangle index
    for many collision shapes
  + `maxRadius()` methods for collision shapes
  + cleaner debug visualization of swept spheres
  + a debug visualization option to color the children of a compound shape
  + `countCollisionListeners()` and `countCollisionGroupListeners()` methods
     for the `PhysicsSpace` class
  + a `countPinnedNodes()` method for `PhysicsSoftBody`
  + a `parseNativeId()` method for the `MyShape` class, to replace `parseId()`
  + `countCachedMeshes()` and `maxDistance()` methods
    for the `DebugShapeFactory` class
  + `RayTestFlag` value to disable the heightfield accelerator
  + dump the listener counts of a `PhysicsSpace`

+ Improvements to the `DropTest` demo:
  + redesigned the user interface:  use fewer keys and also display
    a pause indicator and counts of active bodies and cached meshes
  + added platforms:  bed of nails, dimpled sheet, rounded rectangle,
    sieve, trampoline
  + added drop (gem) shapes:  dome, (gridiron) football, frame, half pipe,
    letters of the alphabet, prism, pyramid, sword
  + select a drop with RMB, delete or dump the selected drop
  + after deleting a drop, activate any that were asleep

+ changed the Maven groupId to "com.github.stephengold"
+ moved issue-oriented tests to a new package
+ moved the `jme3test` package to a new `Jme3Tests` sub-project
+ Updated the native libraries to version 3.0.4 of `Libbulletjme`.
+ Based on:
  + v5.0 of the `Heart` library,
  + v0.8.1 of the `jme3-utilities-ui` library, and
  + v0.4.8 of the `Wes` library.
+ Built using Gradle v6.1.1 .

## Version 1.3.0for33 released on 5 January 2020

+ Fixed bugs:
  + `PlaneCollisionShape` never visualized.
  + Buffer limits not set in `IndexedMesh`.

+ Added library features:
  + 2-D collision shapes: `Box2dShape` and `Convex2dShape`
  + a `setIndexBuffers()` method for the `DebugShapeFactory` class
  + dump the moments of inertia of dynamic rigid bodies
  + `castRay()`, `forwardAxisIndex()`, `rightAxisIndex()`, and `upAxisIndex()`
    methods for the `PhysicsVehicle` class
  + `getBrake()`, `getEngineForce()`, and `getSteerAngle()`
    methods for the `VehicleWheel` class
  + a `copyVertices()` method for the `SimplexCollisionShape` class
  + construct a `SimplexCollisionShape` from a `FloatBuffer` range
  + construct a `CylinderCollisionShape` from radius, height, and axis
  + a `getHeight()` method for the `CylinderCollisionShape` class
  + a `setScale(float)` method for the `CollisionShape` class
  + `addChildShape(CollisionShape)` and
    `addChildShape(CollisionShape, float, float, float)`
    methods for the `CompoundCollisionShape` class
  + a `listVolumes()` method for the `MyShape` class
  + `unscaledVolume()` methods for the `BoxCollisionShape`,
    `ConeCollisionShape`, `CylinderCollisionShape`, `EmptyCollisionShape`,
    and `SphereCollisionShape` classes
  + `DumpFlags` values for `BoundsInSpatials` and `VertexData`

+ Improvements to the `DropTest` demo:
  + Added 7 platform options (compound, cone, cylinder, hull, mesh, plane,
    and triangle).
  + Added 11 gem-shape options (barbell, capsule, chair, duck, heart,
    knucklebone, ladder, sphere, star, teapot, and top).
  + Tuned shadow edges.

+ Improvements to the `HeightfieldTest` demo:
  + Combined the demo with `TestScaleChange` and renamed it to `TestRbc`.
  + Added many test shapes.
  + Added a status line.
  + Vary the collision margin and scale.
  + Cursor shape indicates whether raytest finds an object.
  + Visualize Bullet's bounding box.
  + Added a hotkey to toggle the world axes.

+ Began using `createIndexBuffer()` to generate test meshes and V-HACD shapes,
  in order to conserve memory.
+ Refactored `TestRectangularShape` to make it more similar to the demos.
+ Extended `TestDefaults` to cover the `PhysicsGhostObject`,
  `PhysicsVehicle`, and `VehicleWheel` classes.
+ Updated the native libraries to version 2.0.19 of `Libbulletjme`.
+ Based on:
  + the NEW 3.3.0-beta1 release of jMonkeyEngine,
  + v4.3 of the  `jme3-utilities-heart` library,
  + v0.7.10 of the `jme3-utilities-ui` library, and
  + v0.9.12 of the `jme3-utilities-nifty` library.
  + v0.4.7 of the `Wes` library.

## Version 1.2.0for33 released on 16 December 2019

+ Added a `New6Dof` constraint class, to eventually replace both
  `SixDofJoint` and `SixDofSpringJoint`.  Also added 4 associated classes:
  `MotorParam`, `RotationOrder`, `RotationMotor`, and `TranslationMotor`.
+ Added a status line to the `SeJointDemo` application.
+ Changed the function of the Ins key in `SeJointDemo` and `TestDac`.
+ Updated the native libraries to version 2.0.17 of `Libbulletjme`.
+ Based on:
  + version 3.3.0-alpha5 of jMonkeyEngine,
  + version 4.1 of the  `jme3-utilities-heart` library,
  + version 0.7.8 of the `jme3-utilities-ui` library, and
  + version 0.9.10 of the `jme3-utilities-nifty` library.
  + version 0.4.5 of the `Wes` library.

## Version 1.1.1for33 released on 9 December 2019

+ Fixed bugs:
  + Crash due to a denormalized `Quaternion` in `TorsoLink`.
  + "K" key doubly mapped in the `TestDac` application.
+ Added model validation to the `DacWizard` application.
+ Added screenshot capability to 9 demo apps.
+ Extended `TestDefaults` to verify defaults for soft-body configs
  and materials.
+ Updated the native libraries to version 2.0.14 of `Libbulletjme`.
+ Based on:
  + jMonkeyEngine version v3.3.0-beta1, which was later deleted!
  + version 4.2 of the  `jme3-utilities-heart` library,
  + version 0.7.9 of the `jme3-utilities-ui` library, and
  + version 0.9.11 of the `jme3-utilities-nifty` library.
  + version 0.4.6 of the `Wes` library.
+ Built using Gradle v6.0.1 .

## Version 1.1.0for33 released on 4 November 2019

+ Added 4 getters to the `SixDofSpringJoint` class.
+ Added 3 compatibility methods to the `VehicleWheel` class.
+ Added some assertions to the `PhysicsRayTestResult` class.
+ Added the "application" Gradle plugin to the `DacWizard` build script.
+ Updated the native libraries to version 2.0.12 of `Libbulletjme`.
+ Built using Gradle v5.6.4 .

## Version 1.0.0for33 released on 8 October 2019

+ API changes:
  + Based `BulletAppState` on `AbstractAppState` (JME issue 1178).
  + Removed the `extrapolateTransform()` and `getPhysicsScale()` methods
    from `PhysicsRigidBody`.
  + Renamed the `getLocation()` and `getRotation()` methods of
    `ChildCollisionShape`.
  + Privatized the `objectId` fields of `CollisionShape` and `PhysicsJoint`.
  + Privatized the `collisionShape` field of `PhysicsCollisionObject`.
  + Privatized the `bodyA` and `bodyB` fields of `PhysicsJoint`.
  + Privatized the `cfm`, `erp`, and `split` fields of `SoftPhysicsJoint`.
  + Finalized the `getObjectId()` methods
    of `CollisionShape`, `PhysicsCollisionObject`, and `PhysicsJoint`.
  + Protected many constructors that shouldn't be invoked directly.
  + Removed the `countDistinctVertices()` method from `DebugMeshCallback`
+ Fixed bugs:
  + `DacLinks` attempts to link a bone with no vertices
  + in `DynamicAnimControl`, armature joints remain animated in ragdoll mode
  + in `BuoyDemo`, old skeleton visualization persists after model a change
  + `NullPointerException` while de-serializing an `AbstractPhysicsControl`
  + NPEs while serializing/de-serializing a `DynamicAnimControl` that's
    not added to a Spatial
  + `NullPointerException` while cloning a `SoftBodyControl`
  + out-of-bounds exception in `DebugMeshCallback` for an empty debug mesh
  + `SoftBodyDebugControl` doesn't resize debug meshes
  + `RuntimeException` in `DacWizard` while loading a non-model J3O
  + `NullPointerException` in `DacWizard` after loading a non-animated model
  + `OtoOldAnim.j3o` asset contained an invalid `MatParamOverride`
  + scaling and rotation bugs in `DacWizard`
  + bind pose not applied in to models in `TrackDemo` and `WatchDemo` apps
  + `RopeDemo` delete key cancels skeleton visualization
+ Added library features:
  + `getSquaredSpeed()` and `setEnableSleep()` for `PhysicsRigidBody`
  + `getActivationState()` for `PhysicsCollisionObject`
  + `Activation` and `AfMode` classes
  + `correctAxes()`, `principalAxes()`, and `setChildTransform()`
    for `CompoundCollisionShape`
  + `copyRotation()` and `copyTransform() methods for `ChildCollisionShape`
  + `countMeshTriangles()` for `MeshCollisionShape`
  + `isConvex()`, `isInfinite()`, `isNonMoving()`, and `isPolyhedral()` methods
    for `CollisionShape`
  + `getViewDirection()` for `MinieCharacterControl`
  + `IndexedMesh` constructors handle `TriangleFan` and
    `TriangleStrip` mesh types
  + `SoftBodyControl` handles 4 more mesh types
+ Enhancements to `PhysicsDumper`:
  + shape and group info of a `PhysicsCharacter`
  + group, orientation, scale, and shape of a `PhysicsGhost`
  + AABBs, activation state, damping, and friction of a `PhysicsRigidBody`
  + ID of the `CollisionShape` of a `PhysicsRigidBody`
  + wheels of a `PhysicsVehicle`
  + describe a `PlaneCollisionShape`
  + simplify descriptions of various shapes, especially compounds
+ Changes to `MultiSphereDemo`:
  + Renamed to `DropTest`.
  + Added box, compound, cone, cylinder, simplex, and V-HACD shapes.
  + Changed the Ins key to add a single gem instead of a shower.
  + Added a UI to tune damping and friction.
  + Added a `HeightfieldCollisionShape` platform as an alternative.
  + Randomized the initial orientation of each dynamic body.
+ Other improvements:
  + Updated `DacWizard` and demo apps to work with the new animation system.
  + Implemented `SoftJointDebugControl`.
  + `MinieAssets` sub-project converts OgreXML and glTF assets to J3O format.
  + Added the `ForceDemo` app.
  + Added the `TestCollisionShapeFactory`, `TestIssue1120`, and
    `TestPhysicsRayCast` apps from jme3-examples.
  + Added a "go limp" action to the "puppetInSkirt" test of `TestSoftBody`.
  + Avoid aliasing in `HeighfieldCollisionShape` constructors.
  + Added "toggle axes" and "toggle boxes" hotkeys to various demo apps.
  + Updated the native libraries to version 2.0.10 of `Libbulletjme`.
  + Based on version 4.1 of the `jme3-utilities-heart` library, version
    0.7.8 of the `jme3-utilities-ui` library, and version 0.9.10 of the
    `jme3-utilities-nifty` library.
  + Built using Gradle v5.6.2 .

release log continues at https://github.com/stephengold/Minie/blob/master/MinieLibrary/release-notes-pre10.md
