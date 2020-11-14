# Release log for the Minie library, DacWizard, and MinieExamples

## Version 3.1.0-test1 released on TBD

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
 + API changes:
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

## Version 0.9.15for33 released on 29 August 2019

 + API changes:
   + Deprecated the `extrapolateTransform()` and `getPhysicsScale()` methods of
     the `PhysicsRigidBody` class.
   + Deleted the unused `OverlapListener` interface.
 + Fixed bug:
   + "body A does not exist" `NullPointerException` while loading a
     `DynamicAnimControl` from a J3O.
 + Added library features:
   + `getScale()` and `getTransform()` methods for `PhysicsCollisionObject`
   + Argument validation for `MeshCollisionShape` constructors
 + Other improvements:
   + Merged the functionality of `TestHullContact` into the
     `MultiSphereDemo` application.
   + Stopped overriding the default collision margin in `MultiSphereDemo`.

## Version 0.9.14for33 released on 25 August 2019

 + Simplified the construction of collision shapes from multiple meshes.
 + Changed `DacLinks` to skip `setGravity()` on kinematic bodies.
 + Added verification of the local copy of gravity in `PhysicsSpace`
   when assertions are enabled.
 + Updated the native libraries to version 2.0.7 of `Libbulletjme`.
 + Based on version 3.0 of the `jme3-utilities-heart` library, version
   0.7.7 of the `jme3-utilities-ui` library, and version 0.9.9 of the
   `jme3-utilities-nifty` library.

## Version 0.9.13for33 released on 7 August 2019

 + API changes:
   + Finalized the `TorsoLink.countManaged()` method.
   + Standardized `TranslationalLimitMotor.getAccumulatedImpulse()` to use
     caller-provided storage.
 + Fixed bugs:
   + Various bugs in debug visualization, including one where shadows
     were cast by visualizations of bounding boxes and swept spheres and one
     where axes were visualized after shapes were no
     longer visualized.
   + A `NullPointerException` in `DacLinks.findManagerForVertex()`.
   + A `NullPointerException` in `DacLinks.managerMap()`.
   + An `AssertionError` in `PhysicsSpace.countJoints()`.
   + An `AssertionError` caused by scaled compound shapes.
   + User objects were not cloned/serialized, even if they
     implement `JmeCloneable` or `Savable`.
   + Motor-enable flags and accumulated impulses weren't properly
     loaded/saved/cloned.
   + Control not found in `TrackDemo`.
   + A `NullPointerException` when changing models in `BalanceDemo`.
 + Added library features:
   + Support for V-HACD using Riccardo's Java bindings.
   + A warning in case a joint is added to a `PhysicsSpace` before
     the bodies that it joins.
   + Constructors for box/cylinder/sphere shapes based on float buffers.
   + Optional filtering of physics dumps.
   + Dump CCD/sleep parameters of dynamic rigid bodies.
   + `isEnabled()` and `setEnabled()` methods for `TranslationalLimitMotor`.
   + A flag to dump motors.
   + A `setAccumulatedImpulse()` method for `RotationalLimitMotor`.
   + `FilterAll` methods `countExceptions()`, `defaultReturnValue()`,
     and `listExceptions()`.
   + A `UserFilter` class.
 + Other improvements:
   + Extended the `setDebugViewPorts()` method of `BulletAppState` to accept
     multiple arguments.
   + Added a `JointDemo` app.
   + More thorough dumps/descriptions of joints, especially 6-DOF joints
     and their motors.
   + Enhanced the `BuoyDemo`, `MultiSphereDemo`, `RopeDemo`, `TestDac`,
     and `TestSoftBody` apps with hotkeys to toggle debug visualization options.
   + Added a hotkey to `MultiSphereDemo` to delete gems.
   + Updated the native libraries to version 2.0.5 of `Libbulletjme`.
   + Based on version 2.31 of the `jme3-utilities-heart` library, version
     0.7.6 of the `jme3-utilities-ui` library, and version 0.9.8 of the
     `jme3-utilities-nifty` library.

## Version 0.9.8for33 released on 17 July 2019

 + Added an `update(float, int)` method to the `PhysicsSpace` class.
 + Added `clearCache()`, `getDebugMesh()`, and `getDebugShape(CollisionShape)`
   methods to the `DebugShapeFactory` class.
 + Extended the constructors for `CompoundMesh`, `GImpactCollisionShape`, and
   `HullCollisionShape` to accept multiple meshes.
 + Updated the `TestHullContact` app to be more like a demo.
 + Built using Gradle v5.5.1 .

## Version 0.9.6for33 released on 13 July 2019

 + Finalized the `getRigidBody()` method in the `PhysicsLink`
   class. (API change)
 + Allowed soft-body nodes to have mass=0 (for pinning).
 + Changed the semantics of `RigidBodyControl.setKinematicSpatial()`
   to match jme3-bullet.
 + Added tutorial apps: `HelloSoftBody`, `HelloSoftSoft`, `HelloCloth`,
   and `HelloSoftRope`.
 + Added a `DividedLine` mesh class.
 + Moved the `massForStatic` constant to the `PhysicsBody` class.

## Version 0.9.5for33 released on 6 July 2019

 + Modified `DynamicAnimControl` to work with armatures as well as skeletons.
 + Fixed JME issue 1135 (`ConeJoint` causes rigid body to disappear).
 + Fixed cloning bugs in `SoftPhysicsJoint` and `SoftBodyControl`.
 + Removed 3 `jme3test` apps that now work unmodified with Minie.
 + Added a `contains(PhysicsJoint)` method to the `PhysicsSpace` class.
 + Added modified `TestRagDoll` and `TestGimpactShape` apps that
   work with Minie.
 + Added hotkey-binding hints to `TestHeightfield` and `TestHullContact`.
 + Added a `TestStaticBody` test.
 + Updated the native libraries to version 1.0.90 of `Libbulletjme`.
 + Based on version 2.29 of the `jme3-utilities-heart` library.
 + Based on version 3.3.0-alpha2 of jMonkeyEngine.

## Version 0.9.4 released on 2 July 2019

 + API changes:
   + Made the `DebugAppStateFilter` interface compatible with jme3-bullet again.
   + De-publicized the `PhysicsSpace.setLocalThreadPhysicsSpace()` method.
   + Removed the `createTriangleIndexVertexArray()` method from the
     `NativeMeshUtil` class.
   + Re-implemented anchors as a kind of `PhysicsJoint`.
   + Renamed the `BulletJointDebugControl` class.
   + Removed the `updateAnchorMesh()` method from the
     `NativeSoftBodyUtil` class.
 + Fixed bugs:
   + JME issue 1120 (scaled `GImpactCollisionShape` gets incorrect bounding box)
   + bounding box of a shape not updated for `getAabb()`
   + JME issue 1125 (inaccurate visualization of `HeightfieldCollisionShape`)
   + clusters/joints/nodes of a soft body not cloned/serialized properly
   + tau and impulseClamp of a `Point2PointJoint` not de-serialized properly
   + 7 limit-motor parameters of a `SixDofJoint` not serialized properly
   + JME issues 1126 and 1127 (`TestHoverTank` crash and reset action)
 + Added library features:
   + new classes `SoftAngularJoint` and `SoftLinearJoint` for soft-body joints
   + new classes `CompoundMesh` and `IndexedMesh` for native meshes
   + select single-sided/double-sided debug visualization materials for each
     collision object
   + an exception list for each `FilterAll` instance
   + control which soft bodies have their clusters visualized
   + a new constructor for a `HeightfieldCollisionShape` with additional options
   + access the `BulletDebugAppState` associated with a `BulletAppState`
   + access the `feedback` flag of each physics constraint
   + access 6 per-cluster parameters
   + access the per-constraint property that overrides the number of
     solver iterations
 + Other improvements:
   + Added hotkey-binding hints to demo apps: press H to toggle hints.
   + Bound the up/down arrow keys to control camera movement in
     demo applications.
   + Added a test for default values of newly created physics objects.
   + Customized the `toString()` methods of the `CollisionShape`,
     `PhysicsJoint`, and `PhysicsSpace` classes.
   + Improved the output of `PhysicsDumper`.
   + Customized the `equals()` and `hashCode()` methods of the
     `CollisionShape` class.
   + Avoided calling native code to fill zero-length buffers.
   + Removed native libraries from the Git repository; download them
     from GitHub instead.
   + Updated the native libraries to version 1.0.89 of `Libbulletjme`.

## Version 0.9.3 released on 11 June 2019

 + Moved 2 tutorial apps to a new `jme3utilities.tutorial` package.
 + Fixed bugs:
   + Spatial transform not applied to static bodies in `RigidBodyControl`.
   + `IllegalArgumentException` thrown when translating/rotating a
     heightfield-shaped rigid body.
   + Some physics controls ignore `isEnabled()`.
   + After de-serializing a physics control, its `userObject` is null.
   + `TestHeightfield` attached physics control to the root node.
   + `TestHeightfield` used wrong logger.
 + Added a `PhysicsSpace.destroy()` method for compatibility with jme3-bullet.
 + Improved physics dumps.
 + Removed uses of shared mutable "constants".
 + Made various improvements to MinieExamples.

## Version 0.9.2 released on 7 June 2019

 + API changes:
   + Privatized 5 fields in the `GhostControl` class.
   + Removed the `rebuildSoftBody()` method from the `PhysicsSoftBody` class.
   + Added a `Transform` argument to the `updateMesh()` method in the
     `NativeSoftBodyUtil` class.
 + New features for soft-body physics:
   + Added a `SoftBodyControl` class.
   + Allowed setting `maxSubSteps` to 0 for a variable-length time step.
   + Added `maxTimeStep` parameter to `PhysicsSpace`, for use with a
     variable-length time step.
   + Added a warning when `setGravity()` is applied to a body that isn't
     in any space.
   + Publicized the `distributeEvents()` method of `PhysicsSpace`, for use in
     non-`BulletAppState` applications.
   + An `isEmpty()` method for `PhysicsSoftBody`.
 + Built for compatibility with Java 7.
 + Added debug visualization of soft-body anchors.
 + Avoided cloning/serializing world info and gravity: adding to a physics space
   would trash these data.
 + Improvements to `DacWizard` and examples:
   + Added a `TestSoftBodyControl` application.
   + Made `ClothGrid` dynamic and added a `reposition()` method.
   + Changed `ClothGrid` to minimize directional bias.
   + Added Sony Duck model with license.
   + Use LWJGL v3 to allow fullscreen mode on Linux systems (JME issue #947).
 + Various improvements to physics dumps.
 + Updated the native libraries to version 1.0.73 of `Libbulletjme`.
 + Based on version 2.28.1 of the `jme3-utilities-heart` library.

## Version 0.9.1 released on 28 May 2019

 + API changes:
   + Privatized the `motionState` field in the `PhysicsRigidBody` class.
   + Removed methods from the `PhysicsSoftBody` class:
     + `addAeroForceToNode()`
     + `getPhysicsTransform()`
     + `setPhysicsRotation()`
     + `setPhysicsTransform()`
   + Renamed the `PhysicsRigidBody.getPhysicsTransform()` method
     to `extrapolateTransform()`.
   + Moved the `SoftBodyWorldInfo` class to the `com.jme3.bullet` package.
   + Converted the `PhysicsSoftBody.Config` class to an external class.
 + Expanded soft-body physics: got aerodynamics and anchors working.
 + Improved debug visualization of soft bodies:
   + override default material for shapes if requested
   + don't visualize links if the body has faces
   + visualize clusters
   + generate/update normals if requested
 + Other changes to debug visualization:
   + Ensured that static rigid bodies are visualized in blue.
   + Changed the wireframe debug materials from single-sided to double-sided.
   + Added a `debugMeshInitListener` option to add texture coordinates.
 + Added a `MinieAssets` sub-project.
 + Overrode the default `toString()` method for collision objects.
 + Added methods to the `PhysicsSoftBody` class:
   + `copyClusterMasses()`
   + `countNodesInCluster()`
   + `listNodesInCluster()`
   + `setWindVelocity()`
   + `windVelocity()`
 + Added `Aero` and `ClothGrid` classes.
 + Updated the native libraries to version 1.0.70 of `Libbulletjme`.
 + Based on version 2.28 of the `jme3-utilities-heart` library.

## Version 0.9.0 released on 14 May 2019

 + Added a minimal implementation of soft-body physics (based on Dokthar's
   prior work) that included `ConfigFlag`, `Icosphere`, `MeshEdge`,
   `NativeSoftBodyUtil`, `NetGrid`, `PhysicsBody`, `PhysicsSoftBody`,
   `PhysicsSoftSpace`, `RayTestFlag`, `Sbcp`, `SoftBodyWorldInfo`,
   `SoftPhysicsAppState`, `SoftDebugAppState`, `SoftBodyDebugControl`, and
   `TestSoftBody`.
 + Re-publicized the `update()` method and finalized the `getSpaceId()` method
   of `PhysicsSpace`.
 + Moved the `isInWorld()` method from `PhysicsRigidBody`
   to `PhysicsCollisionObject`.
 + Moved `TubeTreeMesh` class to the `jme3utilities.minie.test.mesh` package.
 + Used `BinaryExporter.saveAndLoad()` to simplify load/save testing.
 + Updated the native libraries to version 1.0.61 of `Libbulletjme`.
 + Based on version 2.27 of the `jme3-utilities-heart` library.

## Version 0.8.1 released on 28 April 2019

 + API changes:
   + Privatized 2 protected fields in the `BulletDebugAppState` class.
   + Privatized 7 protected fields in the `RigidBodyControl` class.
   + Privatized 4 protected fields in the `VehicleControl` class.
   + Renamed the `MyObject` class to `MyPco`.
   + Removed the `BubbleControl`, `SimpleGhostControl`,
     and `SimpleSolidControl` classes.
   + De-publicized the `BoundingBoxDebugControl` constructor.
   + Removed the `setPivot()` method from the `SixDofJoint` class.
 + Changed the semantics of the `addAll()` and `removeAll()` methods in the
   `PhysicsSpace` class; they no longer attempt to add/remove physics joints.
 + Added a swept-sphere visualization feature.
 + Added assertions to catch attempts to read the angular/linear velocities
   of non-dynamic rigid bodies.
 + Added methods to calculate the kinetic/mechanical energy of rigid body
   or a ragdoll.
 + Improved thread safety.
 + Added command-line options to DacWizard: --openGL3, --forceDialog, --verbose
 + Added a `getAngularVelocityLocal()` method to `PhysicsRigidBody`.
 + Moved the `FilterAll` class from MinieExamples into the library.
 + Added `getFrameTransform()` methods for cone, hinge, 6dof, and slider joints.
 + Updated the native libraries to version 1.0.50 of `Libbulletjme`.
 + Built using Gradle v5.3.1 .
 + Based on version 2.26 of the `jme3-utilities-heart` library.

## Version 0.8.0 released on 15 April 2019

 + Made IK joints aware of ragdoll mode. (API changes)
 + Privatized the `PhysicsSpace.physicsSpaceId` field. (API change)
 + Privatized all fields in `VehicleTuning`. (API change)
 + Added `getAngles()` and `getPivotOffset()` to the `SixDofJoint` class.
 + Publicized `RagUtils.coordsMap()` and added `RagUtils.findSkeletonControl()`.
 + Implemented `Comparable`, `equals()`, and `hashcode()`
   in the `LinkConfig` class.
 + Updated the native libraries to version 1.0.49 of `Libbulletjme`.
 + Avoided invoking `PhysicsRigidBody.addJoint()` directly.
 + Based on version 2.25 of the `jme3-utilities-heart` library.
 + Made progress on applications:
   + Added `DacWizard` and `TestIssue1058` applications.
   + Added a `Face` interface to each model tuning.

## Version 0.7.7 released on 28 March 2019

 + Moved `RagUtils.vertexLocations()` and `VectorSet` to
   the `jme3-utilities-heart` library.
 + Made progress on applications:
   + Added `WatchDemo` app.
   + Disabled contact response in `TuneDac`.
   + Moved IK controllers to their own package.
   + Used `getInverseInertiaWorld()` in `TrackController`.

## Version 0.7.6 released on 24 March 2019

 + Fixed a bug where `attachmentMass()` could return an outdated value.
 + Improved the performance of volume calculations for hull and multi-sphere
   shapes.
 + Various API changes in `RagUtils`.
 + Added a `MultiSphere` constructor for a 2-sphere shape, based
   on a `RectangularSolid`.
 + Added `boundingBox()` methods for collision shapes and collision objects.
 + Added `countRigidBodies()` and `getPcoList()` methods to the
   `PhysicsSpace` class.
 + Added a `density()` method to the `PhysicsLink` class.
 + Added a `HullCollisionShape` constructor based on a `FloatBuffer`.
 + Added a `TwoSphere` heuristic for generating `PhysicsLink` shapes.
 + Added a `VectorSet` abstract class with 2 implementations.
 + Updated the native libraries to version 1.0.40 of `Libbulletjme`.

## Version 0.7.5 released on 19 March 2019

 + Added a `DumpFlags` enum and used it to simplify the API
   of `PhysicsDumper`. (API change)
 + Added a `DumpFlag` to disable dumping collision objects in physics spaces.
 + Added a `getSpatial()` method to the `VehicleControl` class.
 + Based on version 2.23 of the `jme3-utilities-heart` library
   and JME 3.2.3-stable.

## Version 0.7.4 released on 12 March 2019

 + Fixed bugs that caused crashes in `GImpactCollisionShape.read()` and
   `GImpactCollisionShape.write()`.
 + Made `PhysicsDescriber` and `PhysicsDumper` both cloneable.
 + Added a `countHullVertices()` method to the `HullCollisionShape` class.
 + Added 2 configuration flags to `PhysicsDumper`.
 + Reorganized the `PhysicsDumper` code related to joints.
 + Updated the native libraries to version 1.0.37 of `Libbulletjme`.
 + Based on version 2.22 of the `jme3-utilities-heart` library.

## Version 0.7.3 released on 9 March 2019

Fixed a bug where `HullCollisionShape.copyHullVertices()` didn't fill the
`result` array.

## Version 0.7.2 released on 9 March 2019

Important changes to the library:

 + Fixed a bug where the angular factors of `PhysicsRigidBody` weren't
   cloned correctly.
 + Fixed a bug where the inverse inertia of `PhysicsRigidBody` wasn't
   read/written/cloned.
 + Write the platform type during `MeshCollisionShape` save and compare
   during load, since saved BVH may be incompatible between platforms.
 + Added a `getInverseInertiaWorld()` method to the `PhysicsRigidBody` class.
 + Added a `copyHullVertices()` method to the `HullCollisionShape` class.
 + Simplified `PhysicsDumper` output.
 + Updated the native libraries to version 1.0.37 of `Libbulletjme`.

## Version 0.7.1 released on 4 March 2019

Important changes to the library:

 + Fixed read/write/clone bugs in `MeshCollisionShape`.
 + Fixed JME issue #1029 using riccardobl's approach.
 + Fixed a crash that occurred while loading a rigid body with mass=0.
 + Fixed a `NullPointerException` that occurred while dumping
   single-ended joints.
 + Added `jump(void)` and `setGravity(float)` methods to `PhysicsCharacter`.
 + Changed the default gravity direction for a `PhysicsCharacter` from -Z to -Y.
 + Moved the `MyObject.describeUser()` method to the `PhysicsDescriber` class.
 + Publicized the `BulletAppState.stopPhysics()` method for better compatibility
   with `jme3-bullet`.
 + Added 2 new constructors for `BoxCollisionShape`.
 + Dump additional information on rigid bodies in `PhysicsDumper`.
 + Ensure that translation axes are locked in `RangeOfMotion`.
 + Reorganized the location/orientation getters for collision objects.
 + Updated the native libraries to version 1.0.34 of `Libbulletjme`.

## Version 0.7.0 released on 19 February 2019

Important changes to the library:

 + Changed the semantics of `DacConfiguration.detach()`: from unlinking
   a `Bone` to detaching an attachment. (API change)
 + Removed the `unlinkAttachment()` method
   from the `DacConfiguration` class. (API change)
 + Replaced `CompoundCollisionShape.getChildren()` with a new `listChildren()`
   method. (API change)
 + Fixed a bug where adding a `DynamicAnimControl`
   to a `Geometry` caused a crash.
 + Fixed a bug where an attached model didn't follow the rigid body when
   its `AttachmentLink` was in dynamic mode.
 + Fixed a bug where `MultiSphere.read()` threw a `ClassCastException`.
 + Created a `MinieCharacterControl` class as a replacement
   for JME's `CharacterControl`.
 + Added a `pinToSelf()` method to the `DynamicAnimControl` class.
 + Added a `contains()` method to the `PhysicsSpace` class.
 + Added `findIndex()` and `listChildren()` methods to the
   `CompoundCollisionShape` class

## Version 0.6.5 released on 8 February 2019

Important changes to the library:

 + Fixed various read/write bugs in `BetterCharacterControl`.
 + Cloned the rigid body in `BetterCharacterControl.cloneFields()`.
 + Standardized `BetterCharacterControl` getters to use
   caller-provided storage. (API changes)
 + Renamed the `describe()`, `getAxis()`, and `parseShapeId()` methods in the
   `MyShape` class. (API changes)
 + Added a `getRigidBody()` method to the `BetterCharacterControl` class.
 + Added accessors for 7 parameters (anisotropic friction, rolling friction,
   spinning friction, contact damping, contact stiffness, deactivation time,
   and contact processing threshold) to the `PhysicsCollisionObject` class.
   These should affect only rigid bodies and vehicles.
 + Added a `parseId()` method to the `MyObject` class.
 + Implemented the `Comparable` interface for the `CollisionShape` class.
 + Added a `setLocationAndBasis()` method to the `PhysicsCollisionObject` class.
 + Added accessors for friction and restitution to the `PhysicsGhostObject`
   and `PhysicsCharacter` classes.  These should have no effect.
 + Updated the native libraries to version 1.0.30 of `Libbulletjme`.

## Version 0.6.4 released on 25 January 2019

Important changes to the library:

 + Standardized the `PhysicsCharacter.getWalkDirection()` method. (API change)
 + Fixed a bug where debug shapes were re-used incorrectly.
 + Fixed a bug where 6 `PhysicsCharacter` parameters were neither loaded
   nor saved.
 + Added an `isDynamic()` method to the `PhysicsRigidBody` class.
 + Added optional axes to debug visualizations.
 + Moved the CCD accessors to the `PhysicsCollisionObject` class.
 + Added a `copyCenter()` method to the `MultiSphere` class.
 + Added the capability to disable the startup message.
 + Added `getUpDirection()` and `reset()` methods to the
   `PhysicsCharacter` class.
 + Added the capability to configure the `PhysicsCharacter` sweep test.
 + Reduced `CollisionShape` validation in the `PhysicsRigidBody` class.
 + Updated the native libraries to version 1.0.29 of `Libbulletjme`.

Important changes to the examples:

 + Port the `TestQ3` app from `jme3-examples`.
 + In `TestRectagularSolid`, set the seed for each trial and add UI text.

## Version 0.6.3 released on 17 January 2019

Important changes to the library:

 + Prohibited `PhysicsRigidBody.setKinematic()` on static bodies.
 + Used `EmptyShape` to permit linking a bone without
   vertices in a `DynamicAnimControl`.
 + Added `getSpatial()` methods to `GhostControl` and `RigidBodyControl`.
 + Updated the native libraries to version 1.0.24 of `jme3-bullet-native`.

Important changes to the examples:

 + Added new apps: `HelloDac`, `HelloBoneLink`, `TestHullContact`.
 + Added apps from the jme3-examples (sub)project: `TestSimplePhysics`,
   `TestRagdollCharacter`, and `TestBoneRagdoll`.
 + Added example tuning for CesiumMan model. (model not provided)
 + Simplified the example tuning for the Jaime model.

## Version 0.6.2 released on 6 January 2019

 + Fixed bug where `SimpleSolidControl.onAdd()` threw a `NullPointerException`
   if the control wasn't added to a `PhysicsSpace`.
 + Added a `countJoints()` method to `PhysicsRigidBody`.
 + Added a `setLimit()` method to `HingeJoint`.
 + Allowed vertical translation of heightfields.
 + Based on version 2.18 of the `jme3-utilities-heart` library
   and JME 3.2.2-stable.

## Version 0.6.1 released on 28 December 2018

 + Added an option to calculate local coordinates
   in `DynamicAnimControl.findManagerForVertex()`.
 + Added a `chainLength` argument to `DynamicAnimControl.setDynamicChain()`.
 + Finalized 4 library methods.
 + Created a `CameraOrbitAppState` class for use in examples.
 + Based on version 2.17 of the `jme3-utilities-heart` library
   and JME 3.2.2-beta1.
 + Disable scene-graph culling for animated models in examples.

## Version 0.6.0 released on 15 December 2018

Noteworthy additions:

 + An `IKController` class for inverse kinematics.  Each `PhysicsLink` maintains
   a list of IK controllers.
 + 3 IK joint creation methods in `DynamicAnimControl`:
   `moveToBody()`, `moveToWorld()`, and `pinToWorld()`.
 + Each `DynamicAnimControl` keeps a list of IK joints and disables those
   joints when entering ragdoll mode.
 + A `BalanceDemo` with 2 examples of `IKController`.
 + An `EmptyShape` class.
 + A `setDynamicChain()` method in `DynamicAnimControl`.
 + Optional `Biped` and `Binocular` interfaces for `DynamicAnimControl`
   subclasses.
 + A `footprint()` method to calculate the "footprint" of a `PhysicsLink`.
 + An `animateSubtree()` method for `DynamicAnimControl`.
 + A constructor for a single-ended `Point2PointJoint` with its
   constraint already satisfied.
 + An `isActive()` method for all collision objects (not just rigid bodies).
 + An `isDetached()` method for all physics links (not just attachments).
 + A `setContactResponse()` method for physics characters (not just bodies).
 + Simple `compareTo()`, `equals()`, and `hash()` methods for collision objects.

Bugs fixed:

 + Single-ended point-to-point joints were created with incorrect world
   locations for their pivots.
 + `PhysicsRigidBody` and `PhysicsCharacter` were not cloned properly.

Debugging improvements:

 + Generally made dumps more compact by trimming trailing zeros.
 + In dumps, indicate joints with out-of-space bodies.
 + In dumps, indicate joints that lack a dynamic body.
 + In dumps, indicate non-responsive rigid bodies.
 + In visualizations, draw non-contact physics characters in yellow wireframe.

Other important changes:

 + Added the concept of a `DynamicAnimControl` being "ready" for dynamic-mode
   only after the 1st physics timestep.  This helps avert initialization bugs.
 + Turned off hardware skinning in `DacLinks.createSpatialData()` to provide
   access to the true positions of mesh vertices.
 + Modified the `DynamicAnimControl.centerOfMass()` method to also estimate the
   velocity vector of the center of mass.
 + Links in kinematic mode now update their body's location and velocity for
   every frame, instead of just for each timestep.
 + Eliminated the `PhysicsJoint.getPivotInWorld()` method. (API change)
 + Renamed `DacPhysicsLinks` to `DacLinks`. (API change)
 + `PhysicsSpace` accessors now return physics-object collections sorted by ID.

Other details:

 + Updated the native libraries to version 1.0.21 of `jme3-bullet-native`.
 + Based on version 2.16 of the `jme3-utilities-heart` library.

## Version 0.5.1 released on 5 December 2018

 + Added a "contact response" option for rigid bodies.
 + Added an `isStatic()` method to `PhysicsCollisionObject`.
 + Prohibited scaling of `SimplexCollisionShape`.
 + Added a `MultiSphere` constructor for a capsule shape with indexed axis.

Noteworthy changes to debug visualization:

 + Fixed a bug where physics objects and joints continued to be visualized after
   setting a filter to exclude them.
 + Update debug spatials on every change to `debugMeshNormals`
   or `debugMeshResolution`.
 + For a `CompoundCollisionShape`, generate a new debug spatial on every frame.
 + Visualize non-responsive rigid bodies in yellow.

Noteworthy changes to `DynamicAnimControl`:

 + Added `centerOfMass()` and `setAttachmentConfig()` methods.
 + Collect mesh-vertex coordinates in a `HashSet` (instead of an `ArrayList`)
   to increase the efficiency of `createSpatialData()`.

Other details:

 + Updated the native libraries to version 1.0.20 of `jme3-bullet-native`.
 + Based on version 2.15 of the `jme3-utilities-heart` library.

## Version 0.5.0 released on 29 November 2018

Minie moved from the Jme3-utilities Project to a new GitHub repo.

Noteworthy features added:

 + Added single-ended versions of all 6 `PhysicsJoint` types.
 + Added optional heuristics for configuring the center, shape, and
   mass of a `PhysicsLink`.
 + Added 4 methods for compatibility with the `jme3-bullet` library.
 + Cache and re-use debug meshes.
 + Added an enable flag and a breaking impulse threshold to every
   `PhysicsJoint` object.
 + Added an MhGame model for use by `TestDac`.
 + Added `SeJointDemo`, `TestLargeMesh` and `TestRectangularSolid` example apps.
 + Added a `HullCollisionShape` constructor based on `RectangularSolid`.
 + Added `MultiSphere` constructors based on `BoundingSphere`
   and `RectangularSolid`.

Other important changes:
 + Debug-mesh properties are now per-collision object, instead of per-shape.
 + In the example model tunings, configure masses based on density.
 + In `BulletJointDebugControl`, visualize the A and B ends of each joint
   in distinct colors.
 + Moved 10 assertion-based tests to the library's "test" source set.
 + Fixed a bug in the `TestDac` application where controls were not removed
   for certain models.
 + Fixed a bug in `BulletVehicleDebugControl` where odd-numbered wheels were
   never updated.

Other details:

 + Updated the native libraries to version 1.0.18 of `jme3-bullet-native`.
 + Based on version 2.14 of the `jme3-utilities-heart` library.

## Version 0.4.5 released on 20 November 2018

Main features added:

 + New `MultiSphere` collision shape and `MultiSphereDemo` app.
 + Configure normals and resolution of the debug mesh for each collision shape.
 + Register an init listener for a `BulletDebugAppState`.
 + The Puppet model with its licensing history (for the `TestDac` app).
 + Test whether a collision shape is convex.
 + Vertex counts and volume calculations for various collision shapes.
 + Calculate half extents for simplex and hull collision shapes.
 + Copy the vertices of a `SimplexCollisionShape`.

Bugs fixed:

 + Incorrect default limits for `SixDofJoint`.
 + Crash in `PhysicsSpace.stepSimulation()` after reading a hull shape
   from a model asset.
 + Debug mesh is not updated after the shape's margin changes.
 + `MyShape.volume()` ignores scaling of capsule shapes.

Incompatible changes to the library API:

 + Renamed `PhysicsRigidBody.getJoints()` to `listJoints()` and changed its
   semantics to reduce aliasing.
 + Privatized the joint list in `PhysicsRigidBody`.
 + Removed 3 inverse-kinematics stub methods from `DynamicAnimControl`.
 + Standardized the `getLowerLimit()` and `getUpperLimit()` methods
   in the `TranslationalLimitMotor` class.
 + Removed the `getTriangleIndexVertexArray()` method
   from the `NativeMeshUtil` class.

Other details:

 + Updated the native libraries to version 1.0.15 of `jme3-bullet-native`.
 + Based on version 2.13 of the `jme3-utilities-heart` library.

## Version 0.4.4 released on 12 November 2018

 + Fixed map cloning bugs in `DynamicAnimControl` and
   `ConfigDynamicAnimControl`.
 + Added a `countJoints()` method and removed the `destroy()` method of
   the `PhysicsSpace` class.
 + Reduce aliasing in the `BulletDebugAppState` constructor.

## Version 0.4.3 released on 8 November 2018

 + Changed `DynamicAnimControl.setMass()` to take a bone name or a physics link.
 + Made `MyControlP` aware that `DynamicAnimControl` does not support
   local physics.
 + Added `DynamicAnimControl` tunings for the MhGame and Puppet models.

## Version 0.4.2 released on 3 November 2018

More changes to `DynamicAnimControl`:

 + Added the capability to release attachments.
 + Gave each `PhysicsLink` a name that's distinct from its bone's name.
 + Added `hasAttachmentLink()` and `unlinkAttachment()` methods.
 + Removed the `isLinkName()` method.
 + Renamed the `isBoneLinkName()` method to `hasBoneLink()`.
 + Renamed the `unlink()` method to `unlinkBone()`.
 + Changed the `attachmentBoneNames()` and `linkedBoneNames()` methods to return
   arrays instead of collections.
 + Added example tunings for the Ninja and Oto models.
 + Detect and reject models with ignoreTransform geometries.

Other noteworthy changes to Minie:

 + In `RangeOfMotion`, set the joint's angular limits in addition
   to its motor limits.
 + In `SixDofJoint`, store rotational motors in an array, not a linked list.

## Version 0.4.1 released on 1 November 2018

More design and implementation changes to `DynamicAnimControl`:

 + Added support for attachments nodes.
 + The center of a linked rigid body can be offset from its joint.
 + Major refactoring to base `AttachmentLink`, `BoneLink`, and `TorsoLink`
   on a new `PhysicsLink` class.
 + Renamed `JointPreset` class to `RangeOfMotion`.
 + Refer to links by reference instead of by name.
 + Added a `forceKinematic` option to the `freeze()` methods.
 + Renamed many methods.
 + Preserve animation data during a `rebuild()`.
 + Moved `RagdollCollisionListener` to the `com.jme3.bullet.animation` package.
 + Completed the `read()` and `write()` methods.
 + Lowered the default for `torsoMass` from 15 to 1.
 + Fixed bug where `DynamicAnimControl` reported collisions from other DACs.
 + Catch any attempt to set local physics.
 + Don't re-order controls unless it's necessary.

Other noteworthy changes to Minie:

 + Improved dumps and descriptions of joints, physics controls,
   collision objects, and rigid bodies.
 + Standardized `getPivot()` methods to avoid aliasing.
 + Avoided aliasing in `setViewPorts()` methods.
 + Added a `getTargetVelocity()` method to `TranslationalLimitMotor`.
 + Added an `activate()` method to `PhysicsCollisionObject`.
 + Fixed a bug that caused an assertion failure while reading
   a `CompoundCollisionShape`.
 + Updated the native libraries to v1.0.12 of `jme3-bullet-native`.

## Version 0.4.0 released on 20 October 2018

Extensive design and implementation changes to `KinematicRagdollControl`
and its ilk, now in its own `com.jme3.bullet.animation` package.
`DynamicAnimControl` is now the core, with `ConfigDynamicAnimControl` for
configuration.

 + Bone shapes are now aligned with bone coordinate axes instead of mesh
   coordinate axes.
 + The inverse-kinematics code has been removed in anticipation of a
   complete redesign.
 + Kinematic mode now has 4 submodes.
 + Rigid body updates now take place just before each physics tick instead of
   during scene-graph updates.
 + Added an interim tool for tuning a `DynamicAnimationControl`.
 + Added example tunings for the Jaime and Elephant models.
 + Added per-axis freezing of dynamic joints.

Other noteworthy changes to Minie:

 + Added a getSpatial() method to the `AbstractPhysicsControl` class and
   privatized its `spatial` field.  Also added an empty controlRender() method.
 + Bugfix: `NullPointerException` in `PhysicsSpace.getGravity()`.
 + Bugfix: JVM crashed while reading a `SixDofJoint` from a J3O asset.
 + Added check for invalid location in `RigidBodyMotionState.getWorldLocation()`
 + Disabled the `isInWorld` checks in `PhysicsRigidBody`.
 + Renamed and standardized the accessors of `RigidBodyMotionState`.
 + In `RotationalLimitMotor`: renamed the limit/bounce accessors, added
   accessors for CFM parameters, added `getAccumulatedImpulse()` and
   `getAngle()` methods.
 + In `TranslationalLimitMotor`: added accessors for 4 parameters, added
   `getOffset()` and `setTargetVelocity()` methods.
 + Added `getPhysicsTransform()` and `setPhysicsTransform()` methods to
   the `PhysicsRigidBody` class.
 + Bypassed `setSpatial()` in `GhostControl` and `RigidBodyControl` in case the
   spatial does not change.
 + Eliminated unnecessary aliasing in joint constructors.
 + Named the debug textures in `BulletDebugAppState`.
 + Visualize kinematic bodies in blue instead of magenta, to distinguish
   them from dynamic bodies.

## Version 0.3.5 released on 10 October 2018

Enhancements to `KinematicRagdollControl`:

 + Began treating the torso more like a bone.
 + Implemented a new algorithm to construct hulls without weight thresholds.
 + Redesigned how mass is configured and totaled.
 + Lowered the default dispatch threshold from 10 to 0.
 + Moved all the code in `RagdollUtils` to other classes.
 + Include the torso in `setDamping()`.
 + Added `boneMass()`, `getBoneLink()`, `getJointPreset()`, `gravity()`,
   `linkedBoneNames()`, `setGravity()`, and `torsoMass()` methods.

Other noteworthy changes:

 + Added `getPhysicsScale()` and `setPhysicsScale()` methods to
   `PhysicsRigidBody`.
 + Removed the `space` argument from the addPhysics() and removePhysics()
   methods of `AbstractPhysicsControl` and its subclasses.
 + Added a list-based constructor for `HullCollisionShape`.
 + Fixed a logic bug in `MyObject` where vehicles were not recognized.
 + Added a `setPivot` method to `SixDofJoint.setPivot()`.
 + Added a `physicsTransform()` method to `RigidBodyMotionState`.
 + Added `JointEnd` and `TestRagdollScaling` classes.
 + Updated shared libraries to v1.0.7 of `jme3-bullet-native`.
 + Removed the unused `PhysicsSpace.initNativePhysics()` method.

## Version 0.3.4 released on 5 October 2018

Enhancements to `KinematicRagdollControl`:

 + Removed the `weightThreshold = -1` hack.
 + Replaced `boneList` and `RagdollPreset` with a joint map.
 + Changed coordinate translation to utilize animated geometries
   instead of the controlled spatial.
 + Eliminated the temporary removal of the controlled spatial from the scene.
 + Changed to continue updating ragdoll even after the controlled spatial moves.

Other noteworthy changes:

 + Fixed 2 logic errors in `CylinderCollisionShape.canScale()`.
 + Added result validation to `PhysicsRigidBody.getPhysicsLocation()`.
 + Fixed JME issue #931.
 + Updated shared libraries to v1.0.5 of `jme3-bullet-native`.
 + Improved `applyScale` option in `GhostControl` and `RigidBodyControl` so that
   it will fall back to uniform scaling (if necessary) or skip rescale
   (if scale is unchanged).
 + Added an `isEmpty()` method to the `PhysicsSpace` class.
 + Added `TestSetScale`, `TestIssue918`, and `TestIssue919`.

## Version 0.3.3 released on 2 October 2018

 + Added `applyScale` option to `RigidBodyControl` and `GhostControl`.
 + Added default margin for collision shapes other than capsule and sphere.
 + Eliminated runtime dependency on JME's `jme3-bullet-native` library.
 + Removed `TestIssue896`.

## Version 0.3.2 released on 28 September 2018

 + Made many classes `JmeCloneable`, especially physics controls.
 + Added custom debug materials to collision objects.
 + Added `canScale()` method to collision shapes.
 + Worked around JME issue #919.
 + Prevented setting the margin of a capsule/sphere shape.
 + Implemented limb damping in `KinematicRagdollControl`.
 + Added `getTorso()` method to `KinematicRagdollControl`.
 + Added check for rotation/translation of a heightfield rigid body.
 + Converted `PhysicsBoneLink` to a standalone class.
 + Removed unnecessary constructor from CollisionShape.
 + Added tests.

## Version 0.3.1 released on 24 September 2018

 + Fixed JME issue #896 and added a test for it.
 + Disabled `getMargin()` and `setMargin()` for capsule and sphere shapes.
 + Initialized the scale and margin of compound shapes.
 + Removed various methods and arguments.
 + Added `TestSetMargin` to the test project.

## Version 0.3.0 released on 23 September 2018

 + Fixed JME issue #740.
 + Standardized the design of constructors and accessors to reduce aliasing
   of vectors and quaternions and enable the use of caller-allocated storage.
 + Implemented a more practical approach to filtering debug objects.
 + Simplified `PhysicsCollisionEvent` by eliminating event types.
 + Renamed 2 `PhysicsJoint` methods that misspelled "bodies".
 + Removed many needless fields, methods, and constructors.
 + Made the `VehicleTuning` class `JmeCloneable` and `Savable`.
 + Addressed the possibility of multiple physics controls added to the
   same Spatial.
 + Replaced 6 parameters of `VehicleWheel` with a `VehicleTuning` reference.
 + Eviscerated 5 `cloneForSpatial()` methods.
 + Based on version 2.10 of the jme3-utilities-heart library.

## Version 0.2.10 released on 12 September 2018

 + Fixed JME issue #898.
 + Require collision margin > 0 .
 + Changed default collision margin from 0 to 0.04 .
 + Disabled setMargin() for SphereCollisionShape.
 + Don't allow dynamic bodies to have heightfield or plane shapes.
 + Publicized loggers.
 + Added massForStatic constant in PhysicsRigidBody.
 + Added 2 tests.
 + Privatized the HeightfieldCollisionShape.createShape() method.

## Version 0.2.9 released on 9 September 2018

 + Removed PhysicsCollisionEventFactory.
 + Removed HeightfieldCollisionShape.createJmeMesh(),
   VehicleWheel.getGroundObject(), and a constructor for PhysicsGhostObject.
 + Privatized various methods.
 + Fixed JME issue #894.
 + Implemented a cleaner fix for JME issue #889.
 + Deal with scale changes in physics-debug controls.
 + Decided that physics-debug controls should implement neither JmeCloneable
   nor Savable.
 + Added validation of method arguments.
 + Finalized various fields.
 + Created the jme3utilities.minie.test package.

## Version 0.2.8 released on 3 September 2018

 + Removed some unnecessary methods.
 + Reduced the scope of many methods.
 + Renamed getMass() to mass() in MyControlP.
 + Fixed JME issue #889.
 + Added validation of method arguments, plus some assertions.
 + Based on version 2.8 of the jme3-utilities-heart library.

## Version 0.2.7 released on 1 September 2018

 + Don't setLocalScale() on spatials controlled by debug controls; this is
   related to JME issue #887.
 + Handle ignoreTransforms in GhostControl and RigidBodyControl.
 + Describe rigid bodies and RigidBodyControls similarly.
 + Describe shape scaling and spatial scaling similarly.
 + Describe the half extents of box shapes.

## Version 0.2.6 released on 31 August 2018

 + Fixed JME issues 883 and 887.
 + Ensured that debugViewPorts[] gets initialized in BulletAppState.
 + Changed AbstractPhysicsControl to handle ignoreTransform.
 + Changed DebugAppStateFilter interface to consider only Savable objects.
 + Added validation of method arguments, plus some assertions.
 + Reduced the scope of many fields and methods.
 + Finalized some fields.
 + Removed some unused fields and methods.
 + Added jme3-bullet-native runtime dependency to POM.
 + Replaced iterators with enhanced loops (for readability).
 + Standardized logging.

## Version 0.2.5 released on 24 August 2018

 + Bugfix: PhysicsDumper prints incorrect number of vehicles.
 + Bugfix for JME issue #867 contributed by Riccardo Balbo.
 + Privatized numerous protected fields.
 + Removed 3 PhysicsSpace constructors.
 + Enhanced PhysicsDumper to handle app states and print the joint list and
   (non-identity) orientation for each rigid body.
 + Added BulletAppState.getBroadphaseType().
 + Added validation of method arguments.
 + Changed BulletAppState.setWorldMax() and .setWorldMin() to avoid aliasing.

## Version 0.2.4 released on 22 August 2018

 + Renamed MinieVersion.getVersionShort() to versionShort().
 + Used MyAsset to create debug materials.
 + In BulletDebugAppState, only render viewports that are enabled.
 + Based on version 2.7 of the jme3-utilities-heart library.

## Version 0.2.3 released on 17 August 2018

+ Renamed ray-test flag accessors in PhysicsSpace class. (API change)
+ Added maxSubSteps() method to the PhysicsSpace class.
+ Include more detail when dumping a physics space.
+ Based on version 2.6 of the jme3-utilities-heart library.

## Version 0.2.2 released on 24 July 2018

+ Enhanced PhysicsDescriber to describe axes of cone shapes.
+ Based on version 2.5 of the jme3-utilities-heart library.
+ Remove an obsolete TODO comment.

## Version 0.2.1 released on 19 February 2018

+ Changed BulletDebugAppState to accept an array of viewports to add scenes to,
  instead of creating its own viewport.
+ Added getAxis() method to the ConeCollisionShape class.
+ Allow uniform scaling of capsule, cylinder, and sphere shapes.

## Version 0.2.0 released on 2 February 2018

+ Added axisIndex(), describe(), describeType(), halfExtents(), height(),
  radius(), setHalfExtents(), setHeight(), and setRadius() methods to the
  MyShape utility class.
+ Copied source files from jme3-bullet library and corrected many minor issues.

## Version 0.1.2 released on 26 January 2018

This was the initial baseline release, based largely on code formerly
included in the jme3-utilities-heart, jme3-utilities-debug, and
jme3-utilities-x libraries.