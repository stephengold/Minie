# release notes for the Minie library and related examples

## Version 0.7.7 released on TBD

 + Moved `RagUtils.vertexLocations()` and `VectorSet` to
   the `jme3-utilities-heart` library.
 + Lots of work on apps:
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