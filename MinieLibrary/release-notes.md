# release notes for the Minie library and related tests

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

 + Updated shared libraries to version 1.0.15 of `jme3-bullet-native`.
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
 + Updated native shared libraries to v1.0.12 of `jme3-bullet-native`.

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
 + Converted PhysicsBoneLink to a standalone class.
 + Removed unnecessary constructor from CollisionShape.
 + Added tests.

## Version 0.3.1 released on 24 September 2018

 + Fixed JME issue #896 and added a test for it.
 + Disabled getMargin() and setMargin() for capsule and sphere shapes.
 + Initialized the scale and margin of compound shapes.
 + Removed various methods and arguments.
 + Added TestSetMargin to the test project.

## Version 0.3.0 released on 23 September 2018

 + Fixed JME issue #740.
 + Standardized the design of constructors and accessors to reduce aliasing
   of vectors and quaternions and enable the use of caller-allocated storage.
 + Implemented a more practical approach to filtering debug objects.
 + Simplified PhysicsCollisionEvent by eliminating event types.
 + Renamed 2 PhysicsJoint methods that misspelled "bodies".
 + Removed many needless fields, methods, and constructors.
 + Made the VehicleTuning class JmeCloneable and Savable.
 + Addressed the possibility of multiple physics controls added to the
   same Spatial.
 + Replaced 6 parameters of VehicleWheel with a VehicleTuning reference.
 + Eviscerated 5 cloneForSpatial() methods.
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