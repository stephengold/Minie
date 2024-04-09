# Release log for the Minie library, DacWizard, and MinieExamples

release log continues at https://github.com/stephengold/Minie/blob/master/MinieLibrary/release-notes.md

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
   only after the 1st simulation step.  This helps avert initialization bugs.
 + Turned off hardware skinning in `DacLinks.createSpatialData()` to provide
   access to the true positions of mesh vertices.
 + Modified the `DynamicAnimControl.centerOfMass()` method to also estimate the
   velocity vector of the center of mass.
 + Links in kinematic mode now update their body's location and velocity for
   every frame, instead of just for each simulation step.
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
   `Spatial` does not change.
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
