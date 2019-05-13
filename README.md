<img height="150" src="https://i.imgur.com/YEPFEcx.png">

The [Minie Project][minie] is about improving the integration of
[Bullet Real-Time Physics][bullet] into the
[jMonkeyEngine Game Engine][jme].

It contains 3 sub-projects:

 1. MinieLibrary: the Minie runtime library (in Java)
 2. MinieExamples: demos, examples, and test software (in Java)
 3. [DacWizard][]: a GUI application to configure a ragdoll (in Java)

Java source code is provided under
[a FreeBSD license](https://github.com/stephengold/Minie/blob/master/LICENSE).

## Contents of this document

 + [Why use Minie?](#why)
 + [Downloads](#downloads)
 + [Conventions](#conventions)
 + [History](#history)
 + [How to install the SDK and the Minie Project](#install)
 + [How to add Minie to an existing project](#add)
 + [Choosing a collision shape](#shape)
 + [An introduction to DynamicAnimControl](#dac)
 + [External links](#links)
 + [Acknowledgments](#acks)

<a name="why"/>

## Why use Minie?

jMonkeyEngine comes with 2 Bullet integration libraries.
Why use Minie instead of `jme3-bullet` or `jme3-jbullet`?

 + Minie has many more features. (See the feature list below.)
 + Minie fixes many bugs found in the jMonkeyEngine libraries.
   (See the fix list below.)
 + Due to its shorter release cycle, future features and bug fixes
   will probably appear first in Minie.
 + Minie has automated tests that reduce the risk of regressions and new bugs.
 + Minie's classes are better encapsulated, with fewer public/protected fields
   and less aliasing of small objects like vectors.  This reduces the risk
   of accidentally corrupting its internal data structures.
 + Minie validates method arguments.  This helps detect usage errors that
   can lead to subtle bugs.
 + Minie's source code is more readable and better documented.

Summary of added features:

 + `DynamicAnimControl` for ragdoll simulation:
    + set dynamic/kinematic mode per bone
    + understands attachments
    + highly configurable, with many options for bone mass, center, and shape
    + apply inverse-kinematic controllers and joints
 + `MultiSphere` collision shapes based on `btMultiSphereShape`
 + `EmptyShape` collision shapes based on `btEmptyShape`
 + debugging aids:
    + dump the contents of a `BulletAppState` or `PhysicsSpace`
    + customize debug material per collision object
    + visualize the local axes, bounding box, and/or CCD swept sphere
      of collision objects
    + visualize physics in multiple viewports
    + optional high-resolution debug meshes for convex shapes
    + options to generate debug meshes that include normals (for shading)
 + all joints, shapes, and collision objects implement the `JmeCloneable`
   and `Comparable` interfaces
 + enable/disable a joint
 + single-ended joints
 + settable global default for collision margin
 + access more parameters of rigid bodies:
    + anisotropic friction
    + contact damping
    + contact stiffness
    + contact-processing threshold
    + deactivation time
    + linear factor
    + rolling friction
    + spinning friction
 + option to apply scaling with a `RigidBodyControl`

Some JME bugs that have been fixed in Minie:

 + 772 scale of a physics shape is applied 2x
 + 877 physics joints don't work unless both bodies are dynamic
 + 883 extra `physicsTick()` callbacks
 + 887 debug mesh ignores scaling of `CollisionShape`
 + 889 disabled physics control gets added to a physics space
 + 894 `setRestitutionOrthoLin()` sets wrong joint parameter
 + 901 collision margin initialized to 0
 + 911 sleeping-threshold setters have unexpected side effects
 + 913 missing implementation of `PhysicsJoint.finalizeNative()`
 + 917 `HingeJoint.read()` fails
 + 918 `getImpulseClamp()` returns the wrong value
 + 919 `UnsatisfiedLinkError` in `getLimitSoftness()`
 + 928 crash caused by too many parallel threads
 + 969 linear factors not cloned
 + 1029 sphere-sphere collisions not reported
 + 1037 performance issue with `HullCollisionShape`
 + 1058 crash while removing body from BroadphaseType.SIMPLE PhysicsSpace
 + 1060 doesn't implement `bt32BitAxisSweep3`

Some `jme3-bullet`/`jme3-jbullet` classes that Minie omits:

 + `CharacterControl`: use `MinieCharacterControl` or `BetterCharacterControl`
   instead, or else use `PhysicsCharacter` directly
 + `KinematicRagdollControl`, `HumanoidRagdollPreset`, and `RagdollPreset`:
   use `DynamicAnimControl` instead
 + `RagdollUtils`: not needed

Other important differences:

 + The default collision margin increased from 0 to 0.04 .
 + `RagdollCollisionListener` interface changed and moved
   from the `com.jme3.bullet.collision` package
   to the `com.jme3.bullet.animation` package.

<a name="downloads"/>

## Downloads

Newer releases (since v0.5.0) can be downloaded from
[GitHub](https://github.com/stephengold/Minie/releases).

Older releases (v0.1.1 through v0.4.5) can be downloaded from
[the Jme3-utilities Project](https://github.com/stephengold/jme3-utilities/releases).

Maven artifacts are available from
[JFrog Bintray](https://bintray.com/stephengold/jme3utilities).

<a name="conventions"/>

## Conventions

Package names begin with
`jme3utilities.minie.` (if Stephen Gold holds the copyright) or
`com.jme3.` (if the jMonkeyEngine Project holds the copyright).

The source code is compatible with JDK 7.

<a name="history"/>

## History

Most of Minie was originally forked from `jme3-bullet`,
a library in the [jMonkeyEngine Game Engine][jme].

From January 2018 to November 2018, Minie was a sub-project of
[the Jme3-utilities Project][utilities].

Since November 2018, the Minie Project has been an independent project at
[GitHub][minie].

The evolution of Minie is chronicled in
[its release notes](https://github.com/stephengold/Minie/blob/master/MinieLibrary/release-notes.md).

<a name="install"/>

## How to install the SDK and the Minie Project

### jMonkeyEngine3 (jME3) Software Development Kit (SDK)

Minie currently targets Version 3.2.3 of jMonkeyEngine.
You are welcome to use the Engine without also using the SDK, but I use the SDK,
and the following installation instructions assume you will too.

The hardware and software requirements of the SDK are documented on
[the JME wiki](https://jmonkeyengine.github.io/wiki/jme3/requirements.html).

 1. Download a jMonkeyEngine 3.2 SDK from
    [GitHub](https://github.com/jMonkeyEngine/sdk/releases).
 2. Install the SDK, which includes:
    + the engine itself,
    + an integrated development environment (IDE) based on NetBeans,
    + various plugins, and
    + the Blender 3D application.
 3. To open the Minie project in the IDE (or NetBeans), you will need the
    `Gradle Support` plugin.  Download and install it before proceeding.
    If this plugin isn't shown in the IDE's "Plugins" tool,
    you can download it from
    [GitHub](https://github.com/kelemen/netbeans-gradle-project/releases).
    You don't need this plugin if you merely want to use a pre-built Minie
    release in an Ant project.

### Source files

Clone the Minie repository using Git:

 1. Open the "Clone Repository" wizard in the IDE:
     + Menu bar -> "Team" -> "Git" -> "Clone..." or
     + Menu bar -> "Team" -> "Remote" -> "Clone..."
 2. For "Repository URL:" specify
    `https://github.com/stephengold/Minie.git`
 3. Clear the "User:" and "Password:" text boxes.
 4. For "Clone into:" specify a writable folder (on a local filesystem)
    that doesn't already contain "Minie".
 5. Click on the "Next >" button.
 6. Make sure the "master" remote branch is checked.
 7. Click on the "Next >" button again.
 8. Make sure the Checkout Branch is set to "master".
 9. Make sure the "Scan for NetBeans Projects after Clone" box is checked.
10. Click on the "Finish" button.
11. When the "Clone Completed" dialog appears, click on the "Open Project..."
    button.
12. Expand the root project node to reveal the sub-projects.
13. Select both sub-projects using control-click, then click on the
    "Open" button.

### Build the project

 1. In the "Projects" window of the IDE,
    right-click on the "MinieExamples" sub-project to select it.
 2. Select "Build".

<a name="add"/>

## How to add Minie to an existing project

Adding Minie to an existing JME3 project should be a simple 6-step process:

 1. Remove any existing physics libraries which might interfere with Minie.
 2. Add libraries to the classpath.
 3. Create, configure, and attach a `BulletAppState`,
    if the application doesn't already do so.
 4. Configure the `PhysicsSpace`,
    if the application doesn't already do so.
 5. Create physics controls, collision objects, and joints
    and add them to the `PhysicsSpace`,
    if the application doesn't already do so.
 6. Test and tune as necessary.

### Remove any existing physics libraries

Minie replaces (and is therefore incompatible with) the following
jMonkeyEngine libraries:

 + `jme3-bullet`
 + `jme3-bullet-native`
 + `jme3-jbullet`

Before adding Minie, you should remove these libraries from your project so
they won't interfere with Minie.

#### For Gradle projects

Look for artifacts with these names in the `dependencies` section
of your project's `gradle.build` file and remove them.

#### For Ant projects

Open the project's properties in the IDE (JME 3.2 SDK or NetBeans 8.2):

 1. Right-click on the project (not its assets) in the "Projects" window.
 2. Select "Properties to open the "Project Properties" dialog.
 3. Under "Categories:" select "Libraries".
 4. Click on the "Compile" tab.
 5. Look for libraries with these names in the "Compile-time Libraries"
    listbox.  Select them and click on the "Remove" button.
 6. Click on the "OK" button to exit the "Project Properties" dialog.

### Add libraries to the classpath

Minie comes pre-built as a single library that includes both Java classes
and native libraries.  The Minie library depends on the
jme3-utilities-heart library, which in turn depends on
the standard jme3-core library from jMonkeyEngine.

#### For Gradle projects

For projects built using Maven or Gradle, it is sufficient to specify the
dependency on the Minie library.  The build tools should automatically
resolve the remaining dependencies automatically.

Because Minie is not on JCenter yet, you must explicitly specify the
repository location:

    repositories {
        maven { url 'https://dl.bintray.com/stephengold/jme3utilities' }
        jcenter()
    }
    dependencies {
        compile 'jme3utilities:Minie:0.8.1'
    }

#### For Ant projects

For projects built using Ant, download the 2 non-standard
libraries from GitHub:

 + https://github.com/stephengold/Minie/releases/tag/0.8.1
 + https://github.com/stephengold/jme3-utilities/releases/tag/heart-2.26.0

You'll want both class JARs
and probably the `-sources` and `-javadoc` JARs as well.

Open the project's properties in the IDE (JME 3.2 SDK or NetBeans 8.2):

 1. Right-click on the project (not its assets) in the "Projects" window.
 2. Select "Properties to open the "Project Properties" dialog.
 3. Under "Categories:" select "Libraries".
 4. Click on the "Compile" tab.
 5. Add the `jme3-utilities-heart` class JAR:
    + Click on the "Add JAR/Folder" button.
    + Navigate to the "jme3-utilities" project folder.
    + Open the "heart" sub-project folder.
    + Navigate to the "build/libs" folder.
    + Select the "jme3-utilities-heart-2.26.0.jar" file.
    + Click on the "Open" button.
 6. (optional) Add JARs for javadoc and sources:
    + Click on the "Edit" button.
    + Click on the "Browse..." button to the right of "Javadoc:"
    + Select the "jme3-utilities-heart-2.26.0-javadoc.jar" file.
    + Click on the "Open" button.
    + Click on the "Browse..." button to the right of "Sources:"
    + Select the "jme3-utilities-heart-2.26.0-sources.jar" file.
    + Click on the "Open" button again.
    + Click on the "OK" button to close the "Edit Jar Reference" dialog.
 7. Similarly, add the `Minie` JAR(s).
 8. Click on the "OK" button to exit the "Project Properties" dialog.

#### Create, configure, and attach a BulletAppState

Strictly speaking, a `BulletAppState` isn't required for `Minie`, but
it does provide a convenient interface for configuring, accessing, and
debugging a `PhysicsSpace`.

If your application already has a `BulletAppState`, the code will probably
work fine with `Minie`.  If not, here is a snippet to guide you:

        BulletAppState bulletAppState = new BulletAppState();
        bulletAppState.setDebugEnabled(true); // default=false
        stateManager.attach(bulletAppState);

#### Configure the PhysicsSpace

Section to be written.

#### Create physics controls, collision objects, and joints

Section to be written.

#### Test and tune

Section to be written.

<a name="shape"/>

## Choosing a collision shape

Minie provides more than a dozen `CollisionShape` subclasses.
Because jMonkeyEngine models are composed of triangular meshes,
beginners are often tempted to use mesh-based shapes
(such as `GImpactCollisionShape`) for everything.
However, since mesh-based collision detection is CPU-intensive, primitive
convex shapes (such as boxes and spheres) are usually a better choice, even
if they don't match the model's shape exactly.
In particular, `CapsuleCollisionShape` is often used with humanoid models.

    if (the object isn't involved in collisions) {
        use an EmptyShape
    } else if (its shape can be approximated by an infinite plane) {
        use a PlaneCollisionShape
    } else if (its shape can be approximated by a triangle or a tetrahedron) {
        use a SimplexCollisionShape
    } else if (its shape can be approximated by a centered sphere) {
        use a SphereCollisionShape
    } else if (its shape can be approximated by a centered rectangular solid) {
        use a BoxCollisionShape
    } else if (its shape can be approximated by a centered capsule) {
        use a CapsuleCollisionShape
    } else if (its shape can be approximated by a centered cylinder) {
        use a CylinderCollisionShape
    } else if (its shape can be approximated by a centered cone) {
        use a ConeCollisionShape
    } else if (its shape can be approximated by an ellipsoid
                or an eccentric sphere
                or an eccentric capsule
                or the convex hull of multiple spheres) {
        use a MultiSphere
    } else if (its shape can be approximated by an eccentric rectangular solid
                or an eccentric cylinder
                or an eccentric cone
                or a combination of convex primitives) {
            use a CompoundCollisionShape
    } else if (the object does not move) {
        if (it is a 2-D heightfield) {
            use a HeightfieldCollisionShape
        } else {
            use a MeshCollisionShape
        }
    } else { // if the object moves
        if (its shape can be approximated by the convex hull of a mesh) {
            use a HullCollisionShape
        } else {
            use a GImpactCollisionShape
        }
    }

(Pseudocode adapted from the flowchart on page 13 of the [Bullet User Manual][manual].)

<a name="dac"/>

## An Introduction to DynamicAnimControl

The centerpiece of Minie is `DynamicAnimControl`, a new `PhysicsControl`.
Adding a `DynamicAnimControl` to an animated model provides ragdoll physics and
inverse kinematics.

Configuration of `DynamicAnimControl` mostly takes place before the `Control`
is added to a model `Spatial`.  Adding the `Control` to a `Spatial`
automatically creates the ragdoll, including rigid bodies and joints.
No ragdoll exists before the `Control` is added to a `Spatial`,
and removing a `Control` from its controlled `Spatial` destroys the ragdoll.

The controlled `Spatial` must include the model's `SkeletonControl`.
Usually this is the model's root `Spatial`, but not always.
For a very simple example, see
[HelloDac.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/HelloDac.java).

A model's ragdoll is composed of rigid bodies joined by 6-DOF joints.
Within the `Control`, each `PhysicsRigidBody` is represented by
a `PhysicsLink`, and the links are organized into a tree hierarchy.

`PhysicsLink` has 3 subclasses:

 + `BoneLink`: manages one or more bones in the model’s `Skeleton`.
   Each `BoneLink` has a parent link, to which it is jointed.
   Its parent may be another `BoneLink` or it may be a `TorsoLink`.
 + `TorsoLink`: is always the root of a link hierarchy,
   so it has no parent link.
   It manages all root bones in the model's `Skeleton`.  It also manages any
   `Skeleton` bones that aren't managed by a `BoneLink`.
 + `AttachmentLink`: manages a non-animated model that's
   attached to the main model by means of an attachment `Node`.
   An `AttachmentLink` cannot be the parent of a link.

The default constructor for `DynamicAnimControl` is configured to create a
ragdoll with no bone links, only a `TorsoLink`.
Before adding the `Control` to a `Spatial`, specify which `Skeleton` bones
should be linked, by invoking the `link()` method for each of those bones.

I recommend starting with a default `LinkConfig` and a generous range of motion
for each linked bone:

    dynamicAnimControl.link(boneName, new LinkConfig(), new RangeOfMotion(1f, 1f, 1f));

For a simple example, see
[HelloBoneLink.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/HelloBoneLink.java).

Alternatively, you can generate configuration code for a specific model using
the [DacWizard application][dacwizard], which uses animation data to estimate
the range of motion for each linked bone.

You probably don't want to link every `Bone` in the model's `Skeleton`.
For instance, if the model has articulated fingers, you probably want to link
the hand bones but not the individual finger bones.
Unlinked bones will be managed by the nearest linked ancestor `Bone`.
The `TorsoLink` will manage any bones for which no ancestor `Bone` is linked.
If you link too many bones, the ragdoll may become inflexible or jittery
due to collisions between rigid bodies that don't share a `PhysicsJoint`.

<a name="links"/>

## External links

  + [The Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
  + [The Physics section of the JME Wiki](https://wiki.jmonkeyengine.org/jme3/advanced/physics.html)

YouTube videos about Minie:

  + May 2019 teaser (squishy ball)
    [watch](https://www.youtube.com/watch?v=W3x4gdDn-Ko) (0:13)
    [source code](https://github.com/stephengold/Minie/blob/b1a83f8a6440d8374f09258c6b1d471279833cfa/MinieExamples/src/main/java/jme3utilities/minie/test/TestSoftBody.java)
  + April 2019 walkthru of the DacWizard application
    [watch](https://www.youtube.com/watch?v=iWyrzZe45jA) (8:12)
    [source code](https://github.com/stephengold/Minie/blob/master/DacWizard/src/main/java/jme3utilities/minie/wizard/DacWizard.java)
  + March 2019 short demo (IK for head/eye directions)
    [watch](https://www.youtube.com/watch?v=8zquudx3A1A) (1:25)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/WatchDemo.java)
  + March 2019 teaser (simulating buoyancy)
    [watch](https://www.youtube.com/watch?v=eq09m7pbk5A) (0:10)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/BuoyDemo.java)
  + February 2019 demo (simulating rope)
    [watch](https://www.youtube.com/watch?v=7PYDAyB5RCE) (4:47)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/RopeDemo.java)
  + December 2018 demo (inverse kinematics)
    [watch](https://www.youtube.com/watch?v=ZGqN9ZCCu-8) (6:27)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/BalanceDemo.java)
  + December 2018 teaser (inverse kinematics)
    [watch](https://www.youtube.com/watch?v=fTWQ9m47GIA) (0:51)
  + November 2018 demo (single-ended joints):
    [watch](https://www.youtube.com/watch?v=Mh9k5AfWzbg) (5:50)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/SeJointDemo.java)
  + November 2018 demo (`MultiSphere` shape):
    [watch](https://www.youtube.com/watch?v=OS2zjB01c6E) (0:13)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/MultiSphereDemo.java)
  + October 2018 demo (`DynamicAnimControl` ragdolls):
    [watch](https://www.youtube.com/watch?v=A1Rii99nb3Q) (2:49)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/TestDac.java)

[blender]: https://docs.blender.org "Blender Project"
[bsd3]: https://opensource.org/licenses/BSD-3-Clause "3-Clause BSD License"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[chrome]: https://www.google.com/chrome "Chrome"
[dacwizard]: https://github.com/stephengold/Minie/tree/master/DacWizard "DacWizard Application"
[elements]: http://www.adobe.com/products/photoshop-elements.html "Photoshop Elements"
[findbugs]: http://findbugs.sourceforge.net "FindBugs Project"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[jfrog]: https://www.jfrog.com "JFrog"
[jme]: http://jmonkeyengine.org  "jMonkeyEngine Project"
[makehuman]: http://www.makehumancommunity.org/ "MakeHuman Community"
[manual]: https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf "Bullet User Manual"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[nifty]: http://nifty-gui.github.io/nifty-gui "Nifty GUI Project"
[obs]: https://obsproject.com "Open Broadcaster Software Project"
[utilities]: https://github.com/stephengold/jme3-utilities "Jme3-utilities Project"
[vegdahl]: http://www.cessen.com "Nathan Vegdahl"
[winmerge]: http://winmerge.org "WinMerge Project"

<a name="acks"/>

## Acknowledgments

Like most projects, the Minie Project builds on the work of many who
have gone before.  I therefore acknowledge the following
artists and software developers:

+ Normen Hansen (aka "normen") for creating most of the `jme3-bullet` library
 (on which `Minie` is based) and also for helpful insights
+ Rémy Bouquet (aka "nehon") for co-creating
  `KinematicRagdollControl` (on which `DynamicAnimControl` is based)
  and also for many helpful insights
+ Jules (aka "dokthar") for creating the soft-body fork of jMonkeyEngine
  from which Minie's soft-body support is derived
+ Paul Speed, for helpful insights
+ "oxplay2", for reporting a `PhysicsRigidBody` bug and helping me pin it down.
+ [Nathan Vegdahl][vegdahl], for creating the Puppet model (used in
  the TestDac walkthru video)
+ plus the creators of (and contributors to) the following software:
    + the [Blender][] 3-D animation suite
    + the [Bullet][] real-time physics library
    + the [FindBugs][] source-code analyzer
    + the [Git][] revision-control system and GitK commit viewer
    + the [Google Chrome web browser][chrome]
    + the [Gradle][] build tool
    + the Java compiler, standard doclet, and runtime environment
    + [jMonkeyEngine][jme] and the jME3 Software Development Kit
    + the [Linux Mint][mint] operating system
    + LWJGL, the Lightweight Java Game Library
    + the [MakeHuman][] Community
    + the [Markdown][] document conversion tool
    + Microsoft Windows
    + the [NetBeans][] integrated development environment
    + the [Nifty][] graphical user interface library
    + [Open Broadcaster Software Studio][obs]
    + the PMD source-code analyzer
    + the [WinMerge][] differencing and merging tool

I am grateful to [JFrog][] and [Github][] for providing free hosting for the
Minie Project and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know so I can
correct the situation: sgold@sonic.net