<img height="150" src="https://i.imgur.com/YEPFEcx.png">

The [Minie Project][minie] is about improving the integration of
[Bullet real-time physics simulation][bullet]
and [Khaled Mamou's V-HACD Library][vhacd]
into the [jMonkeyEngine game engine][jme].

It contains 5 sub-projects:

 1. MinieLibrary: the Minie runtime library and its automated tests
 2. [DacWizard]: a GUI application to configure a ragdoll
 3. MinieExamples: demos, examples, tutorials, and non-automated test software
 4. MinieAssets: generate assets used in MinieExamples
 5. Jme3Examples: physics examples from jme3-examples

Complete source code (in Java) is provided under
[a BSD license][license].

<a name="toc"/>

## Contents of this document

 + [Why use Minie?](#why)
 + [Downloads](#downloads)
 + [Conventions](#conventions)
 + [Overview and design considerations](#overview)
 + [How to build Minie from source](#build)
 + [How to add Minie to an existing project](#add)
 + [Choosing collision shapes](#shape)
 + [Debugging physics issues](#debugging)
 + [An introduction to New6Dof](#new6dof)
 + [An introduction to DynamicAnimControl](#dac)
 + [Collision detection](#detect)
 + [An introduction to soft-body physics](#softbody)
 + [An overview of the demo applications](#demos)
 + [External links](#links)
 + [History](#history)
 + [Acknowledgments](#acks)

<a name="why"/>

## Why use Minie?

[jMonkeyEngine][jme] comes with 2 Bullet integration libraries.
Why use Minie instead of `jme3-bullet` or `jme3-jbullet`?

 + Minie has many more features. (See the feature list below.)
 + Minie fixes many bugs found in the jMonkeyEngine libraries.
 + Due to its shorter release cycle, future features and bug fixes
   will probably appear first in Minie.
 + Minie uses automated testing to reduce the risk of regressions and new bugs.
 + Minie's classes are better encapsulated, with fewer public/protected fields
   and less aliasing of small objects like vectors.  This reduces the risk
   of accidentally corrupting its internal data structures.
 + Minie validates method arguments.  This helps detect usage errors that
   can lead to subtle bugs.
 + Minie's source code is more readable and better documented.

Summary of added features:

 + Extensions to `DynamicAnimControl`
 + Soft-body simulation based on `btSoftBody` and `btSoftRigidDynamicsWorld`,
    including anchors and soft-body joints
 + Multi-body simulation based on `btMultiBody` and `btMultiBodyDynamicsWorld`
 + Convex decomposition of meshes using [Khaled Mamou's V-HACD Library][vhacd],
   including progress listeners
 + `New6Dof` physics joints based on `btGeneric6DofSpring2Constraint`
 + Alternative contact-and-constraint solvers based on `btDantzigSolver`,
   `btLemkeSolver`, `btSolveProjectedGaussSeidel`, and `btNNCGConstraintSolver`
 + `MultiSphere` collision shapes based on `btMultiSphereShape`
 + `Box2dShape` collision shapes based on `btBox2dShape`
 + `Convex2dShape` collision shapes based on `btConvex2dShape`
 + `EmptyShape` collision shape based on `btEmptyShape`
 + debugging aids:
    + dump the contents of a `BulletAppState`, `PhysicsSpace`,
      `CollisionShape`, or `MultiBody`
    + visualize physics objects in multiple viewports
    + customize debug material per collision object
    + visualize the local axes, bounding boxes, and/or CCD swept spheres
      of collision objects
    + visualize the children of compound collision shapes
    + optional high-resolution debug meshes for convex shapes
    + options to generate debug meshes that include indices,
      normals (for shading), and/or texture coordinates (for texturing)
 + all joints, shapes, collision objects, and multibodies
   implement the `JmeCloneable` and `Comparable` interfaces
 + enable/disable a `PhysicsJoint`
 + single-ended physics joints
 + access more parameters of rigid bodies, vehicles, characters, etcetera
 + option to apply scaling with a `RigidBodyControl`

Some `jme3-bullet`/`jme3-jbullet` classes that Minie omits:

 + `KinematicRagdollControl`, `HumanoidRagdollPreset`, and `RagdollPreset`:
   use `DynamicAnimControl` instead
 + `RagdollUtils`: not needed

Other important differences:

 + `PhysicsSpace.addAll()` and `PhysicsSpace.removeAll()` add/remove collision
   objects only; they do not add/remove joints.
 + `RagdollCollisionListener` interface changed and moved
   from the `com.jme3.bullet.collision` package
   to the `com.jme3.bullet.animation` package.

[Jump to table of contents](#toc)

<a name="downloads"/>

## Downloads

Newer releases (since v0.5.0) can be downloaded from
[GitHub](https://github.com/stephengold/Minie/releases).

Older releases (v0.1.1 through v0.4.5) can be downloaded from
[the Jme3-utilities Project](https://github.com/stephengold/jme3-utilities/releases).

Newer Maven artifacts (since v1.4.0) are available from
[JCenter](https://bintray.com/stephengold/com.github.stephengold/Minie).

Older Maven artifacts (v0.1.2 through v1.3.0) are available from
[JFrog Bintray](https://bintray.com/stephengold/jme3utilities/Minie).

[Jump to table of contents](#toc)

<a name="conventions"/>

## Conventions

Package names begin with
`jme3utilities.` (if Stephen Gold holds the copyright) or
`com.jme3.` or `jme3test.` (if the jMonkeyEngine Project holds the copyright).

Both the source code and the pre-built libraries are compatible with JDK 7.

[Jump to table of contents](#toc)

<a name="overview"/>

## Overview and design considerations

### The role of physics simulation in games

Most computer games don't require detailed physics simulation.

 + Canned animations usually suffice to illustrate characters walking,
   jumping, and fighting.
 + Detecting when a character enters a fixed zone
   or comes into range of another character is a simple geometric calculation,
   provided the zone or range has a box or sphere shape.
 + For outer-space games, the equations of motion (Newton's 3rd Law) are easily
   implemented from scratch.

Other games require physics simulation, either because detailed physics is
integral to gameplay (as in bowling or auto racing) or else to enhance the
verisimilitude of effects such as collapsing buildings and/or people.
For such games, a real-time physics library such as Minie should prove useful.

### How Minie works

Minie is based on the [Bullet Physics SDK][bullet]:
mature, open-source, 3-D, physics-simulation software,
released under a Zlib license.
Bullet is written in C++,
so Minie uses Java Native Interface to access Bullet objects and methods.
All C++ source code associated with Minie
(including glue code and a partial snapshot of the Bullet SDK)
resides in the [Libbulletjme] repository.

On desktop platforms, JMonkeyEngine automatically loads
the appropriate native library during `JmeDesktopSystem.initialize()`
when it detects Minie's `com.jme3.bullet.util.NativeMeshUtil` class.
On Android platforms, JMonkeyEngine tries to load
the appropriate native library
during static initialization of the `JmeAndroidSystem` class.

Physics simulation is organized around collision objects
(instances of `PhysicsCollisionObject`)
that interact in the context of a physics space (`PhysicsSpace`).
Collision objects can be soft (varying shape) or rigid (non-varying shape).
Rigid objects can be mobile (moving) or static (non-moving).
And moving objects can be dynamic (moved by forces and torques)
or kinematic (moved directly by external calculations).

By themselves, collision objects are invisible
and spatials have no effect on physics.
To visualize an object, it must be associated
with one or more scene-graph spatial(s).
For debugging purposes, Minie can visualize
collision objects by auto-generating spatials for them.
For full-custom visualization, use a `PhysicsControl` to associate
a collision object with a `Spatial`.

A collision object's location and orientation are described
in physics-space coordinates.
These typically correspond to world coordinates of the scene,
and the built-in debug visualization makes this assumption.
However, there may be good reasons
to scale the physics space relative to the scene
and use physics-space units (psu) that are distinct from world units (wu).

For each physics space, simulation occurs in discrete time steps,
which need not correspond to rendered frames.
Each time step consists of 4 phases:

 + forward dynamics part one,
   to apply known forces
   and predict the next position of each collision object
 + broadphase collision detection,
   to quickly determine (using axis-aligned bounding boxes)
   which objects (if any) might possibly collide
 + narrowphase collision detection,
   to compute actual contacts (if any) between between objects,
   and
 + forward dynamics part 2,
   to apply contact forces, solve constraints, and update positions.

To simplify the creation and management of physics spaces,
Minie provides app states.
`BulletAppState` is the simplest of these; it manages a single
space without any soft objects or multibodies.
Simulation of that space can take place on the render thread
or else on a dedicated physics thread.
Either way, the simulation attempts to synchronize to real time
after every rendered frame.
With `BulletAppState`, debug visualization can be enabled
(or disabled) by invoking `setDebugEnabled()`.

Normal collisions (between collision objects) are reported asynchronously
to listeners registered at the `PhysicsSpace`.
For fast-moving objects,
Minie offers optional continous collision detection (CCD)
using swept spheres;
such collisions are reported through those same listeners.

Dynamic rigid bodies "go to sleep" after 2 seconds of inactivity.

### Computational efficiency

The computational cost of collision detection grows rapidly with
the number of collision objects and the complexity of their shapes.
To simulate physics in real time, with modest CPUs,
it's vital to keep the physics simple:

 + Use very simple collision shapes (such as boxes, capsules, and spheres)
   wherever possible.
 + Minimize the number of collision objects by
   merging static objects together and
   simulating only the most relevant moving objects.
 + Minimize the number of nodes in each soft body.

### Scaling the world

For a physics simulation, it might seem natural to choose kilograms and meters
as the units of mass and distance, respectively.
However, this is not a requirement, and for many games,
MKS units are not the best choice.

Bullet documentation recommends that dynamic bodies have
masses as close as possible to 1.

Also, to improve the performance and reliability of collision detection,
Bullet applies a margin to most collision objects.
By default, this margin is 0.04 physics-space units (psu).
While the margin is configurable, Bullet documentation
recommends against doing so.
For some collision shapes, margin increases the effective size of the object
and distorts its effective shape.
For this reason, it's undesirable to have a collision object
with any radius smaller than about 0.2 psu.

On the other hand, dynamic bodies should not be made too large.
Dynamic bodies in forced contact tend to jiggle.
Jiggling is mostly noticeable for sharp-edged bodies (such as boxes)
resting on uneven surfaces, under high gravity.
The higher the gravity (in psu per second squared),
the shorter the simulation time step (in seconds) needs to be.
For efficient and realistic simulation of Earth-like gravity (9.8 m/s)
with the default margin (0.04 psu) and time step (0.0167 seconds),
the psu should be 0.3 meters or larger.
This puts a soft upper limit on the size (in psu) of dynamic bodies.

Since Minie's debug visualization assumes that physics coordinates are
equivalent to world coordinates, these recommendations could impact
model creation and scene-graph design.
Physics units should therefore be chosen with care,
preferably early in the development process.

[Jump to table of contents](#toc)

<a name="build"/>

## How to build Minie from source

 1. Install the build tools:
   + a Java Development Kit and
   + [Gradle]
 2. Download and extract the source code from GitHub:
   + using Git:
     + `git clone https://github.com/stephengold/Minie.git`
     + `cd Minie`
     + `git checkout -b latest 1.6.0`
   + using a web browser:
     + browse to [https://github.com/stephengold/Minie/releases/tag/1.6.0][latest]
     + follow the "Source code (zip)" link
     + save the ZIP file
     + unzip the saved ZIP file
     + `cd` to the extracted directory/folder
 3. Set the `JAVA_HOME` environment variable:
   + using Bash:  `export JAVA_HOME="` *path to your JDK* `"`
   + using Windows Command Prompt:  `set JAVA_HOME="` *path to your JDK* `"`
 4. Run the Gradle wrapper:
   + using Bash:  `./gradlew build`
   + using Windows Command Prompt:  `.\gradlew build`

After a successful build,
Maven artifacts will be found in `MinieLibrary/build/libs`.

You can install the Maven artifacts to your local cache:
 + using Bash:  `./gradlew :MinieLibrary:publishToMavenLocal`
 + using Windows Command Prompt:  `.\gradlew :MinieLibrary:publishToMavenLocal`

### Customizing Minie

By default, Minie's class JAR includes native libraries
for all the platforms Minie supports.
While this is convenient, it results in a large JAR
and (potentially) a bloated application.
If you build Minie from source,
you can customize it to include native libraries only for specific platforms.

To configure which native libraries will be included in the JAR,
edit the MinieLibrary/build.gradle script.
Look for the section where the `btf` variables are set.
It should look something like this:

        btfAndroid_ARM7 = 'ReleaseSp'
        btfAndroid_ARM8 = 'ReleaseSp'
        btfAndroid_X86 = 'ReleaseSp'
        btfAndroid_X86_64 = 'ReleaseSp'
        btfLinux32 = 'ReleaseSp'
        btfLinux64 = 'ReleaseSp'
        btfLinux_ARM64 = 'ReleaseSp'
        btfMacOSX32 = 'ReleaseSp'
        btfMacOSX64 = 'ReleaseSp'
        btfWindows32 = 'ReleaseSp'
        btfWindows64 = 'ReleaseSp'

For example, to include only the 64-bit Linux native library,
change the other `btf` variables to `''` and rebuild:

        btfAndroid_ARM7 = ''
        btfAndroid_ARM8 = ''
        btfAndroid_X86 = ''
        btfAndroid_X86_64 = ''
        btfLinux32 = ''
        btfLinux64 = 'ReleaseSp'
        btfLinux_ARM64 = ''
        btfMacOSX32 = ''
        btfMacOSX64 = ''
        btfWindows32 = ''
        btfWindows64 = ''

You can also customize Minie to include debug-enabled native libraries
for specific platforms:

        btfAndroid_ARM7 = ''
        btfAndroid_ARM8 = ''
        btfAndroid_X86 = ''
        btfAndroid_X86_64 = ''
        btfLinux32 = ''
        btfLinux64 = ''
        btfLinux_ARM64 = ''
        btfMacOSX32 = ''
        btfMacOSX64 = ''
        btfWindows32 = ''
        btfWindows64 = 'DebugSp'

Similarly, you can specify double-precision (Dp-flavored) native libraries
for specific platforms:

        btfAndroid_ARM7 = ''
        btfAndroid_ARM8 = ''
        btfAndroid_X86 = ''
        btfAndroid_X86_64 = ''
        btfLinux32 = ''
        btfLinux64 = 'ReleaseDp'
        btfLinux_ARM64 = ''
        btfMacOSX32 = ''
        btfMacOSX64 = 'ReleaseDp'
        btfWindows32 = ''
        btfWindows64 = 'ReleaseDp'

[Jump to table of contents](#toc)

<a name="add"/>

## How to add Minie to an existing project

Adding Minie to an existing JME3 project is a 6-step process:

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
 + `jme3-bullet-native-android`
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
and native libraries.
The Minie library depends on the
standard jme3-terrain library and the
Heart library, which in turn depends on
the standard jme3-core library from jMonkeyEngine.

#### For Gradle projects

For projects built using Maven or Gradle, it is sufficient to specify the
dependency on the Minie Library.  The build tools should automatically
resolve the remaining dependencies automatically.

    repositories {
        jcenter()
    }
    dependencies {
        compile 'com.github.stephengold:Minie:1.6.0'
    }

#### For Ant projects

For projects built using [Ant], download the Minie and Heart
libraries from GitHub:

 + https://github.com/stephengold/Minie/releases/tag/1.6.0
 + https://github.com/stephengold/Heart/releases/tag/5.2.1

You'll want both class jars
and probably the `-sources` and `-javadoc` jars as well.

Open the project's properties in the IDE (JME 3.2 SDK or NetBeans 8.2):

 1. Right-click on the project (not its assets) in the "Projects" window.
 2. Select "Properties to open the "Project Properties" dialog.
 3. Under "Categories:" select "Libraries".
 4. Click on the "Compile" tab.
 5. Add the `Heart` class jar:
    + Click on the "Add JAR/Folder" button.
    + Navigate to the download folder.
    + Select the "Heart-5.2.1.jar" file.
    + Click on the "Open" button.
 6. (optional) Add jars for javadoc and sources:
    + Click on the "Edit" button.
    + Click on the "Browse..." button to the right of "Javadoc:"
    + Select the "Heart-5.2.1-javadoc.jar" file.
    + Click on the "Open" button.
    + Click on the "Browse..." button to the right of "Sources:"
    + Select the "Heart-5.2.1-sources.jar" file.
    + Click on the "Open" button again.
    + Click on the "OK" button to close the "Edit Jar Reference" dialog.
 7. Similarly, add the `Minie` jar(s).
 8. Click on the "OK" button to exit the "Project Properties" dialog.

### Create, configure, and attach a BulletAppState

Strictly speaking, a `BulletAppState` isn't required for Minie, but
it does provide a convenient interface for configuring, accessing, updating,
and debugging a `PhysicsSpace`.

If your application already has a `BulletAppState`, the code will probably
work fine with Minie.
If not, here is a snippet to guide you:

        SoftPhysicsAppState bas = new SoftPhysicsAppState();
        stateManager.attach(bas);
        PhysicsSoftSpace physicsSpace = bas.getPhysicsSoftSpace();

If you don't need soft bodies, you can instantiate a `BulletAppState` directly:

        BulletAppState bas = new BulletAppState();
        stateManager.attach(bas);
        PhysicsSpace physicsSpace = bas.getPhysicsSpace();

Or if you need multibodies, instantiate a `MultiBodyAppState`:

        MultiBodyAppState bas = new MultiBodyAppState();
        stateManager.attach(bas);
        MultiBodySpace physicsSpace = bas.getMultiBodySpace();

(Minie doesn't support combining soft bodies and multibodies in
a single `PhysicsSpace` yet.)

By default, the physics simulation executes on the render thread.
To execute it on a dedicated thread, use:

        bas.setThreadingType(BulletAppState.ThreadingType.PARALLEL);

By default, simulation advances based on the time per frame (tpf)
reported by the renderer.
To advance the physics simulation at a different rate, use:

        bas.setSpeed(0.5f); // simulate physics at half speed

By default, a Dynamic Bounding-Volume Tree (DBVT) is used for broadphase
collision detection.
To specify a different data structure, use `setBroadphaseType()`:

        SoftPhysicsAppState bas = new SoftPhysicsAppState();
        bas.setBroadphaseType(PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        bas.setWorldMax(new Vector3f(1000f, 10f, 1000f));
        bas.setWorldMin(new Vector3f(-1000f, -10f, -1000f));
        stateManager.attach(bas);
        PhysicsSoftSpace physicsSpace = bas.getPhysicsSoftSpace();

By default, a Sequential Impulse (SI) solver is
used to resolve contacts and constraints.
To specify a different type of solver, invoke `setSolverType()`
before attaching the AppState:

        bas.setSolverType(SolverType.Dantzig);

Caveats:

 1. For soft-body simulations, SI is the only supported solver type.
 2. The NNCG solver doesn't support multibodies.

By default, debug visualization is disabled. To enable it, use:

        bas.setDebugEnabled(true);

Other `BulletAppState` configurables, used to customize debug visualization,
are described in the [Debug visualization tips and tricks](#visualization)
section of this document.

### Configure the PhysicsSpace

Attaching a `BulletAppState` instantiates a `PhysicsSpace` that
you can access immediately:

        PhysicsSpace space = bas.getPhysicsSpace();

`SoftPhysicsAppState` instantiates a `PhysicsSoftSpace`, which is a subclass:

        PhysicsSoftSpace space = bas.getPhysicsSoftSpace();

and `MultiBodyAppState` instantiates a `MultiBodySpace`:

        MultiBodySpace space = bas.getMultiBodySpace();

Physics simulation can run with a fixed time step or a variable time step.
The default configuration is a fixed time step of 1/60th of a second
with up to 4 time steps per frame.

To configure a variable time step with a maximum of 0.25 seconds:

        space.setMaxSubSteps(0);
        space.setMaxTimeStep(0.25f);

To configure a fixed time step of 0.01 second with up to 6 time steps per frame:

        space.setAccuracy(0.01f);
        space.setMaxSubSteps(6);

Note that `setAccuracy()` has no effect when `maxSubSteps==0`,
while `setMaxTimeStep()` has no effect when `maxSubSteps>0`.

The contact solver performs a fixed number of iterations per time step,
by default, 10.  For higher-quality simualtion, increase this number.  For
instance, to use 20 iterations:

        space.setSolverNumIterations(20);

Each `PhysicsBody` contains an acceleration vector
that determines the effect of gravity on that body.
In addition, each `PhysicsSpace` has a gravity vector,
which is applied to bodies as they are added to the space.
To simulate a zero-gravity environment,
set the gravity of the space to zero:

        space.setGravity(Vector3f.ZERO);

To simulate a non-uniform gravity field,
update the gravity of each body before each physics tick:

        public void prePhysicsTick(PhysicsSpace space, float timeStep) {
            Collection<PhysicsCollisionObject> pcos = space.getPcoList();
            for (PhysicsCollisionObject pco : pcos) {
                if (pco instanceof PhysicsBody) {
                    updateGravity((PhysicsBody) pco);
                }
            }
        }

### Other global configuration

By default, the native library prints a startup message to `System.out`.
Once the library is loaded (but not started) you can disable this message:

        NativeLibrary.setStartupMessageEnabled(false);

Once the native library is loaded,
you can test whether it uses double-precision arithmetic:

        boolean doublePrecision = NativeLibrary.isDoublePrecision();

You can also test whether it was built for debugging
(with assertions enabled, symbols not stripped,
and debug information generated):

        boolean debug = NativeLibrary.isDebug();

The default collision margin for new shapes is 0.04 physics-space units.
To configure a default margin of 0.1 psu:

        CollisionShape.setDefaultMargin(0.1f);

Note that the Bullet Manual advises against changing the default margin.

### Create physics controls, collision objects, and joints

You can create collision objects directly, using the constructors:

        float radius = 2f;
        CollisionShape sphere2 = new SphereCollisionShape(radius);
        PhysicsGhostObject ghost1 = new PhysicsGhostObject(sphere2);
        float mass = 1f;
        PhysicsRigidBody body1 = new PhysicsRigidBody(sphere2, mass);

or indirectly, by adding physics controls to scene-graph spatials:

        float radius = 2f;
        CollisionShape sphere2 = new SphereCollisionShape(radius);

        Node ghostNode1 = new Node("ghostNode1");
        GhostControl gc1 = new GhostControl(sphere2);
        ghostNode1.addControl(gc1);

        Node rigidNode1 = new Node("rigidNode1");
        float mass = 1f;
        RigidBodyControl rbc1 = new RigidBodyControl(sphere2, mass);
        rigidNode1.addControl(rbc1);

Either way, the object(s) won't be simulated unless added to a `PhysicsSpace`.
Also, note that collision shapes can be shared between objects.

To instantiate a static body, specify mass=0.

<a name="shape"/>

## Choosing collision shapes

Minie provides 16 `CollisionShape` subclasses:

  + `Box2dShape`
  + `BoxCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/My74h2Q.png">
  + `CapsuleCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/R9NMEwc.png">
  + `CompoundCollisionShape`:
    + constructed explicitly <img height="160" align="middle" src="https://i.imgur.com/gL6rgAA.png">
    + auto-generated by V-HACD <img height="160" align="middle" src="https://i.imgur.com/UqzsBvw.png">
  + `ConeCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/jZFVxQd.png">
  + `Convex2dShape`
  + `CylinderCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/ey249X8.png">
  + `EmptyShape`
  + `GImpactShape` <img height="160" align="middle" src="https://i.imgur.com/TOLRsig.png">
  + `HeightfieldCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/JwtpqOM.png">
  + `HullCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/Rf61rcw.png">
  + `MeshCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/glB3Ujk.png">
  + `MultiSphere` <img height="160" align="middle" src="https://i.imgur.com/nTZEc8C.png">
  + `PlaneCollisionShape`
  + `SimplexCollisionShape`:
     + with 3 vertices <img height="160" align="middle" src="https://i.imgur.com/9YIhjq0.png">
     + with 4 vertices <img height="160" align="middle" src="https://i.imgur.com/l1fYSfc.png">
  + `SphereCollisionShape` <img height="160" align="middle" src="https://i.imgur.com/OPYrxRe.png">

In general, use the simplest shape that yields the desired behavior.

### Limitations of particular shapes

#### Dynamic rigid bodies

Not all collision shapes are suitable for dynamic rigid bodies.
In particular, the following shapes are suitable ONLY
for kinematic (unaffected by forces) or static (non-moving) collision objects:

 + `Box2dShape`
 + `Convex2dShape`
 + `EmptyShape`
 + `HeightfieldCollisionShape`
 + `MeshCollisionShape`
 + `PlaneCollisionShape`
 + `SimplexCollisionShape` with 1-3 vertices

(Simplex shapes with 4 vertices are fine for dynamic rigid bodies.)

#### Precision and margins

Most collision shapes incorporate a margin.
According to the Bullet Manual, the purpose of margin is
"to improve performance and reliability of the collision detection."

While methods are provided to adjust margins, doing so is not recommended.

For certain shapes, margin increases the effective size of the collision object
and distorts its effective shape:

 + `ConeCollisionShape`
 + `Convex2dShape`
 + `HullCollisionShape`
 + `SimplexCollisionShape`

Margin also distorts a `CylinderCollisionShape`,
but its effect on size isn't monotonic.

Distortion due to margin is most noticeable for small shapes.

When using imprecise shapes, you can compensate somewhat for margin
by shrinking the shape's dimensions
(for a `ConeCollisionShape` or `CylinderCollisionShape`)
or moving its defining vertices inward
(for a `SimplexCollisionShape` or `HullCollisionShape`).

Another workaround would be to scale the physics space so that
the effects of margin become less obvious.

If these workarounds are impractical,
consider using a precise shape, one that isn't distorted by margin:

  + `Box2dShape`
  + `BoxCollisionShape`
  + `CapsuleCollisionShape`
  + `GImpactShape`
  + `HeightfieldCollisionShape`
  + `MeshCollisionShape`
  + `MultiSphere`
  + `PlaneCollisionShape`
  + `SphereCollisionShape`

#### Scaling

Some applications require collision shapes that are scalable
(can be dynamically shrunk or enlarged).
However, not all collision shapes can scale arbitrarily.
In particular,

 + `SimplexCollisionShape` doesn't support scaling at all;
   the only allowed scaling is (1,1,1).
 + `CapsuleCollisionShape`, `ConeCollisionShape`, and `SphereCollisionShape`
   support only uniform scaling, where all axes have the same scale factor.
   For instance, (0.2,0.2,0.2) or (9,9,9).
 + `CylinderCollisionShape` allows the height and base to scale independently,
   but the scale factors of both base axes must be equal.
   In other words, the cross section must remain circular.
   So (9,9,9) would be allowed for any cylinder,
   but (9,1,1) would be allowed only for cylinders
   where the local X axis is the height axis.

You can test at runtime whether a particular scaling
is applicable to particular shape:

    if (shape.canScale(newScale)) {
        shape.setScale(newScale);
    }

### An algorithm for choosing a shape

Because jMonkeyEngine models are composed of triangular meshes,
beginners are often tempted to use mesh-based shapes
(such as `GImpactCollisionShape`) for everything.
However, since mesh-based collision detection is CPU-intensive, primitive
convex shapes (such as boxes and spheres) are usually a better choice, even
if they don't match the model's shape exactly.
In particular, `CapsuleCollisionShape` is often used with humanoid models.

    if (the object doesn't move and isn't involved in collisions) {
        use an EmptyShape
    } else if (the object doesn't move and its shape can be approximated by an infinite plane) {
        use a PlaneCollisionShape
    } else if (the object doesn't move and its shape can be approximated by point, line segment, or triangle) {
        use a SimplexCollisionShape
    } else if (the object doesn't move and its shape can be approximated by rectangle) {
        use a Box2dShape
    } else if (the object doesn't move and its shape can be approximated by convex polygon) {
        use a Convex2dShape
    } else if (its shape can be approximated by a tetrahedron) {
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
        if (its shape can be approximated by a convex hull) {
            use a HullCollisionShape
        } else if (its shape can be decomposed into convex hulls) {
            use a compound of hull shapes
        } else {
            use a GImpactCollisionShape
        }
    }

(Pseudocode adapted from the flowchart on page 13 of the [Bullet User Manual][manual].)

Note that `GImpactCollisionShape` should be your last resort.

Minie has Khaled Mamou's Volumetric-Hierarchical Approximate Convex
Decomposition (V-HACD) algorithm built in.
V-HACD makes it easy to decompose any 3-D model into a
compound of hull shapes:

        VHACDParameters p = new VHACDParameters();
        CollisionShape shape
                = CollisionShapeFactory.createVhacdShape(modelRoot, p, null);

The V-HACD algorithm may be costly to run, but in many applications it can
be run during the build process and stored in J3O format.
At runtime, the resulting shape will be far more efficient
than a comparable `GImpactCollisionShape`.

[Jump to table of contents](#toc)

<a name="debugging"/>

## Debugging physics issues

When a physics simulation doesn't work as expected, debug visualization
(configured at the `BulletAppState`) should be enabled to uncover gross issues
such as incorrect collision shapes, incorrect initial positions,
bodies that have gone inactive, and objects not added to the `PhysicsSpace`.

If further details are desired, temporary print statements might be added
at key points.
To streamline this process, Minie provides a configurable dumper
for app states, physics spaces, collision shapes, multibodies,
viewports, and scene graphs.

<a name="visualization"/>

### Debug visualization tips and tricks

By default, debug visualization is disabled.
To enable it, configure the `BulletAppState`:

        bas.setDebugEnabled(true);

By default, debug visualization renders convex collision shapes using meshes
with up to 42 vertex locations.
It can also generate debug meshes with up to 256 vertex locations.
To override the low-resolution default on a per-object basis:

        collisionObject.setDebugMeshResolution(DebugShapeFactory.highResolution);

(This setting has no effect on objects with non-convex shapes.)

Debug visualization caches the mesh
for every non-compound collision shape it renders.
To clear this cache:

        DebugShapeFactory.clearCache();

By default, debug visualization renders only to the
application's main `ViewPort`.
To specify a different `ViewPort` (or an array of viewports) use:

        bas.setDebugViewPorts(viewPortArray);

For a very simple example, see
[TestDebugToPost](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/TestDebugToPost.java).

#### Customizing what is rendered

By default, debug visualization renders the shape of every
`PhysicsCollisionObject`, but not its bounding box nor its swept sphere.
To override these defaults, set filters to identify for which collision objects
each feature should be rendered:

        BulletDebugAppState.DebugAppStateFilter all = new FilterAll(true);
        BulletDebugAppState.DebugAppStateFilter none = new FilterAll(false);
        bas.setDebugBoundingBoxFilter(all); // all bounding boxes
        bas.setDebugFilter(none);           // no collision shapes
        bas.setDebugSweptSphereFilter(all); // all swept spheres

By default, debug visualization doesn't render the centers nor the local axes of
collision objects.
To override this default, increase the axis length to a positive value:

        bas.setAxisLength(1f);

If local axes are rendered, then by default the arrows are one pixel wide.
You can specify wider lines:

        bas.setDebugAxisLineWidth(3f); // axis arrows 3 pixels wide

or you can specify 3-D arrows:

        bas.setDebugAxisLineWidth(0f); // solid arrows

#### Customizing the materials

By default, Minie visualizes the shapes of collision objects
using single-sided wireframe materials:

 + yellow for any collision object without contact response,
   which includes any `PhysicsGhostObject`
 + magenta for a `PhysicsRigidBody` or `MultiBodyCollider`
   (with contact response) that's both dynamic and active
 + blue for a `PhysicsRigidBody` or `MultiBodyCollider`
   (with contact response) that's either static or kinematic or sleeping
 + pink for a `PhysicsCharacter` (with contact response)
 + red for a `PhysicsSoftBody` with faces
 + orange for a `PhysicsSoftBody` with links but no faces

Note that on Android platforms,
the default debug materials are solid colors instead of wireframes.

Some collision objects are best visualized using double-sided materials.
You can override the single-sided default on a per-object basis:

        collisionObject.setDebugNumSides(2);

Note that `setDebugNumSides(0)` makes the object's shape invisible
in debug visualization,
even if the object is selected by the debug filter.

If further customization is required, the debug material can be customized
on a per-object basis:

        collisionObject.setDebugMaterial(myMaterial);

Note that `setDebugNumSides()` has no effect on custom debug materials.

#### Customizing the meshes

The default debug materials
don't need lighting, normals, or texture coordinates.
By default, debug visualization doesn't provide these amenities.
However, a customized debug material might require them.

You can override the no-normals default on a per-object basis:

        collisionObject1.setDebugMeshNormals(DebugMeshNormals.Facet);
        collisionObject2.setDebugMeshNormals(DebugMeshNormals.Smooth);
        collisionObject2.setDebugMeshNormals(DebugMeshNormals.Sphere);

Generating index buffers for meshes usually reduces
the number of vertices that must be rendered.
However, generating index buffers for large meshes can take a long time.
By default, Minie doesn't generate index buffers for debug meshes
that have more than 6,000 vertices.

You can tune this threshold:

        DebugShapeFactorysetIndexBuffers(1000);

The threshold has no effect on debug meshes previously generated.
To make this setting retroactive, clear the cache.

#### Callbacks for further customization

`BulletAppState` invokes a callback during initialization.
You can use this callback to provide lighting for debug visualization:

        DebugInitListener callbackObject = new DebugInitListener() {
            public void bulletDebugInit(Node physicsDebugRootNode) {
                AmbientLight ambient = new AmbientLight(aColor);
                physicsDebugRootNode.addLight(ambient);
                DirectionalLight sun = new DirectionalLight(direction, dColor);
                physicsDebugRootNode.addLight(sun);
            }
        };
        bas.setDebugInitListener(callbackObject);

`BulletAppState` invokes a callback each time it generates a debug mesh.
You can use this callback to add texture coordinates to the mesh:

        DebugInitListener callbackObject = new DebugMeshInitListener() {
            public void debugMeshInit(Mesh debugMesh) {
                VertexBuffer pos = debugMesh.getBuffer(VertexBuffer.Type.Position);
                int numVertices = pos.getNumElements();
                FloatBuffer positions = (FloatBuffer) pos.getDataReadOnly();
                FloatBuffer uvs = BufferUtils.createFloatBuffer(2 * numVertices);
                // TODO: fill the uvs buffer with data
                debugMesh.setBuffer(VertexBuffer.Type.TexCoord, 2, uvs);
                uvs.flip();
            }
        };
        collisionObject.setDebugMeshInitListener(callbackObject);

### An introduction to PhysicsDumper

The following temporary statements could be used to dump
(to `System.out`) all collision objects in a `PhysicsSpace`:

        PhysicsDumper dumper = new PhysicsDumper();
        dumper.dump(physicsSpace);

Here is sample output for a space containing 2 rigid bodies and nothing else:

```text
PhysicsSoftSpace with 0 chars, 0 ghosts, 0 joints, 2 rigids, 0 softs, 0 vehicles
 bphase=DBVT grav[y=-30] timeStep[0.0166667 maxSS=4] listeners[c=0 cg=0 t=1]
 solver[SI iters=10 cfm=0 batch=128 mode=WarmStart,VelocityDependent,SIMD,Cone]
 rayTest=SubSimplex,HeightfieldAccel
 SbwInfo grav[y=-30] offset=0 norm[xyz=0] water=0 air=1.2 maxDisp=1000
  Rigid Sta loc[y=-20] fric=0.5
   Box he[xyz=20] marg=0.04
   with 0 joints
  Rigid Dyn(mass=1) user=Material loc[x=1.2788 y=1.17978 z=1.0783] orient[x=-0.251 y=-0.578 z=0.649 w=-0.426] fric=0.5
   v[x=-0.252605 y=0.232249 z=-0.550773] grav[y=-30] ccd[mth=5 r=2.02997] damp[l=0.6 a=0.6] sleep[lth=0.8 ath=1 time=0.0666667] moms[x=0.95612 y=1.8819 z=1.99822]
   MultiSphere r[1.09406] marg=0.04 scale[x=1.91423 y=1.15929 z=1.02588]
   with 0 joints
```

2-space indentation indicates the hierarchy of spaces/objects/joints.
Single-space indentation indicates additional description
of the foregoing object.

To dump a `PhysicsSpace` to a text file:

        PrintStream dumpStream = new PrintStream("dump.txt");
        PhysicsDumper dumper = new PhysicsDumper(dumpStream);
        dumper.dump(physicsSpace);

#### Customizing what is dumped

You can dump an entire `BulletAppState`, including its `PhysicsSpace`:

        dumper.dump(bulletAppState);

You can dump individual collision objects:

        dumper.dump(character);
        dumper.dump(collider);
        dumper.dump(ghostObject);
        dumper.dump(rigidBody);
        dumper.dump(softBody);

You can dump individual collision shapes:

        dumper.dump(collisionShape, "");

When dumping a `PhysicsSpace`,
the default is to describe every collision object;
physics joints are counted but not described.
To describe the joints in each body, configure the dumper like so:

        dumper.setEnabled(DumpFlags.JointsInBodies, true); // default=false

To describe the motors in each joint, configure the dumper like so:

        dumper.setEnabled(DumpFlags.Motors, true); // default=false

To dump just the physics joints (no collision objects):

        dumper.setEnabled(DumpFlags.Pcos, false); // default=true
        dumper.setEnabled(DumpFlags.JointsInSpaces, true); // default=false

When dumping a `PhysicsSpace`, you can apply a filter
to restrict which physics objects are listed.
For instance, to dump only those physics objects that lack a user object:

        String indent = "";
        BulletDebugAppState.DebugAppStateFilter noUser = new UserFilter(null);
        dumper.dump(physicsSpace, indent, noUser);

Other dump flags can be set, for instance,
to describe the nodes or clusters in each soft body
or the child shapes in each compound collision shape.

[Jump to table of contents](#toc)

<a name="new6dof"/>

## An introduction to New6Dof

A `PhysicsJoint` connects one physics body to another
(or to a fixed point in space), constraining how the body(s) can move.

For instance, a door might swing on hinges.
In simulation, the hinges would be represented by a joint
with a single degree of freedom (DOF): rotation around the axis of the hinges.

Or a door might slide along a track.
In that case, the track would be represented by a joint
that’s free only to translate along the axis of the track.

Or imagine a robot arm mounted on a ball-and-socket:
it can freely turn and twist to any imaginable orientation,
but the ball end can’t leave the socket. (This requires 3 degrees of freedom.)

`New6Dof` is a new physics joint intended to replace the older `SixDofJoint`
and `SixDofSpringJoint` joints found in jme3-bullet and jme3-jbullet.

### Features

`New6Dof` is a versatile physics joint with 3 rotation DOFs
 and 3 translation DOFs; it can potentially rotate or slide on any axis or axes.
Locking various DOFs allows it to simulate almost any kind of joint:

 + To simulate a swinging door, you’d lock all 3 translation DOFs
   and all but one of the rotation DOFs.
 + To simulate a sliding door, you’d lock all 3 rotation DOFs
   and all but one of the translation DOFs.
 + To simulate a ball-and-socket, you’d disable all 3 of the translation DOFs.

In addition to DOF locking, `New6Dof` also implements limits, springs, motors,
and servos:

 + Using limits, you can prevent a door from sliding or swinging
   beyond certain points.
 + Using a spring, you can make a door automatically return
   to a neutral position when you release it.
 + Using a motor, you can control the rate at which a door opens and closes.
 + Using a servo, you can make a robot arm turn smoothly from one
   orientation to another, as if under remote control.

A `New6Dof` can only join rigid bodies:
no ghost objects, characters, multibodies, or soft bodies.

### Coordinate systems and defaults

TODO tutorials and more info

[Jump to table of contents](#toc)

<a name="dac"/>

## An introduction to DynamicAnimControl

The centerpiece of Minie is `DynamicAnimControl`, a kind of `PhysicsControl`.
Adding a `DynamicAnimControl` to an animated model provides ragdoll physics and
inverse kinematics.
`DynamicAnimControl` can also be used to simulate ropes.

Configuration of `DynamicAnimControl` mostly takes place before the `Control`
is added to a model `Spatial`.  Adding the `Control` to a `Spatial`
automatically creates the ragdoll, including rigid bodies and joints.
No ragdoll exists before the `Control` is added to a `Spatial`,
and removing a `Control` from its controlled `Spatial` destroys the ragdoll.

The controlled `Spatial` must include the model's `SkeletonControl`.
Usually this is the model's root `Spatial`, but not always.
For a very simple example, see
[HelloDac.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/tutorial/HelloDac.java).

A model's ragdoll is composed of rigid bodies joined by physics joints.
Within the `Control`, each `PhysicsRigidBody` is represented by
a `PhysicsLink`, and the links are organized in a tree hierarchy.

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
[HelloBoneLink.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/tutorial/HelloBoneLink.java).

When you run `HelloBoneLink`, press the space bar to put the control into
dynamic mode.
You'll see the linked bones go limp while the remainder of the Ninja model
stays rigid.

As an alternative to hand-coding the control configuration,
you can generate configuration code for a specific model using
the [DacWizard application][dacwizard], which uses animation tracks to estimate
the range of motion for each linked bone.

You probably don't want to link every `Bone` in the model's `Skeleton`.
For instance, if the model has articulated fingers, you probably want to link
the hand bones but not the individual finger bones.
Unlinked bones will be managed by the nearest linked ancestor `Bone`.
The `TorsoLink` will manage any bones for which no ancestor `Bone` is linked.
If you link too many bones, the ragdoll may become inflexible or jittery
due to collisions between rigid bodies that don't share a `PhysicsJoint`.

[Jump to table of contents](#toc)

<a name="detect"/>

## Collision detection

Minie provides 4 collision-detection interfaces:

 1. You can add collision listeners to a `PhysicsSpace` to be notified about
    up to 4 collision contacts per colliding object, including references to
    both objects.
 2. You can add collision-group listeners to a `PhysicsSpace` to be notified
    about collisions involving particular groups.
 3. You can add ragdoll-collision listeners to any `DynamicAnimControl` to be
    notified about collision contacts involving its ragdoll where the applied
    impulse exceeds a certain threshold. (This is built atop interface #1.)
 4. You can invoke `getOverlappingObjects()` on any `PhysicsGhostObject` to
    enumerate all collision objects that overlap with it, based on
    axis-aligned bounding boxes.

Minie also provides ray tests and sweep tests.

[Jump to table of contents](#toc)

<a name="softbody"/>

## An introduction to soft-body physics

While rope, cloth, and foam rubber can be simulated using many small rigid
bodies, it is more convenient and efficient to treat them as individual bodies
that can be deformed.
To this end, Minie supports simulation of soft bodies in a manner
roughly analogous to that for rigid bodies:

 + In place of `BulletAppState`, use `SoftPhysicsAppState`.
 + In place of `PhysicsSpace`, use `PhysicsSoftSpace`.
 + In place of `PhysicsRigidBody`, use `PhysicsSoftBody`.
 + In place of `RigidBodyControl`, use `SoftBodyControl`.

`PhysicsSoftSpace` is a subclass of `PhysicsSpace`.
It implements soft-body physics in addition to all the
features of an ordinary `PhysicsSpace` (such as rigid bodies).

The abstract class `PhysicsBody` is a superclass of both `PhysicsRigidBody`
and `PhysicsSoftBody`.
It provides access to properties that rigid bodies and soft bodies
have in common, such as gravity, location, mass, and joints.

Soft bodies can collide with both rigid bodies and soft bodies.
They can also be joined to other bodies of both types, using special subclasses
of `PhysicsJoint`.

### A comparison of soft bodies and rigid bodies

Unlike a rigid body, a soft body doesn't have a `CollisionShape` or
an orientation.
Instead, it is composed of point masses (called "nodes") whose locations
are specified in physics-space coordinates.
A soft body's shape, structure, mass distribution, and position are all defined
by its mesh of nodes:

 + To simulate rope, nodes can be connected in pairs (called "links").
 + To simulate cloth, nodes can be connected to form triangles (called "faces").
 + To simulate foam rubber, nodes can be connected to form tetrahedra (also
   called "tetras").

(Soft-body nodes are entirely unrelated to `com.jme3.scene.Node`,
the kind of node found in the scene graph.)

Unlike a rigid body, the physics location of a soft body is not its center
of mass, but rather the center of its axis-aligned bounding box.

Like rigid bodies, soft bodies have collision margins.
However, since a soft body lacks a `CollisionShape`,
different accessors are used:

    float oldMargin = softBody.margin();
    softBody.setMargin(0.1f);

Soft bodies lack many other features of rigid bodies, including:

 + motion state (for extrapolating between time steps),
 + deactivation/sleeping (for efficient simulation), and
 + continuous collision detection (CCD) (for fast-moving objects).

Like rigid bodies, soft bodies can be constructed directly (using `new`)
or they can be created using physics controls (such as `SoftBodyControl`)
which tie them to particular spatials in the scene graph.
However, unlike a `RigidBodyControl`, a `SoftBodyControl` can only be
dynamic (spatial follows body) never kinematic (body follows spatial).

### Constructing a soft body

To construct a soft body directly, start with the null constructor:

    PhysicsSoftBody softBody = new PhysicsSoftBody()

This produces an empty body (one without any nodes, links, faces, tetras,
or joints) that isn't added to any physics space.

Methods are provided to append nodes, links, and faces to a soft body.
However, it's often more convenient to generate a `com.jme3.scene.Mesh`
(the same kind of mesh used in scene-graph geometries)
with the desired shape and topology and append it to the body
using a utility method:

 + `NativeSoftBodyUtil.appendFromTriMesh()`
   to append nodes and faces from a mesh with Mode.Triangles
 + `NativeSoftBodyUtil.appendFromLineMesh()`
   to append nodes and links from a mesh with Mode.Lines

Be aware that meshes intended for graphics rendering often prove
unsuitable for soft-body simulation.
For instance, they may define multiple vertices at the same position
or their edges/faces may be insufficiently subdivided.

To construct a soft body using `SoftBodyControl`, instantiate a control
and add it to a `Geometry`:

    SoftBodyControl sbc = new SoftBodyControl();
    geometry.addControl(sbc);

Access the newly-constructed `PhysicsSoftBody` using `sbc.getBody()`.

If you add the control to a scene-graph `Node` instead of a `Geometry`,
it will traverse the node's subtree and use the first `Geometry` it finds.

### Soft-body configuration and pose matching

Each soft body has numerous properties that can affect its behavior.
Most of these are stored in its configuration object, which can be
accessed using `getSoftConfig()`.
Soft bodies and configuration objects are one-to-one.

Configuration properties with `float` values are enumerated
by the `Sbcp` ("soft-body configuration parameter") enum.
For instance, a soft body can have a preferred shape (called its "default pose")
that it tends to return to when deformed.
The strength of this tendency depends on the configuration object's
"pose matching" parameter, which defaults to zero.

For a simple example using `SoftPhysicsAppState`, `SoftBodyControl`, and
pose matching, see
[HelloSoftBody.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/tutorial/HelloSoftBody.java).

### Soft-soft collisions

By default, collisions between soft bodies are not handled (ignored).
One way to handle soft-soft collisions for a specific body is to
set the `VF_SS` collision flag in its configuration object:

    SoftBodyConfig config = softBody.getSoftConfig();
    int oldFlags = config.getCollisionFlags();
    config.setCollisionFlags(oldFlags, ConfigFlag.VF_SS);

For a simple example of a collision between 2 soft bodies, see
[HelloSoftSoft.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/tutorial/HelloSoftSoft.java).

### Solver iterations

During every time step, the simulator applies a series of
iterative solvers to each soft body:

 + a cluster solver
 + a drift solver
 + a position solver
 + a velocity solver

The number of iterations for each solver is stored in the body's
configuration object.
When simulating collisions, you can often improve accuracy by increasing the
number of position-solver iterations:

    SoftBodyConfig config = softBody.getSoftConfig();
    config.setPositionIterations(numIterations);  // default=1

### Stiffness coefficients

Each soft body has 3 stiffness coefficients.
These are stored in its "material" object,
which can be accessed using `getSoftMaterial()`.
Soft bodies and their material objects are one-to-one.
(Soft-body materials are unrelated to `com.jme3.material.Material`,
the kind of material used to render geometries.)

To simulate an object that flexes easily (such as cloth), create a soft
body with many faces and set its angular-stiffness coefficient
to a small value (such as zero):

    PhysicsSoftBody.Material softMaterial = softBody.getSoftMaterial();
    softMaterial.setAngularStiffness(0f); // default=1

For a simple example of cloth simulation, see
[HelloCloth.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/tutorial/HelloCloth.java).

### Mass distribution

When a node is appended to a soft body, it has mass=1.
To alter the mass of a pre-existing node, use the `setNodeMass()` method:

    softBody.setNodeMass(nodeIndex, desiredMass);

You can also alter the total mass of a soft body, distributing the mass across
the pre-existing nodes in various ways:
  + in proportion to the current mass of each node, using `setMassByCurrent()`,
  + in proportion to the area of adjacent faces, using `setMassByArea()`, or
  + in a custom fashion, using `setMasses()`.

`softBody.setMass()` is equivalent to `setMassByCurrent()`.

If a soft-body node has mass=0, it becomes pinned/immovable, like a static
`PhysicsRigidBody`.

For a simple example of a pinned node, see
[HelloPin.java](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/tutorial/HelloPin.java).

TODO: ropes, applying forces, anchors, soft joints, world info, aerodynamics

### Clusters

By default, soft-body collisions are handled using nodes and faces.
As an alternative, they can be handled using groups of connected nodes
(called "clusters").
To enable cluster-based rigid-soft collisions for a specific soft body,
set its `CL_RS` collision flag.
To enable cluster-based soft-soft collisions, set its `CL_SS` flag.

Clusters can overlap, but they can't span multiple bodies.
In other words, a single node can belong to multiple clusters,
but a single cluster can't contain nodes from multiple bodies.

When a soft body is created, it doesn't have any clusters.
Once nodes are appended to a body, clusters can be generated automatically,
using an iterative algorithm that's built into Bullet:

    softBody.generateClusters(k, numIterations);

[Jump to table of contents](#toc)

<a name="demos"/>

## An overview of the demo applications

Demo applications have been created to showcase certain features of Minie.
The following demos are found in the `jme3utilities.minie.test` package of
the MinieExamples sub-project:

  + `BalanceDemo`
    demonstrates models that balance their weight between 2 feet
  + `BuoyDemo`
    demonstrates ragdolls with buoyancy
  + `DropTest` (also known as `MultiSphereDemo`)
    demonstrates falling rigid bodies with various shapes
  + `ForceDemo`
    demonstrates forces, torques, and impulses applied in zero gravity
  + `JointDemo`
    demonstrates a crawling robot made of boxes and 6-DOF joints
  + `RopeDemo`
    demonstrates simulation of ropes using `DynamicAnimControl`
  + `SeJointDemo`
    demonstrates various single-ended joints
  + `TestDac`
    demonstrates `DynamicAnimControl` applied to various models
  + `TestSoftBody`
    demonstrates soft-body physics without `SoftBodyControl`
  + `TestSoftBodyControl`
    demonstrates soft-body physics with `SoftBodyControl`
  + `WatchDemo`
    demonstrates head/eye inverse kinematics for various models

(Not all applications in the package are intended as demos;
those not listed above are primarily for testing purposes.)

For many of the demos, video walkthrus are available from YouTube.

The demos are controlled by primarily by keyboard input.
At startup, a help node is displayed,
containing a brief description of each key's function.

For convenience, the mapping of keys to actions
is largely standardized.
For instance, in all 11 demos:

 + the "H" key toggles visibility of help node,
 + F5 toggles visibility of the render-statistics overlay,
 + the "O" key dumps the physics space,
 + the "C" key dumps the camera's position, and
 + the Escape key ends the application.

For camera control, all demos use
the standard `FlyByCamera` with `setDragToRotate(true)`.
This means you can rotate the camera
by dragging with the left mouse button (LMB).
Furthermore:

 + the "W" and "S" keys dolly the camera forward and back, respectively,
 + the "A" and "D" keys dolly the camera left and right, respectively,
 + the "Q" and up-arrow keys raise the camera, and
 + the "Z" and down-arrow keys lower the camera.

Some of the demos (such as `DropTest`)
rely entirely on debug visualization to render the physics objects.
Others (such as `TestDac`) use physics controls.
When physics controls are in use,
the "/" key to toggles debug visualization on and off.

[Jump to table of contents](#toc)

<a name="links"/>

## External links

  + [the Minie Physics Library page](https://jmonkeystore.com/38308161-c3cf-4e23-8754-528ca8387c11)
    at [the JmonkeyStore](https://jmonkeystore.com/)
  + [The Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
  + [The Physics section of the JME Wiki](https://wiki.jmonkeyengine.org/jme3/advanced/physics.html)

YouTube videos about Minie:

  + June 2019 teaser #2 (rubber duck)
    [watch](https://www.youtube.com/watch?v=GKc-_SqcpZo) (0:16)
    [source code](https://github.com/stephengold/Minie/blob/d0326f636dbed76c809cb8ec654bfaf94786e988/MinieExamples/src/main/java/jme3utilities/minie/test/TestSoftBodyControl.java)
  + June 2019 teaser #1 (jogger in skirt)
    [watch](https://www.youtube.com/watch?v=lLMBIASzAAM) (0:24)
    [source code](https://github.com/stephengold/Minie/blob/40add685ec9243c3fa1e10f8b38b805a04a32863/MinieExamples/src/main/java/jme3utilities/minie/test/TestSoftBody.java)
  + May 2019 teaser #3 (wind-blown flag)
    [watch](https://www.youtube.com/watch?v=7dcBr0j6sKw) (0:06)
    [source code](https://github.com/stephengold/Minie/blob/9fb33ce21c5082af36ce2969daa79d63b57c0641/MinieExamples/src/main/java/jme3utilities/minie/test/TestSoftBody.java)
  + May 2019 teaser #2 (squishy ball and tablecloth)
    [watch](https://www.youtube.com/watch?v=-ECGEe4CpcY) (0:12)
    [source code](https://github.com/stephengold/Minie/blob/fe55f9baf83158d6347f765b4ff6bbf892056919/MinieExamples/src/main/java/jme3utilities/minie/test/TestSoftBody.java)
  + May 2019 teaser #1 (squishy ball)
    [watch](https://www.youtube.com/watch?v=W3x4gdDn-Ko) (0:13)
    [source code](https://github.com/stephengold/Minie/blob/b1a83f8a6440d8374f09258c6b1d471279833cfa/MinieExamples/src/main/java/jme3utilities/minie/test/TestSoftBody.java)
  + April 2019 walkthru of the DacWizard application
    [watch](https://www.youtube.com/watch?v=iWyrzZe45jA) (8:12)
    [source code](https://github.com/stephengold/Minie/blob/master/DacWizard/src/main/java/jme3utilities/minie/wizard/DacWizard.java)
  + March 2019 short demo (IK for head/eye directions)
    [watch](https://www.youtube.com/watch?v=8zquudx3A1A) (1:25)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/WatchDemo.java)
  + March 2019 teaser (buoyancy)
    [watch](https://www.youtube.com/watch?v=eq09m7pbk5A) (0:10)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/BuoyDemo.java)
  + February 2019 demo (ropes)
    [watch](https://www.youtube.com/watch?v=7PYDAyB5RCE) (4:47)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/RopeDemo.java)
  + December 2018 demo (inverse kinematics)
    [watch](https://www.youtube.com/watch?v=ZGqN9ZCCu-8) (6:27)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/BalanceDemo.java)
  + December 2018 teaser (inverse kinematics)
    [watch](https://www.youtube.com/watch?v=fTWQ9m47GIA) (0:51)
  + November 2018 demo (single-ended joints)
    [watch](https://www.youtube.com/watch?v=Mh9k5AfWzbg) (5:50)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/SeJointDemo.java)
  + November 2018 demo (`MultiSphere` shape)
    [watch](https://www.youtube.com/watch?v=OS2zjB01c6E) (0:13)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/MultiSphereDemo.java)
  + October 2018 demo (`DynamicAnimControl` ragdolls)
    [watch](https://www.youtube.com/watch?v=A1Rii99nb3Q) (2:49)
    [source code](https://github.com/stephengold/Minie/blob/master/MinieExamples/src/main/java/jme3utilities/minie/test/TestDac.java)

[ant]: https://ant.apache.org "Apache Ant Project"
[blender]: https://docs.blender.org "Blender Project"
[bsd3]: https://opensource.org/licenses/BSD-3-Clause "3-Clause BSD License"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[chrome]: https://www.google.com/chrome "Chrome"
[dacwizard]: https://github.com/stephengold/Minie/tree/master/DacWizard "DacWizard Application"
[dokthar]: https://github.com/dokthar/jmonkeyengine "Dokthar's fork of JMonkeyEngine"
[elements]: http://www.adobe.com/products/photoshop-elements.html "Photoshop Elements"
[findbugs]: http://findbugs.sourceforge.net "FindBugs Project"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[heart]: https://github.com/stephengold/Heart "Heart Project"
[imgur]: https://imgur.com/ "Imgur"
[jfrog]: https://www.jfrog.com "JFrog"
[jme]: https://jmonkeyengine.org  "jMonkeyEngine Project"
[jme-ttf]: http://1337atr.weebly.com/jttf.html "jME-TTF Rendering System"
[latest]: https://github.com/stephengold/Minie/releases/tag/1.6.0 "latest release"
[libbulletjme]: https://github.com/stephengold/Libbulletjme "Libbulletjme Project"
[license]: https://github.com/stephengold/Minie/blob/master/LICENSE "Minie license"
[log]: https://github.com/stephengold/Minie/blob/master/MinieLibrary/release-notes.md "release log"
[makehuman]: http://www.makehumancommunity.org/ "MakeHuman Community"
[manual]: https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf "Bullet User Manual"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[meld]: http://meldmerge.org/ "Meld Tool"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[nifty]: http://nifty-gui.github.io/nifty-gui "Nifty GUI Project"
[obs]: https://obsproject.com "Open Broadcaster Software Project"
[profont]: http://tobiasjung.name/profont "ProFont Project"
[utilities]: https://github.com/stephengold/jme3-utilities "Jme3-utilities Project"
[vegdahl]: http://www.cessen.com "Nathan Vegdahl"
[vhacd]: https://github.com/kmammou/v-hacd "V-HACD Library"
[vhacdBindings]: https://github.com/riccardobl/v-hacd-java-bindings "V-HACD Java Bindings Project"
[wes]: https://github.com/stephengold/Wes "Wes Project"
[winmerge]: http://winmerge.org "WinMerge Project"

[Jump to table of contents](#toc)

<a name="history"/>

## History

The evolution of the project is chronicled in [its release log][log].

Most of Minie was originally forked from `jme3-bullet`,
a library in the [jMonkeyEngine Game Engine][jme].

From January to November 2018, Minie was a sub-project of
[the Jme3-utilities Project][utilities].

Since November 2018, Minie has been a separate project at
[GitHub][minie].

[Jump to table of contents](#toc)

<a name="acks"/>

## Acknowledgments

Like most projects, the Minie Project builds on the work of many who
have gone before.  I therefore acknowledge the following
artists and software developers:

+ Normen Hansen (aka "normen") for creating most of the `jme3-bullet` library
 (on which Minie is based) and also for helpful insights
+ Rémy Bouquet (aka "nehon") for co-creating
  `KinematicRagdollControl` (on which `DynamicAnimControl` is based)
  and also for many helpful insights
+ Jules (aka "dokthar") for creating [the soft-body fork of jMonkeyEngine][dokthar]
  from which Minie's soft-body support is derived
+ Khaled Mamou for creating and licensing the [V-HACD Library][vhacd]
  for decomposing meshes into convex hulls
+ Riccardo Balbo (aka "riccardo") for creating and licensing
  the [V-HACD Java Bindings Project][vhacdBindings]
+ "ndebruyn" for early testing of Minie on Android platforms
+ Adam T. Ryder (aka "tryder") for creating and licensing
  the [jME-TTF] rendering system
+ Paul Speed, for helpful insights
+ "oxplay2", for reporting a `PhysicsRigidBody` bug and helping me pin it down
+ [Nathan Vegdahl][vegdahl], for creating the Puppet model
+ Tobias Jung, for distributing [ProFont]
+ plus the creators of (and contributors to) the following software:
    + the [Blender] 3-D animation suite
    + the [Bullet] real-time physics library
    + the [FindBugs] source-code analyzer
    + the [Git] revision-control system and GitK commit viewer
    + the [Firefox] and [Google Chrome][chrome] web browsers
    + the [Gradle] build tool
    + the Java compiler, standard doclet, and virtual machine
    + [jMonkeyEngine][jme] and the jME3 Software Development Kit
    + the [Linux Mint][mint] operating system
    + LWJGL, the Lightweight Java Game Library
    + the [MakeHuman] Community
    + the [Markdown] document-conversion tool
    + the [Meld] visual merge tool
    + Microsoft Windows
    + the [NetBeans] integrated development environment
    + the [Nifty] graphical user-interface library
    + [Open Broadcaster Software Studio][obs]
    + the PMD source-code analyzer
    + [ProFont], the programmers' font
    + the [WinMerge] differencing and merging tool

I am grateful to [Github], [JFrog], and [Imgur]
for providing free hosting for this project
and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know so I can
correct the situation: sgold@sonic.net

[Jump to table of contents](#toc)