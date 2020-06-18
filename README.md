<img height="150" src="https://i.imgur.com/YEPFEcx.png">

The [Minie Project][project] is about improving the integration of
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
 + [Tutorials](#tutorials)
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
 + collision shapes:
   + `MultiSphere` shapes based on `btMultiSphereShape`
   + `Box2dShape` shapes based on `btBox2dShape`
   + `Convex2dShape` shapes based on `btConvex2dShape`
   + `EmptyShape` shape based on `btEmptyShape`
 + debugging aids:
    + dump the contents of a `BulletAppState`, `PhysicsSpace`,
      `CollisionShape`, or `MultiBody`
    + visualize physics objects in multiple viewports
    + customize debug material per collision object
    + visualize the local axes, velocities, bounding boxes, CCD swept spheres,
      and gravity vectors of collision objects
    + visualize the children of compound collision shapes
    + optional high-resolution debug meshes for convex shapes
    + options to generate debug meshes that include indices,
      normals (for shading), and/or texture coordinates (for texturing)
 + all joints, shapes, collision objects, and multibodies
   implement the `JmeCloneable` and `Comparable` interfaces
 + enable/disable a `PhysicsJoint`
 + single-ended physics joints
 + ignore lists for collision objects
 + application-specific data for collision objects
 + access more parameters of rigid bodies, vehicles, characters, joints,
   collision shapes, contact/constraint solvers, etcetera
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
`com.jme3.`/`jme3test.` (if the jMonkeyEngine Project holds the copyright).

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
if it detects Minie's `com.jme3.bullet.util.NativeMeshUtil` class.
On Android platforms, the native library is loaded (if present)
during static initialization of the `JmeAndroidSystem` class.

Physics simulation is organized around collision objects
(instances of `PhysicsCollisionObject`)
that interact in the context of a collision space (`CollisionSpace`).
Collision objects can be soft (varying shape) or rigid (non-varying shape).
Rigid objects can be mobile (moving) or static (non-moving).
And mobile objects can be dynamic (moved by forces, torques, and impulses)
or kinematic (moved directly by external calculations).

In this document, collision spaces that simulate forces, torques, and impulses
are referred to as "physics spaces".

By themselves, collision objects are invisible,
while scene-graph spatials have no effect on physics.
To visualize a collision object, it must be associated
with one or more scene-graph spatial(s).
For debugging purposes, Minie can visualize
collision objects by auto-generating spatials for them.
For full-custom visualization, use a `PhysicsControl` to associate
one or more collision objects with a `Spatial`.

A collision object's location and orientation are described
in physics-space coordinates.
These typically correspond to world coordinates of the scene,
and the built-in debug visualization makes this assumption.
However, there may be good reasons
to scale the physics space relative to the scene
and use physics-space units (psu) that are distinct from world units (wu).

In each physics space, simulation occurs in discrete time steps,
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
Minie offers optional continuous collision detection (CCD)
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
   merging static bodies together and
   simulating only the most relevant moving bodies.
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
     + `git checkout -b latest 1.7.0`
   + using a web browser:
     + browse to [https://github.com/stephengold/Minie/releases/tag/1.7.0][latest]
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

### Customizing Minie builds

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

<a name="tutorials"/>
<a name="add"/>
<a name="rigidbody"/>
<a name="shape"/>
<a name="debugging"/>
<a name="new6dof"/>
<a name="dac"/>
<a name="detect"/>
<a name="softbody"/>

## Tutorials

 + [How to add Minie to an existing project](https://stephengold.github.io/Minie/minie/minie-library-tutorials/add.html)
 + [An introduction to rigid-body physics](https://stephengold.github.io/Minie/minie/minie-library-tutorials/rigidbody.html)
 + [Choosing collision shapes](https://stephengold.github.io/Minie/minie/minie-library-tutorials/shape.html)
 + [Debugging physics issues](https://stephengold.github.io/Minie/minie/minie-library-tutorials/debug.html)
 + [An introduction to New6Dof](https://stephengold.github.io/Minie/minie/minie-library-tutorials/new6dof.html)
 + [An introduction to DynamicAnimControl](https://stephengold.github.io/Minie/minie/minie-library-tutorials/dac.html)
 + [Collision detection](https://stephengold.github.io/Minie/minie/minie-library-tutorials/detect.html)
 + [An introduction to soft-body physics](https://stephengold.github.io/Minie/minie/minie-library-tutorials/softbody.html)

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
  + `NewtonsCradle`
    demonstrates dynamic restitution and point-to-point joints
  + `PoolDemo`
    demonstrates 3 kinds of dynamic friction
  + `RopeDemo`
    demonstrates simulation of ropes using `DynamicAnimControl`
  + `SeJointDemo`
    demonstrates various single-ended joints
  + `TargetDemo`
    demonstrates shooting balls at various targets
  + `TestDac`
    demonstrates `DynamicAnimControl` applied to various models
  + `TestDebugToPost`
    demonstrates debug visualization to a post `ViewPort`
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
Each includes a help node,
containing a brief description of each key's function.

For convenience, the mapping of keys to actions
is largely standardized.
In most demos:

 + the "H" key toggles the help node between minimized and full versions,
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

Some of the demos (such as `DropTest` and `TargetDemo`)
rely entirely on debug visualization to render the physics objects.
Others (such as `TestDac` and `PoolDemo`) use physics controls.
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
[antora]: https://antora.org/ "Antora Project"
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
[minie]: https://github.com/stephengold/Minie "Minie Repository"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[nifty]: http://nifty-gui.github.io/nifty-gui "Nifty GUI Project"
[obs]: https://obsproject.com "Open Broadcaster Software Project"
[profont]: http://tobiasjung.name/profont "ProFont Project"
[project]: https://stephengold.github.io/Minie "Minie Project"
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
    + the [Antora] static website generator
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