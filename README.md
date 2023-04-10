<img height="150" src="https://i.imgur.com/YEPFEcx.png" alt="Minie Project logo">

The [Minie Project][project] is about improving the integration of
[Bullet real-time physics simulation][bullet]
and [Khaled Mamou's V-HACD Library][vhacd] into
[the jMonkeyEngine (JME) game engine][jme].

It contains 8 sub-projects:

 1. MinieLibrary: the Minie runtime library and its automated tests
 2. [DacWizard]: a GUI application to configure a ragdoll
 3. VhacdTuner: a GUI application to tune the V-HACD algorithm for a particular mesh
 4. TutorialApps: tutorial apps
 5. MinieExamples: demos, examples, and non-automated test software
 6. MinieAssets: generate assets used in MinieExamples
 7. MinieDump: a command-line utility to dump J3O assets
 8. Jme3Examples: physics examples from jme3-examples

Complete source code (in Java) is provided under
[a 3-clause BSD license][license].


<a name="toc"></a>

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


<a name="why"></a>

## Why use Minie?

[jMonkeyEngine][jme] comes with `jme3-jbullet`,
its own Bullet integration library.
Why use Minie instead of `jme3-jbullet`?

 + Minie has many more features. (See the feature list below.)
 + Minie fixes many bugs found in `jme3-jbullet`.
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

Some jme3-jbullet classes that Minie omits:

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


<a name="downloads"></a>

## Downloads

Newer releases (since v0.5.0) can be downloaded from
[GitHub](https://github.com/stephengold/Minie/releases).

Older releases (v0.1.1 through v0.4.5) can be downloaded from
[the Jme3-utilities Project](https://github.com/stephengold/jme3-utilities/releases).

Maven artifacts since v3.1.0 are available from
[MavenCentral](https://central.sonatype.com/search?q=Minie&namespace=com.github.stephengold).

[Jump to table of contents](#toc)


<a name="conventions"></a>

## Conventions

Package names begin with `jme3utilities.`
(if Stephen Gold holds the copyright) or
`com.jme3.`/`jme3test.` (if the jMonkeyEngine Project holds the copyright).

The source code and pre-built libraries are compatible with JDK 8.

[Jump to table of contents](#toc)


<a name="overview"></a>

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

[How Minie works](https://stephengold.github.io/Minie/minie/implementation.html)

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

Dynamic bodies in forced contact tend to jiggle.
Jiggling is mostly noticeable for sharp-edged bodies (such as boxes)
resting on uneven surfaces, under high gravity.
The higher the gravity (in psu per second squared),
the shorter the simulation time step (in seconds) needs to be.
For efficient and realistic simulation of Earth-like gravity (9.8 m/s)
with the default margin (0.04 psu) and time step (0.0167 seconds),
the psu should be 0.3 meters or larger.
This puts a soft lower limit on the dimensions (in psu) of dynamic bodies.

Since Minie's debug visualization assumes that physics coordinates are
equivalent to world coordinates, these recommendations could impact
model creation and scene-graph design.
Physics units should therefore be chosen with care,
preferably early in the development process.

[Jump to table of contents](#toc)


<a name="build"></a>

## How to build Minie from source

[How to build Minie from source](https://stephengold.github.io/Minie/minie/build.html)

[Jump to table of contents](#toc)

<a name="tutorials"></a>
<a name="add"></a>
<a name="rigidbody"></a>
<a name="shape"></a>
<a name="debugging"></a>
<a name="new6dof"></a>
<a name="dac"></a>
<a name="detect"></a>
<a name="softbody"></a>

## Tutorials

 + [How to add Minie to an existing project](https://stephengold.github.io/Minie/minie/minie-library-tutorials/add.html)
 + [An introduction to rigid-body physics](https://stephengold.github.io/Minie/minie/minie-library-tutorials/rigidbody.html)
 + [Choosing collision shapes](https://stephengold.github.io/Minie/minie/minie-library-tutorials/shape.html)
 + [Debugging physics issues](https://stephengold.github.io/Minie/minie/minie-library-tutorials/debug.html)
 + [An introduction to New6Dof](https://stephengold.github.io/Minie/minie/minie-library-tutorials/new6dof.html)
 + [An introduction to DynamicAnimControl](https://stephengold.github.io/Minie/minie/minie-library-tutorials/dac.html)
 + [Collision detection](https://stephengold.github.io/Minie/minie/minie-library-tutorials/collision.html)
 + [An introduction to soft-body physics](https://stephengold.github.io/Minie/minie/minie-library-tutorials/softbody.html)

[Jump to table of contents](#toc)


<a name="demos"></a>

## An overview of the demo applications

[An overview of the demo applications](https://stephengold.github.io/Minie/minie/demos.html)

[Jump to table of contents](#toc)


<a name="links"></a>

## External links

+ [the Minie Physics Library page](https://library.jmonkeyengine.org/#!entry=11511%2F38308161-c3cf-4e23-8754-528ca8387c11)
  in [the JmonkeyEngine Library][library]
+ [The Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
+ [The physics section of the jMonkeyEngine Wiki](https://wiki.jmonkeyengine.org/docs/3.4/physics/physics.html)
+ [Alan Chou's game-physics tutorial](http://allenchou.net/game-physics-series/)
+ [V-HACD v4 slideshow](https://docs.google.com/presentation/d/1OZ4mtZYrGEC8qffqb8F7Le2xzufiqvaPpRbLHKKgTIM)

YouTube videos about Minie:

  + September 2022 teaser (splitting rigid bodies in real time)
    [watch](https://www.youtube.com/watch?v=9IsCSgoKJeE) (0:53)
    [source code](https://github.com/stephengold/Minie/blob/fd0a61d2d24f354e0a9418cfc904c5985b69cfd8/MinieExamples/src/main/java/jme3utilities/minie/test/SplitDemo.java)
  + August 2022 walkthru of the VhacdTuner application
    [watch](https://www.youtube.com/watch?v=iEWJtPujmM8) (7:45)
    [source code](https://github.com/stephengold/Minie/blob/e1b7781fd06d8060ab96928379509a732fd9398f/VhacdTuner/src/main/java/jme3utilities/minie/tuner/VhacdTuner.java)
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

[Jump to table of contents](#toc)


<a name="history"></a>

## History

The evolution of this project is chronicled in
[its release log][log].

Most of Minie was originally forked from `jme3-bullet`,
a library in the [jMonkeyEngine Game Engine][jme].

From January to November 2018, Minie was a sub-project of
[the Jme3-utilities Project][utilities].

Since November 2018, Minie has been a separate project, hosted at
[GitHub][minie].

[Jump to table of contents](#toc)


<a name="acks"></a>

## Acknowledgments

Like most projects, the Minie Project builds on the work of many who
have gone before.  I therefore acknowledge the following
artists and software developers:

+ Normen Hansen (aka "normen") for creating most of the `jme3-bullet` library
  (on which Minie is based) and also for helpful insights
+ Rémy Bouquet (aka "nehon") for co-creating
  `KinematicRagdollControl` (on which `DynamicAnimControl` is based),
  for creating the Jaime model, and also for many helpful insights
+ Jules (aka "dokthar") for creating [the soft-body fork of jMonkeyEngine][dokthar]
  from which Minie's soft-body support is derived
+ Khaled Mamou for creating and licensing the [V-HACD Library][vhacd]
  for decomposing meshes into convex hulls
+ Riccardo Balbo (aka "riccardo") for creating and licensing
  the [V-HACD Java Bindings Project][vhacdBindings]
+ "ndebruyn" for early testing of Minie on Android platforms
+ Pavly Gerges (aka "Pavl_G") for testing Minie on Raspberry Pi
+ Adam T. Ryder (aka "tryder") for creating and licensing
  the [jME-TTF] rendering system
+ [Paul Speed (aka "pspeed42")][pspeed], for creating the [SimMath library][simMath]
+ "oxplay2", for reporting a `PhysicsRigidBody` bug and helping me pin it down
+ "duncanj", for pull request #15
+ "qwq", for suggesting changes to how rigid-body contacts are managed
  and for authoring the `ConveyorDemo` application
+ [Nathan Vegdahl][vegdahl], for creating the Puppet model
+ Tobias Jung, for distributing [ProFont]
+ plus the creators of (and contributors to) the following software:
    + the [Antora] static website generator
    + the [Blender] 3-D animation suite
    + the [Bullet] real-time physics library
    + the [Checkstyle] tool
    + the [FindBugs] source-code analyzer
    + the [Firefox] and [Google Chrome][chrome] web browsers
    + the [Git] revision-control system and GitK commit viewer
    + the [GitKraken] client
    + the [GNU Project Debugger][gdb]
    + the [Gradle] build tool
    + the [IntelliJ IDEA][idea] and [NetBeans] integrated development environments
    + the [Java] compiler, standard doclet, and runtime environment
    + [jMonkeyEngine][jme] and the jME3 Software Development Kit
    + the [Linux Mint][mint] operating system
    + LWJGL, the Lightweight Java Game Library
    + the [MakeHuman] 3-D character creation tool
    + the [Markdown] document-conversion tool
    + the [Meld] visual merge tool
    + Microsoft Windows
    + the [Nifty] graphical user-interface library
    + [Open Broadcaster Software Studio][obs]
    + the PMD source-code analyzer
    + [ProFont], the programmers' font
    + the [WinMerge] differencing and merging tool

I am grateful to [GitHub], [Sonatype], [JFrog],
[Travis], [MacStadium], [YouTube], and [Imgur]
for providing free hosting for this project
and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know, so I can
correct the situation: sgold@sonic.net

[Jump to table of contents](#toc)


[antora]: https://antora.org/ "Antora Project"
[blender]: https://docs.blender.org "Blender Project"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[checkstyle]: https://checkstyle.org "Checkstyle"
[chrome]: https://www.google.com/chrome "Chrome"
[dacwizard]: https://github.com/stephengold/Minie/tree/master/DacWizard "DacWizard Application"
[dokthar]: https://github.com/dokthar/jmonkeyengine "Dokthar's fork of JMonkeyEngine"
[elements]: https://www.adobe.com/products/photoshop-elements.html "Photoshop Elements"
[findbugs]: http://findbugs.sourceforge.net "FindBugs Project"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[gdb]: https://www.gnu.org/software/gdb/ "GNU Project Debugger"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gitkraken]: https://www.gitkraken.com "GitKraken client"
[gradle]: https://gradle.org "Gradle Project"
[heart]: https://github.com/stephengold/Heart "Heart Project"
[idea]: https://www.jetbrains.com/idea/ "IntelliJ IDEA"
[imgur]: https://imgur.com/ "Imgur"
[java]: https://java.com "Java"
[jfrog]: https://www.jfrog.com "JFrog"
[jme]: https://jmonkeyengine.org  "jMonkeyEngine Project"
[jme-ttf]: https://1337atr.weebly.com/jttf.html "jME-TTF Rendering System"
[library]: https://library.jmonkeyengine.org "jMonkeyEngine Library"
[license]: https://github.com/stephengold/Minie/blob/master/LICENSE "Minie license"
[log]: https://github.com/stephengold/Minie/blob/master/MinieLibrary/release-notes.md "release log"
[macstadium]: https://www.macstadium.com/ "MacStadium"
[makehuman]: http://www.makehumancommunity.org/ "MakeHuman Community"
[manual]: https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf "Bullet User Manual"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[meld]: https://meldmerge.org "Meld merge tool"
[minie]: https://stephengold.github.io/Minie/minie/overview.html "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[nifty]: https://nifty-gui.github.io/nifty-gui "Nifty GUI Project"
[obs]: https://obsproject.com "Open Broadcaster Software Project"
[profont]: https://tobiasjung.name/profont "ProFont Project"
[project]: https://stephengold.github.io/Minie "Minie Project"
[simMath]: https://github.com/Simsilica/SimMath "SimMath Library"
[sonatype]: https://www.sonatype.com "Sonatype"
[travis]: https://travis-ci.com "Travis CI"
[utilities]: https://github.com/stephengold/jme3-utilities "Jme3-utilities Project"
[vegdahl]: https://www.cessen.com "Nathan Vegdahl"
[vhacd]: https://github.com/kmammou/v-hacd "V-HACD Library"
[vhacdBindings]: https://github.com/riccardobl/v-hacd-java-bindings "Riccardo's V-hacd-java-bindings Project"
[wes]: https://github.com/stephengold/Wes "Wes Project"
[winmerge]: https://winmerge.org "WinMerge Project"
[youtube]: https://www.youtube.com/ "YouTube"
