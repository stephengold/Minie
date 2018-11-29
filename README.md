<img height="150" src="https://i.imgur.com/YEPFEcx.png">

The [Minie Project][minie] is about improving the integration of
[Bullet Real-Time Physics][bullet] into the
[jMonkeyEngine Game Engine][jme].

It contains 2 sub-projects:

 1. MinieLibrary: the Minie runtime library (in Java)
 2. MinieExamples: demos, examples, and test software (in Java)

Summary of features:

 + `DynamicAnimControl` for ragdoll simulation:
    + set dynamic/kinematic mode per bone
    + attachment nodes
    + multiple options for bone mass, center, and shape
 + `MultiSphere` collision shapes based on `btMultiSphereShape`
 + enhanced visualization for debugging:
    + visualize in multiple viewports
    + hi-res debug meshes for convex shapes
    + customize debug material per collision object
    + debug meshes with normals
 + single-ended joints
 + ... etcetera
 + fixes for many issues
 + improved JavaDoc and inline documentation
 + decoupled from the jMonkeyEngine release cycle
 + demonstration apps provided
 + Java source code provided under FreeBSD license

## Contents of this document

 + [Downloads](#downloads)
 + [Conventions](#conventions)
 + [History](#history)
 + [How to install the SDK and the Minie Project](#install)
 + [How to add Minie to an existing project](#add)
 + [External links](#links)
 + [Acknowledgments](#acks)

<a name="downloads"/>

### Downloads

Older releases (v0.1.1 through v0.4.5) can be downloaded from
[the Jme3-Utilities Project](https://github.com/stephengold/jme3-utilities/releases).

Maven artifacts are available from
[JFrog Bintray](https://bintray.com/stephengold/jme3utilities).

<a name="conventions"/>

### Conventions

Package names begin with
`jme3utilities.minie` (if Stephen Gold holds the copyright) or
`com.jme3.bullet` (if the jMonkeyEngine Project holds the copyright).

The source code is compatible with JDK 7.

<a name="history"/>

### History

Since November 2018, the Minie Project has been hosted at
[GitHub][minie].

From January 2018 to November 2018, Minie was a sub-project of
[the Jme3-Utilities Project][utilities].

Most of Minie was originally forked from `jme3-bullet`,
a library in the [jMonkeyEngine Game Engine][jme].

<a name="install"/>

## How to install the SDK and the Minie Project

### jMonkeyEngine3 (jME3) Software Development Kit (SDK)

Minie currently targets Version 3.2.1 of jMonkeyEngine.
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
 3. To open the project in the SDK (or NetBeans), you will need the `Gradle
    Support` plugin.  Download and install it before proceeding.

### Source files

Clone the repository using Git:

 1. Open the Clone wizard in the IDE:
     + Menu bar -> "Team" -> "Remote" -> "Clone..."
 2. For "Repository URL:" specify
    `https://github.com/stephengold/Minie.git`
 3. Clear the "User:" and "Password:" text boxes.
 4. For "Clone into:" specify a writable folder (on a local filesystem)
    which doesn't already contain "Minie".
 5. Click on the "Next >" button.
 6. Make sure the "master" remote branch is checked.
 7. Click on the "Next >" button again.
 8. Make sure the Checkout Branch is set to "master".
 9. Make sure the "Scan for NetBeans Projects after Clone" box is checked.
10. Click on the "Finish" button.
11. When the "Clone Complete" dialog appears, click on the "Open Project..."
    button.
12. Expand the root project node to reveal the sub-projects.
13. Select both sub-projects using control-click, then click on the
    "Open" button.

### Build the project

 1. In the "Projects" window, right-click on the "MinieExamples" sub-project to
    select it.
 2. Select "Build".

<a name="add"/>

## How to add Minie to an existing project

Section to be written.

    repositories {
        maven { url 'https://dl.bintray.com/stephengold/jme3utilities' }
    }
    dependencies {
        compile 'jme3utilities:Minie:0.4.5'
    }

<a name="links"/>

## External links

  + November 2018 [MultiSphere demo video](https://www.youtube.com/watch?OS2zjB01c6E)
  + October 2018 [DynamicAnimControl demo video](https://www.youtube.com/watch?v=A1Rii99nb3Q)

[blender]: https://docs.blender.org "Blender Project"
[bsd3]: https://opensource.org/licenses/BSD-3-Clause "3-Clause BSD License"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[chrome]: https://www.google.com/chrome "Chrome"
[elements]: http://www.adobe.com/products/photoshop-elements.html "Photoshop Elements"
[findbugs]: http://findbugs.sourceforge.net "FindBugs Project"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[jfrog]: https://www.jfrog.com "JFrog"
[jme]: http://jmonkeyengine.org  "jMonkeyEngine Project"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[obs]: https://obsproject.com "Open Broadcaster Software Project"
[utilities]: https://github.com/stephengold/jme3-utilities "Jme3-Utilities Project"
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
+ Paul Speed, for helpful insights
+ [Nathan Vegdahl][vegdahl], for creating the Puppet model (used by `TestDac`)
+ the creators of (and contributors to) the following software:
    + the [Blender][] 3-D animation suite
    + the [Bullet][] real-time physics library
    + the [FindBugs][] source-code analyzer
    + the [Git][] revision-control system and GitK commit viewer
    + the [Google Chrome web browser][chrome]
    + the [Gradle][] build tool
    + the Java compiler, standard doclet, and runtime environment
    + jMonkeyEngine and the jME3 Software Development Kit
    + LWJGL, the Lightweight Java Game Library
    + the Markdown document conversion tool
    + Microsoft Windows
    + the [NetBeans][] integrated development environment
    + [Open Broadcaster Software Studio][obs]
    + the PMD source-code analyzer
    + the [WinMerge][] differencing and merging tool

I am grateful to [JFrog][] and [Github][] for providing free hosting for the
Minie Project and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know so I can
correct the situation: sgold@sonic.net