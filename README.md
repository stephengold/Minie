<img height="150" src="https://i.imgur.com/YEPFEcx.png" alt="Minie Project logo">

First one, refer to an <a href="https://github.com/stephengold/Minie"> origin of this fork </a>

<h1>This fork currently contains only one <a href=https://github.com/stephengold/Minie/commit/a69541ef4716236460510c8b2d92b13fb664739d>fix</h1>

To install this fix you need donwload .jar file in releases then put it in 'libs' folder of your root project,  then add these implementations to your build.gradle closure
```groovy

repositories {
    maven { url "https://jitpack.io" }
    mavenCentral()
    mavenLocal()
    flatDir {
        dirs('libs')
    }
}

  dependencies {
  implementation 'com.github.stephengold:Minie:9.0.1' //requried b'cuz it loads native libs , while my 9.0.2 doesn't.
  implementation 'com.github.stephengold:Minie:9.0.2-SNAPSHOT'
}

```
