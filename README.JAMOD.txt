To get jamod built/working on Linux:

1. Install RXTX
   The docs (http://rxtx.qbang.org/wiki/index.php/Download) say v2.0 supports
   the javax.comm package names (necessary for binary version of jamod), but
   that no longer seems to be the case.

   Install the 2.1.7 binary version:
   a. download/extract the tar file
   b. sudo cp RXTXcomm.jar /usr/lib/jvm/java-6-sun/jre/lib/ext
   c. sudo cp Linux/i686-unknown-linux-gnu/librxtxSerial.so
        /usr/lib/jvm/java-6-sun/jre/lib/i386

2. Build jamod
   jamod supports the RXTX gnu.io package names, but must be built to do so.
   The latest SNAPSHOT build doesn't include build.xml or build.properties,
   and there's no information about build steps.

   Instead, build the 1.2 version:
   a. download/extract the tar file (jamod-1.2rc1-src.tar.gz)
   b. edit build.properties so build.serial.gnu=true
   c. totally WHACK the build.xml rip out doc targets
   d. ant jar
   e. sudo cp build/jamod.jar /usr/lib/jvm/java-6-sun/jre/lib/ext

3. Build JamodTest
   If jamod.jar is copied to system location, this can be as easy as:

     javac JamodTest.java

   If not, just include jamod.jar in the classpath for javac, e.g.:

     javac -cp jamod.jar JamodTest.java
