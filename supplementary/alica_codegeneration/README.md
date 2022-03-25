# ALICA Codegeneration

This is the ALICA Codegeneration package.

## Build

You can run `build.sh` to automatically get dependencies and build the code.

## Manual Build

To build the ALICA codegeneration jar files, run `mvn -DskipTests=true install` inside the root folder of alica_codegeneration.

After building the jar files using maven, move the codegeneration jar file `alica_codegeneration/target/PlanDesignerFX-Codegeneration-0.1.1.X.jar` to `/alica/supplementary/alica_designer_runtime/codegen/`. Next, move the default plugin `alica_codegeneration/alica-plan-designer-fx-default-plugin/target/alica-plan-designer-fx-default-plugin-0.1.1-SNAPSHOT.jar` to `/alica/supplementary/alica_designer_runtime/codegen/plugins/`.

### Java

The only dependency, that you need to install manually is Java OpenJDK11 and Maven:

`sudo apt install -y openjdk-11-jdk maven`

Please check whether openjdk-11 is your active java installation on your system:

`java -version`

The output should say something like this:

`openjdk version "11.0.8" 2020-07-14`
`OpenJDK Runtime Environment (build 11.0.8+10-post-Ubuntu-0ubuntu118.04.1)`
`OpenJDK 64-Bit Server VM (build 11.0.8+10-post-Ubuntu-0ubuntu118.04.1, mixed mode, sharing)`

Otherwise you also might need to switch your active Java version with this command:

`sudo update-alternatives --config java`
