set -e

# Change CWD to code_generation folder
cd "$(dirname "$0")"

# Install java and maven
sudo apt install -y openjdk-11-jdk maven

# Check java default is correct (https://stackoverflow.com/a/7335120/2015911)
JAVA_VER=$(java -version 2>&1 | grep -oP 'version "?(1\.)?\K\d+')
if [ "$JAVA_VER" -ge 11 ]; then
    echo "Java version ok"
else
    echo "Wrong default java version detected! Continuing in 5s..."
    sleep 5
    # exit 1
fi

# Compile and move JAR files
mvn -DskipTests=true install
rm ../alica_designer_runtime/codegen/*.jar ../alica_designer_runtime/codegen/plugins/*.jar
mv target/*.jar ../alica_designer_runtime/codegen/
mv alica-plan-designer-fx-default-plugin/target/*.jar ../alica_designer_runtime/codegen/plugins/
