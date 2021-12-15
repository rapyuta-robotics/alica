# Install Dependencies
sudo apt-get update
sudo apt-get -qq install -y clang-format-10

# Change to source directory.
cd $CI_SOURCE_PATH
ls -la

# This directory should have its own .clang-format config file
if [ ! -f .clang-format ]; then
    echo ".clang-format file not provided. Exiting"
    exit 1
fi

# Saving current state with git before running clang
git config --global user.email "dummy@example.com"
git config --global user.name "Dummy Name"
git add . && git commit -m "Dummy Commit"

# Run clang-format
echo "Running clang-format"
source ./format_cpp.sh

# Catch errors while formatting
if [ $? -ne 0 ]
then
  echo "Error in formatting"
  exit 1
fi

echo "Showing changes in code style:"
git diff-index HEAD

# Make sure no changes have occured in repo
if ! git diff-index --quiet HEAD --; then
    # changes
    echo "clang-format test failed: changes required to comply to formatting rules. See diff above.";
    exit 1 # error
fi

echo "Passed clang-format test"
exit 0
