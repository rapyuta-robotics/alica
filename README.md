# alica
###ALICA (A Language for Interactive Cooperative Agents)

**Howto Get the Framework Running**

1. Install Ubuntu 14.04 LTS with a partition of at least 60 GB. * is one of {U, X, K}, where U and K are recommended. 
2. Download and install the following packages, please install the gnuplot-x11 package in an extra step, as it produces collisions with gnuplot-qt otherwise: 
  * sudo apt-get install git vim gitk meld bison re2c libode-dev gnuplot-qt libxv-dev libtbb-dev
  * sudo apt-get install gnuplot-x11
  * sudo apt-get install xserver-xorg-dev-lts-utopic mesa-common-dev-lts-utopic libxatracker-dev-lts-utopic libopenvg1-mesa-dev-lts-utopic libgles2-mesa-dev-lts-utopic libgles1-mesa-dev-lts-utopic libgl1-mesa-dev-lts-utopic 
libgbm-dev-lts-utopic libegl1-mesa-dev-lts-utopic
  * Currently needed because of Cambada-Stuff: 
	- sudo apt-get install xsdcxx libxerces-c-dev freeglut3-dev libvtk5-dev libvtk5-qt4-dev
3. Download and install ROS Indigo by typing the following lines one after another into a shell: 

	* sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
	* wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	* sudo apt-get update
	* sudo apt-get install ros-indigo-desktop-full ros-indigo-qt-gui-core ros-indigo-qt-build
	* sudo rosdep init
	* rosdep update 
	* echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
	* . ~/.bashrc
	* sudo apt-get install python-rosinstall

4. Create your own ROS catkin workspace (you can choose the place and name of your workspace as you like) 

	* mkdir -p <path2workspace>/<nameofworkspace>/src
	* cd <path2workspace>/<nameofworkspace>/src
	* catkin_init_workspace

5. Checkout the source code from the different GIT repositories into your catkin workspace src folder. Please replace <project> of the following line with the repositories you need. At the moment there are cace, symrock, ice, alica, alica-plan-designer, alica-expansion-modules, cnc-msl, cnc-msldriver, cnc-turtlebots, cnc-naos, supplementary. For the **RoboCup people of Carpe Noctem Cassel** alica, alica-plan-designer, cnc-msl,alica-expansion-modules and supplementary should be enough. For the RoboCup robot and its driver developers additionally cnc-msldriver is necessary. **For all other** alica, alica-plan-designer and supplementary is enough. 

	* cd <path2workspace>/<nameofworkspace>/src
	* git clone git@github.com:carpe-noctem-cassel/<project>.git

6. Set the environment variables of your bash by adding the following lines into your '~/.bashrc'-file. ROS already added its own environment variable into your '~/.bashrc'-file. The lines below have to be inserted below 'source /opt/ros/indigo/setup.bash'. The first line lets you source your workspace in every console/bash! So take care if you use multiple workspaces. Furthermore, does the next line work only if you have build your workspace once, because the first build will create the 'devel/setup.bash'-file. 

	```
	source <path2workspace>/<nameofworkspace>/devel/setup.bash
	#this defines the domain you are working in
	export DOMAIN_FOLDER="<path2workspace>/<nameofworkspace>/src/cnc-msl"
	export DOMAIN_CONFIG_FOLDER="$DOMAIN_FOLDER/etc"
	#fancy prompt that also shows the current git branch
	export PS1='\[\033[01;32m\]\u@\h\[\033[01;34m\] \w \[\033[01;31m\]$(__git_ps1 "[%s]")\[\033[01;34m\]\$\[\033[00m\] '
	```
7. Compile your catkin workspace 
	* cd <path2workspace>/<nameofworkspace>
	* catkin_make



