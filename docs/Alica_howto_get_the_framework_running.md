# alica
###ALICA (A Language for Interactive Cooperative Agents)

**Howto Get the Framework Running**

1. Install Ubuntu 14.04 LTS with a partition of at least 60 GB. * is one of {U, X, K}, where U and K are recommended. 

2. Download and install the following packages: 
	```
	sudo apt-get install git vim gitk meld bison re2c libode-dev libcgal-demo libcgal-dev gnuplot-qt libxv-dev libtbb-dev
	```
   Please install the following package in an extra step, as it produces collisions with gnuplot-qt otherwise: 
	```
	sudo apt-get install gnuplot-x11
	```
   Install now following packages
   
	```
	sudo apt-get install xsdcxx libxerces-c-dev freeglut3-dev libvtk5-dev libvtk5-qt4-dev
	```
	
3. Download and install ROS Indigo by typing the following lines one after another into a shell: 

	```
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	sudo apt-get update
	sudo apt-get install ros-indigo-desktop-full ros-indigo-qt-gui-core ros-indigo-qt-build
	sudo rosdep init
	rosdep update 
	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
	. ~/.bashrc
	sudo apt-get install python-rosinstall
	```

4. Create your own ROS catkin workspace (you can choose the place and name of your workspace as you like) 

	```
	mkdir -p <path2workspace>/<nameofworkspace>/src
	cd <path2workspace>/<nameofworkspace>/src
	catkin_init_workspace
	```

5. Checkout the source code from the different GIT repositories into your catkin workspace src folder. Have a look at github.com/carpe-noctem-cassel for a full list of all repositories. For the RoboCup people alica, alica-plan-designer, cnc-msl, and supplementary should be enough.

   Permission denied (publickey)? You may want to create SSH keys first. 

	```
	cd ~/cnws/src
	git clone git@github.com:carpe-noctem-cassel/alica.git
	git clone git@github.com:carpe-noctem-cassel/alica-plan-designer.git
	git clone git@github.com:carpe-noctem-cassel/supplementary.git
	git clone git@github.com:carpe-noctem-cassel/cnc-msl.git
	```
   For the RoboCup robot and its driver developers additionally cnc-msldriver is necessary. 

	```
	git clone git@github.com:carpe-noctem-cassel/cnc-msldriver.git
	```

6. Copy the file 'gitconfig' from '<path2workspace>/<nameofworkspace>/src/cnc-msl/configuration' to '~/.gitconfig' and edit enter your e-mail and name in the upper part of the file. 

	```
	cp ~/cnws/src/cnc-msl/configuration/gitconfig ~/.gitconfig
	vim ~/.gitconfig
	```

7. Set the environment variables of your bash by adding the following lines into your '~/.bashrc'-file. ROS already added its own environment variable into your '~/.bashrc'-file. The lines below have to be inserted below 'source /opt/ros/indigo/setup.bash'. The first line lets you source your workspace in every console/bash! So take care if you use multiple workspaces. Furthermore, does the next line work only if you have build your workspace once, because the first build will create the 'devel/setup.bash'-file. 

	**Be careful this step should be done by our team members**

	```
	source <path2workspace>/<nameofworkspace>/devel/setup.bash
	#this defines the domain you are working in
	export DOMAIN_FOLDER="<path2workspace>/<nameofworkspace>/src/cnc-msl"
	export DOMAIN_CONFIG_FOLDER="$DOMAIN_FOLDER/etc"
	#fancy prompt that also shows the current git branch
	export PS1='\[\033[01;32m\]\u@\h\[\033[01;34m\] \w \[\033[01;31m\]$(__git_ps1 "[%s]")\[\033[01;34m\]\$\[\033[00m\] '
	```

	**For all other**

	```
	source <path2workspace>/<nameofworkspace>/devel/setup.bash
	#this defines the domain you are working in
	export DOMAIN_FOLDER="<path2workspace>/<nameofworkspace>/src/<YourProjectName>"
	export DOMAIN_CONFIG_FOLDER="$DOMAIN_FOLDER/etc"
	#fancy prompt that also shows the current git branch
	export PS1='\[\033[01;32m\]\u@\h\[\033[01;34m\] \w \[\033[01;31m\]$(__git_ps1 "[%s]")\[\033[01;34m\]\$\[\033[00m\] '
	```

	* *DOMAIN_FOLDER* this is the path to your project.
	* *DOMAIN_CONFIG_FOLDER* the plans and behaviours will be located here. At this directory should be the Alica.conf and the Globals.conf. 
	* *Alica.conf* represents the configuration for the engine. [Example Alica.conf](Alica_alica_conf.md)
	* *Globals.conf*  role matching and team ids of the robots. [Example Globals.conf](Alica_globals_conf.md)
	* You can create your own confuguration files here too. The Syntax is:
	
	```
	[TopLevelSection]
		[Sub]
			angle = 134.37;
			speed = 10000;
		[!Sub]
	................................
	................................
	................................
	...........and so on............
	................................
	................................
	................................
	[!TopLevelSection]
	```

	To be able to access to your conf file you need a instance of the systemConfig, example:
	
	```
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	double angle = (*sc)["NameOfYourConfigFile"]->get<double>("TopLevelSection.Sub.angle", NULL);
	```

8. Compile your catkin workspace 

	```
	cd <path2workspace>/<nameofworkspace>
	catkin_make
	```


