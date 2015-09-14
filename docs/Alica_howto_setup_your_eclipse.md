#Howto Setup your Eclipse CDT

1. At first you need a compiled catkin workspace, with the folders like src, devel, build properly set. See therefore [Framework](Alica_howto_get_the_framework_running.md)

2. Download the latest Eclipse CDT Edition from eclipse.org 

3. Execute the following command at the root of your catkin-workspace: 

	```
	catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE:STRING=Debug -D_ECLIPSE_VERSION=4.4
	```

4. Start eclipse from a console, in order to have the Environment Variables properly set. For the Environment Variables see [Framework](Alica_howto_get_the_framework_running.md) --> Step 7 

5. Import the project generated in step 3 by selecting "File-->Import-->Existing projects into workspace" and hit the "next"-button. Browse to your catkin workspace root directory and do not check "Copy projects into workspace". Then press the "finish"-button. 

6. You should be almost fine now. Usually the implicitly linked libraries like stdio, sstream, and so on are not included in your eclipse project at that point. In order to do so, go to the project properties (right-click the projects root folder and choose properties from the pop-up menu) and go to "C/C++ General-->Preprocessor Include Paths, Macros etc.", choose the provider tab on the right and check "CDT GCC Built-in Compiler Settings". Afterwards hit the "apply"-button. 

7. Another step is to set your eclipse to use the c++ 2011 standard. Therefore, you should navigate to the same window as in step 6. Under Providers select "CDT GCC Built-in Compiler Settings" and uncheck "Use global provider shared between projects" and add 
	```
	-std=c++11 
	```
at the end of "Command to get compiler specs:". Apply that... 

8. Furthermore you need to check, whether the **__cplusplus macro definition is present and if it is, whether it is set to 201103L**. Therefore go to the Project-Properties (as in Step 6.) and go to "C/C++ Include Paths and Symbols". Find the macro in the list on the right side of the window (if it is missing you are fine). Set __cplusplus to 201103L if necessary. 

9. Just to be sure, regenerate the index by "right-click on the project folder-->index-->rebuild". This will take a while and you can see the progress in the lower right edge of Eclipse. If you still have unresolved include errors or something similar in your error tab, double-click on those errors. Sometimes they vanish by open the corresponding files. 

10. Eclipse has extensive formatting configuration capabilities. To add our formatting profile to Eclipse, perform the following steps: 
	
	```
	Select Window -> Preferences -> C/C++ ->Code Style -> Formatter 
	Click Import... 
	Select the CNC_ROS_Format.xml from <path2cnc-msl>/configuration 
	Click ok 
	As you edit a file, Eclipse should use this new profile to format your code following our conventions. To reformat an entire file, select Edit -> Format (shortcut is Strg+Shift+F). 
	```

11. Insert the following line into your eclipse.ini file. The file is located in your eclipse installation folder. Please replace Firstname Lastname with your full name. 
	```
	-Duser.name=Firstname Lastname
	```

