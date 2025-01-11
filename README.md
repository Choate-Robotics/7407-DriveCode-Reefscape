
# 7407-DriveCode-Reefscape

Team 7407 Wired Boars Reefscape Robot Code

## File Tree:
```
7407-DriveCode-Reefscape
├── autonomous (Contains autonomous routines for robot)
├── command (Contains commands for command scheduling)
│   └── __init__.py
├── oi (Operator Interface)
│   ├── OI.py (Contains keymappings to commands)
│   └── keymap.py (Contains controller keymaps for each subsystem and controller)
├── sensors (Contains sensor classes)
│   └── __init__.py (Contains sensor classes)
├── subsystem (Contains subsystem classes)
│   └── __init__.py
├── tests (Contains custom tests for code verification)
│   └── __init__.py
│   └── conftest.py (Contains test fixtures)
│   └── test_examples.py (Contains test examples for writing tests)
├── utils (Contains utilities like optimizations, conversions)
│   └── __init__.py
├── .gitignore (Filters out unnecessary files, for example *.pyc)
├── README.md (This file)
├── constants.py (Variables held constant throughout code.)
├── config.py (Easy configurations for entire robot.)
├── pyproject.toml (DO NOT EDIT.)
├── robot.py (Central program, controls everything.)
└── robot_systems.py (Contains initialized sensors and subsystems)
```


## Getting Started:

Check out our documentation [here](https://choate-robotics.github.io/Programming-SOPs/).



<!-- You will need to have at least python 3.12 installed on your computer. 



### Directions

#### Clone the repository code onto your computer:

```

git clone https://github.com/Choate-Robotics/7407-DriveCode-Reefscape.git

```


#### Linux and Mac

You might have to replace "python" at the end with "python3" depending on how python is configured in your system.

#### Windows Powershell


### Deploying Code:
Connect to the robot's wifi.
``python -m robotpy deploy``
If absolutely necessary, use ``python -m robotpy deploy --skip-tests`` to avoid WPILib version issues on the robot.


## Best Practices

### Pre-Commit, Formatting

Make sure to run ```pre-commit install``` before your first commit. When you commit, pre-commit will automatically check all files you have staged using Flake8, Black, ISort, and other formatters.

- If the response contains an ERROR:

	- If the error response contains "Files were modified by this hook":
		- ``git add .``
		- ``git commit -m "Message"``
	- Otherwise, manually fix the issues outlined, re-stage your files ( ``git add.``) and recommit.

Do not forget to ``git add .`` before committing.

### Commenting
Comment, comment, comment!
 - Use block quotes to start any function with parameters, and every class's "\_\_init\_\_" function. Block quotes should contain:
	 - Summary
	 - Arguments, with types and descriptions
	 - Return description
	 There are many extensions to help with docstrings. Examples include:
		 - autoDocstring on VsCode
		 - On PyCharm
			 - Place your cursor over a function or class name.
			 - Alt-Enter
			 - Generate documentation string stub
 - Use single line comments for any function without parameters with a description of the function.
 - Use single line comments before any complex function to describe how it works, and to the right of any line or variable that is very complicated.
 - Use TODO comments freely.

### Adding libraries

**REVISE THIS WITH PYPROJECT.TOML**


### Committing, Pushing, and Pulling
To commit:
```
git add .
git commit -m "Message"
```
To push:
```
git push
```
To pull:
```
git fetch
git pull
```

### Branching
To branch, first make sure that all your local changes are committed. If you would like to abandon the changes, run ``git reset --hard``. Be very careful with resetting.
To branch: ``git branch {branch name}

Branch names are as follows:
 - Subsystem Initialization branch format: init/{subsystem}
	 - Example: init/shooter
	 - Example: init/drivetrain
 - Feature branch format: feat/{subsystems}/{feature}
	 - Example: feat/shooter/optimized_shooting
	 - Example: feat/intake-index/ejection
 - Fix branch format: fix/{subsystems}/issue
	 - Example: fix/camera_server/wrong_ports
	 - Example: fix/robot/network_loop_time
	 - Example: fix/sensors/clean_up
 - Competition branch format: comp/{competition}/day/{day}
	 - Example: comp/battlecry/day/0 (load_in, initial setup, configurations)
	 - Example: comp/hartford/day/1

### Pull Requests

When you have finished a feature, fix, or initialization, create a pull request. Pull requests should 
be created from your branch to the dev branch, and then when tested on the robot in the dev branch
code should then be merged into the main branch.

### Competition Exceptions
#### Pre-commits
 - To avoid frustration, please use ``git commit -m "{Message}" --no-verify``
### Debugging:
#### Logger
 - USE LOGGER! It makes it easier on everyone to debug.
#### Smart Dashboard/Shuffleboard
 - Shuffleboard is preferred over the Smart Dashboard and console for debugging. To use shuffleboard, just push a string, number, boolean, or similar value to the SmartDashboard using "wpilib.SmartDashboard.pushNumber ..." etc. The value is then accessible through ShuffleBoard.

## Resources
 - [RobotPy Documentation](https://robotpy.readthedocs.io/en/stable/) We love RobotPy!
 - [WPILib Documentation](https://docs.wpilib.org/en/stable/index.html) RobotPy is just a wrapper for the WPILib C++ Code. Most of the structure remains the same.
 - [Chief Delphi](https://www.chiefdelphi.com/) Many a sensor problem have been fixed by looking here.
 - [7407 DriveCode-2021-Python](https://github.com/Choate-Robotics/7407-DriveCode-2021-Python) Worlds level code! -->
