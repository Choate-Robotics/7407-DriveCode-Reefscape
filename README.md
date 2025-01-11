
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
├── poetry.lock (DO NOT EDIT. Use "poetry add {package}" to add packages.)
├── pyproject.toml (DO NOT EDIT.)
├── robot.py (Central program, controls everything.)
└── robot_systems.py (Contains initialized sensors and subsystems)
```


## Getting Started:

Check out our documentation [here](https://choate-robotics.github.io/Programming-SOPs/).



<!-- You will need to have at least python 3.10 installed on your computer. 


### Apple Silicon ARM Mac

If you have an Apple Silicon ARM mac, at some point you will be asked to intall Rosetta, you will be asked to install it at some point. It is required, so go ahead when it asks.

If you want to know if Rosetta is installed you can look at the folder ```usr/libexec/Rosetta```. If there is any files in this folder then you have Rosetta installed.

You will need to intall the intel x64 version python 3.10 on your computer. 

#### Install Homebrew

You will need to install homebrew first.

```
arch -x86_64 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

```

#### Install Python
All the intel versions of all software on the Mac is in the ```/usr/local/bin``` folder. You will need to be in that directory to run any of the intel versions on your Mac. After getting into ```/usr/local/bin``` then use homebrew to install at least python 3.10.
```

arch -x86_64 ./brew install python@3.10

```

If you want a convenient alias for intel python, run the next command with the successful python from the last command:

```
 echo "alias python86='arch -x86_64 /usr/local/bin/python3.10-intel64'" > ~/.zshrc
```


#### Install Poetry

```
arch -x86_64 curl -sSL https://install.python-poetry.org | ./python3-intel64 -
```

You may get an error when you try to install poetry:

```
urllib.error.URLError: <urlopen error [SSL: CERTIFICATE_VERIFY_FAILED] certificate verify failed: unable to get local issuer certificate (_ssl.c:997)>
```

If this happens you need go to spotlight and run ```Install Certificates.Command```.  Then you will be able to install poetry.

After poetry installation is complete, change into a directory where you want python programs

```
      git clone https://github.com/Choate-Robotics/7407-DriveCode-Template.git
      cd 7407-DriveCode-Template
      poetry update
 ```
   
 To open virtualenv shell (Run this every time you open a new terminal in that folder)
 ```
      python86 -m poetry shell
 ```

### Non-Mac Directions

#### Clone the repository code onto your computer:

```

git clone https://github.com/Choate-Robotics/7407-DriveCode-Template.git

```
If you prefer ssh:

```

git clone git@github.com:Choate-Robotics/7407-DriveCode-Template.git

```

### If you don't have Poetry installed already:

#### Linux and Mac

```

curl -k https://install.python-poetry.org/ | python3 - 

```
You might have to replace "python" at the end with "python3" depending on how python is configured in your system.

#### Windows Powershell

```

(Invoke-WebRequest -Uri https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py -UseBasicParsing).Content | python -

```

Further information can be found here: https://python-poetry.org/docs/



Make sure to add Poetry to your path.



### With Poetry Installed:

```

poetry shell

poetry install

```

### Deploying Code:
Connect to the robot's wifi.
``python robot.py deploy``
If absolutely necessary, use ``python robot.py deploy --no-version-check`` to avoid WPILib version issues on the robot.


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
Always use ``poetry add {library}`` to add libraries. This ensures that libraries are compatible and allows everyone to work easier.
Never, ever, edit poetry.lock or pyproject.toml manually.

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
To integrate a branch with branch **Main**,  create a pull-request with the same title as your branch. Make sure pre-commits pass before pushing to ensure clean code.

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
