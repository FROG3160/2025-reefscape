# 2025-reefscape
Code for FIRST Robotics Competition 2025 game - Reefscape

Install/Setup

If you haven't already, set up your laptop with the instructions from [FROG Windows System Setup](https://frog3160.github.io/setup).

**Note:**  Make sure you have Python 3.12 installed. An installer can be downloaded from [here](https://www.python.org/downloads/release/python-3128/).

When you follow the instructions to clone the repository, VS Code should ask you about installing the recommended extensions.  Confirm that you want them installed, they contain some customizations we'll use.

Once the extensions are installed you should see 4 links in the status bar at the bottom:  "Create .venv", "Install Robotpy", "Sync/Update", and "Deploy to Robot"

You should only need to click on the "Create .venv" and then the "Install Robotpy" buttons once.  This will create the python virtual environment for your code and all the packages to install into, and then install robotpy into that environment, respectively.

Once you've clicked on the first two, click on "Sync/Update".  This will use the pyproject.toml file to install any other packages needed, including the files that will be deployed to the robot.

Once code has been tested in simulation and you are ready to deploy it to the robot, connect to the robot, either through USB, or over network, and click on the "Deploy to Robot" button.

"Sync/Update" and "Deploy to Robot" will be used over and over again as we upgrade libraries and packages and then send the code to the robot for further testing.
