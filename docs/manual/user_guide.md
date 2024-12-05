User Guide
=======================

@page user_guide User Guide


Getting started
-----------------------

### Prerequisites
This project depends on Git, STM32CubeIDE and Docker.

For Git see: https://git-scm.com/downloads

For STM32CubeIDE see: https://docs.nav2.org/getting_started/index.html#installation

On Linux, Docker can be installed with the terminal command:
```
sudo apt install docker
```

In order for microRos dependencies to be downloaded when building, Docker must be
configured to run with root privilige. This can be done with:
```
sudo groupadd docker
sudo usermod -aG docker $USER
```

Log in and out after completing this step in order for group membership to be evaluated.

### Installation
1. Clone the repository:
```
git clone https://github.com/DVA490-474-Project-Course/stm32-code.git
```
2. Start STM32CubeIDE

3. Log in to your ST account in the IDE if you aren't already logged on.

4. Click 'File>Open Projects from file System...'

5. Click the 'Directory...' button and select the repository folder.

6. Make sure the project is selected in the checklist and click 'Finish'.

7. Generate code by clicking 'Project>Generate Code'.

Usage
-----------------------

In order to upload the program, connect the PC to the USB port CN1  (the one located on
the side of the board which contains the smaller processor, which is the debugger) and
click Run>Run.

Building for the first time may take a few minutes since microROS is downloading its
dependencies to the project.
