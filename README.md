# 2021-robot
🤖 Code for our 2021 competition robot.

## Controls
We use three total joysticks to control the robot:

* 2 x **Logitech Attack 3** (`joystick_left` and `joystick_right`)
* 1 x **Logitech Extreme 3D Pro** (`joystick_alt`)

<img src="res/ATK3.png" height="600"><img src="res/X3D.png" height="600">

## Run Code With Gradle
Gradle allows us to build and deploy our Java code to the robot
1. Right-click on the `build.gradle` file in Visual Studio Code and press `Build Robot Code`
1. Select the *Java Build* from the dropdown at the top to build the code
1. Fix any problems that come up and repeat steps 1 and 2 until the terminal says **BUILD SUCCESSFUL**
    1. A common problem that may come up has to do with formatting, which is shown by the error `Execution failed for task ':spotlessJavaCheck'`. You can fix this problem by running `./gradlew :spotlessApply` in a terminal window
1. Connect to the robot’s wifi
1. Right-click on the `build.gradle` file in Visual Studio Code and press `Deploy Robot Code`
1. Select the *Java Build* from the dropdown at the top to deploy the code