
Pull requests are required for all changes.
Create a pull request, and ask the team to review your code.

While you are writing or changing a subsytem, test your changes in the corresponding Container for your subsystem.
Edit the `container` field in Robot.java to be your robot.   

Available setups are
  GAME // the robot used for the game, and testing all the components together
  INTAKE // robot with just intake
  DRIVE // robot with just the drivetrain setup
  ARM // robot with the delivery arm
  CLIMB // robot with the climber
  VISION // robot with the limelight vision system


When creating new profiles for different robots, use index.html in the docs directory

Add the file in the deploy/swerve directory

To use different drivetrain profile change the return string inside getDriveTrainName method.