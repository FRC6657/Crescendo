# FRC Team 6657's 2024 code for CRESCENDO
This project uses the WPILib command-based structure, with an [Advantage Kit](https://github.com/Mechanical-Advantage/AdvantageKit) IO structure.

### **Subsystems**
  - [Swerve Drivetrain](src/main/java/frc/robot/subsystems/drive) MAXSwerve drivebase
  - [Intake](src/main/java/frc/robot/subsystems/intake) Our ground note intake.
  - [Outtake](src/main/java/frc/robot/subsystems/outtake) Our note shooter for scoring in the Amp and Speaker.
  - [Climb](src/main/java/frc/robot/subsystems/climb) Dual single stage elevators for a simple robust climb.
  - [LEDs](src/main/java/frc/robot/subsystems/intake) Used to send information to the drivers or human player.
  - [Superstructure](src/main/java/frc/robot/subsystems/Superstructure.java) Coordinates movement between subsystems. 

### **Credits**
  - [Advantage Kit](https://github.com/Mechanical-Advantage/AdvantageKit): The library we use to log and structure our codebase.
  - [REV MAXSwerve Template](https://github.com/REVrobotics/MAXSwerve-Java-Template): The rough reference for our swerve code.
  - [PhotonVision](https://github.com/PhotonVision/photonvision): Vision Software running on our OrangePi5 for AprilTag detection.
  - [Choreo](https://github.com/SleipnirGroup/Choreo): Pathplanning software used for autonomous pathing.
  - [8033 Highlander Robotics](https://github.com/HighlanderRobotics/): Wonderful training resources, and has provided code help to us on numerous occations.
