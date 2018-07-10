# 4256-18-Faraday
the Cyborg Cats' Java robot code for FRC Power Up 2018  
  
The classes under com.cyborgcats.reusable were designed to work year after year with little to no modification, so they would likely work for other FRC teams using similar hardware. Please just cite our work in a comment somewhere if you decide to use it. Thanks!  
  
primary code outline:  
_reusable_  
**Compass:** main logic for dealing with angles  
**Drivetrain:** interface for creating autonomous-compatible drivetrains  
**Fridge:** functions that simulate complicated controls like toggles  
**Gimbal:** class for controlling camera gimbals  
**Gyro:** extends Kauai Labs AHRS to simplify calls to the gyro  
**PID:** manages PID loops  
**Subsystem:** interface for creating autonomous-compatible mechanisms  
**Xbox:** extends XboxController and lays out constants to make working with Xbox One controllers more efficient  
  
_reusable.Autonomous_  
**Events:** logic for controlling Subsystems autonomously  
**Leash:** logic for autonomously following Paths with a Drivetrain  
**Odometer:** abstract class for getting data from a field positioning system  
**Path:** interface for creating Leash-compatible lines and curves  
**P_Bezier:** an implementation of Path allowing for the use of cubic splines  
**P_Curve:** an implementation of Path allowing for the use of mathematical expressions  
**Strategy:** abstract class for integrating and following a set of Events and a Leash  
  
_reusable.Phoenix_
**Convert:** a complicated looking class that provides simple syntax for converting raw encoder counts to more useful units  
**Encoder:** an enum that stores important facts about various encoder models  
**Talon:** extends CTRE TalonSRX to simplify calls to motors, especially those with encoders  
**Victor:** essentially the same as Talon  
  
  
_this year_  
**D_Swerve:** an implementation of Drivetrain for 4-module swerve  
**SwerveModule:** integrates 2 Talons, basically a backend for D_Swerve  
  
_this year.Autonomous_  
**Coach:** a class that helps decide which Strategy to use based on input from the SmartDashboard  
**O_Encoder:** an implementation of Odometer for an encoder on a swerve wheel  
**O_ZED:** an implementation of Odometer for visual odometery from the ZED  
**Strategy2018:** extends Strategy with useful constants and game-specific functions  
**S_DriveForward:** extends Strategy2018 (drives straight forward, drops cube if appropriate)  
**S_DropInNearest:** extends Strategy2018 (drops a cube in the nearest appropriate location)  
**S_PassLine:** extends Strategy2018 (just passes the auto line)  
**S_Slither:** extends Strategy2018 (follows a sin wave)  
  

student team:  
Hayden Shively  
Ian Woodard  
Michael Pritchett  
  
QUESTIONS AND COMMENTS ARE WELCOME!  
  
Special thanks to Mr. Ice, Mr. Shultz, and Mr. Fultz!
