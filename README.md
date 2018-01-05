# 4256-17-Rockefeller  
the Cyborg Cats' Java robot code for FRC Steamworks 2017  
-> winner of the Innovation in Control Award Sponsored by Rockwell Automation at the Rocket City Regional!  
  
The classes under com.cyborgcats.reusable were designed to work year after year with little to no modification, so they would likely work for other FRC teams using similar hardware. Please just cite our work in a comment somewhere if you decide to use it. Thanks!  
  
primary code outline:  
_reusable_  
**R_CANTalon:** extends CTRE CANTalon to understand gear ratios and convert encoder counts to angles; expects CTRE Magnetic Encoders  
**R_Gyro:** extends Kauai Labs AHRS in order to make finding the difference between current and target angles easier  
**R_Xbox:** extends XboxController and lays out constants to make working with Xbox One controllers more efficient  
**V_Compass:** main logic for dealing with angles  
**V_Fridge:** functions that simulate complicated controls like toggles  
**V_PID:** obviously manages our PID loops  
_this year_  
**R_DriveTrain:** integrates 4 swerve modules using math from Chief Delphi and provides alignment capabilities  
**R_SwerveModule:** integrates 3 CANTalons and contains alignment and field oriented code  
  
filename prefix conventions:  
**R:** a class that codes something tangible, like a motor or Xbox controller  
**V:** a class that embodies real abilities, but doesn't interact with hardware  
In addition, please note that the "abstract" label on our classes just means that everything is static.  

student team:  
Hayden Shively  
Ian Woodard  
Jack Bauer  
Teagan LeVar  
  
QUESTIONS AND COMMENTS ARE WELCOME!  
  
Special thanks to Mr. Ice, Mr. Albertson, and Mr. Fultz!
