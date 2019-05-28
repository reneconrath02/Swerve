/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GyroSwerveDriveCommand;

/**
 * Add your docs here.
 */
public class GyroSwerveDrive extends Subsystem {
  public double[] speed = new double[4];
  public double[] angle = new double[4];
  public boolean fcd = true;

  public void gyroDrive(double str, double fwd, double rot){
    computeSwerveInputs(str, fwd, rot);
    setSetpoints(rot);

    for(int i = 0; i < 4; i++){
      RobotMap.swerveMod[i].moveModule(speed[i], angle[i]);
    }

    // System.out.println("Here22");
  }

  public void computeSwerveInputs (double str, double fwd, double rot){ //what does this do!!!!
    double gyroAngle = -1 * Math.toRadians(RobotMap.gyro.getAngle() % 360); //makes gyro angle in radians mulitplied by negative one because gyro direction is clockwise, and we wanted it counterclockwise

    if(fcd){
      double intermediary = fwd * Math.cos(gyroAngle) + str * Math.sin(gyroAngle);
      str = -fwd * Math.sin(gyroAngle) + str * Math.cos(gyroAngle); //x and y vlaues into polar coordinates, also in Ether's
      fwd = intermediary;
    }

    double a = str - rot * (RobotMap.SWERVE_LENGTH / RobotMap.SWERVE_RADIUS); //Vector components(Ether's number 1)
    double b = str + rot * (RobotMap.SWERVE_LENGTH / RobotMap.SWERVE_RADIUS);
    // double a = 0;
    // double b = 0;
    // double c = .2;
    // double d = .2;
    double c = fwd - rot * (RobotMap.SWERVE_WIDTH / RobotMap.SWERVE_RADIUS);
    double d = fwd + rot * (RobotMap.SWERVE_WIDTH / RobotMap.SWERVE_RADIUS);
    
    speed[1] = Math.sqrt ((a * a) + (d * d)); //Ether's stuff
    speed[2] = Math.sqrt ((a * a) + (c * c));
    speed[0] = Math.sqrt ((b * b) + (d * d));
    speed[3] = Math.sqrt ((b * b) + (c * c));

    angle[1] = Math.atan2 (a, d) / Math.PI;
    angle[2] = Math.atan2 (a, c) / Math.PI;
    angle[0] = Math.atan2 (b, d) / Math.PI;
    angle[3] = Math.atan2 (b, c) / Math.PI;
  }

  public void driveStarightOnly(double fwd){ //what does this do!!!!

    // double a = str - rot * (RobotMap.SWERVE_LENGTH / RobotMap.SWERVE_RADIUS); //Vector components(Ether's number 1)
    // double b = str + rot * (RobotMap.SWERVE_LENGTH / RobotMap.SWERVE_RADIUS);
    double a = 0;
    double b = 0;
    double c = fwd;
    double d = fwd;
    // double c = fwd - rot * (RobotMap.SWERVE_WIDTH / RobotMap.SWERVE_RADIUS);
    // double d = fwd + rot * (RobotMap.SWERVE_WIDTH / RobotMap.SWERVE_RADIUS);
    
    speed[1] = Math.sqrt ((a * a) + (d * d)); //Ether's stuff
    speed[2] = Math.sqrt ((a * a) + (c * c));
    speed[0] = Math.sqrt ((b * b) + (d * d));
    speed[3] = Math.sqrt ((b * b) + (c * c));

    angle[1] = Math.atan2 (a, d) / Math.PI;
    angle[2] = Math.atan2 (a, c) / Math.PI;
    angle[0] = Math.atan2 (b, d) / Math.PI;
    angle[3] = Math.atan2 (b, c) / Math.PI;

    for(int i = 0; i < 4; i++){
      RobotMap.swerveMod[i].moveModule(speed[i], angle[i]);

    }
  }

  public void setSetpoints(double rot){ //turning angles into encoder setpoints for PID
    for(int i = 0; i < 4; i++){
      // SmartDashboard.putNumber("angle: " + i, angle[i]);
      // SmartDashboard.putNumber("speed: " + i, speed[i]);

      double encCount = RobotMap.swerveMod[i].encoder.pidGet();
      angle[i] = (angle[i] + 1) * RobotMap.SWERVE_ENC_CIRC / 2 + RobotMap.SWERVE_SETPOINT_OFFSET[i]; 
      if(angle[i] > RobotMap.SWERVE_ENC_CIRC) angle[i] -= RobotMap.SWERVE_ENC_CIRC;

      double degreesBeforeFlip = 90.0;
      // if(Math.abs(encCount - angle[i]) > RobotMap.SWERVE_ENC_CIRC / 360 * degreesBeforeFlip) { //go the short way
      //   angle[i] = getOppositeAngle(i);
      //   speed[i] *= -1;
      // }
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new GyroSwerveDriveCommand());
  }
}
