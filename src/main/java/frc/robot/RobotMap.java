/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.SwerveModule;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static double SWERVE_ENC_CIRC = 4.927;
  public static double kP = 0.7;
  public static final double[] SWERVE_SETPOINT_OFFSET = {0.7, 4.72, 3.28, 3.456};
	public static double[][] SWERVE_PID_CONSTANTS = {{kP, 0.0, 0.0}, {kP, 0.0, 0.0}, {kP, 0.0, 0.0}, {kP, 0.0, 0.0}};
  public static boolean[] ANGLE_MOTOR_INVERTED = {true, false, false, false};

  //swerve
  public static SwerveModule[] swerveMod;
	public static double MAX_VEL = 10;
  public static double MAX_ACC = 5;
	public static double WHEELBASE_WIDTH = 0.8;
	public static double WHEELBASE_LENGTH = 0.8;
	public static double SWERVE_WHEEL_DIAMETER = 0.05; // in m?
	public static int SWERVE_MAX_CURRENT = 30; // in amps
	public static int SWERVE_CURRENT_DUR = 100; // in ms
	public static double SWERVE_LENGTH = 21.5;
	public static double SWERVE_WIDTH = 21.5;
	public static double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_LENGTH, 2) + Math.pow(SWERVE_WIDTH, 2));
	public static double SWERVE_LOOP_TIME = 0.100; // in ms (50 ms default)
  public static double SWERVE_PID_TOLERANCE = SWERVE_ENC_CIRC / 100.0 / 4.0; // .25%
  
  public static AHRS gyro;
  
  public static void init(){

    swerveMod = new SwerveModule[4];

    for(int i = 0; i < 4; i++){
      swerveMod[i] = new SwerveModule(i, ANGLE_MOTOR_INVERTED[i]);
    }

    gyro = new AHRS(SPI.Port.kMXP);

  }
}
