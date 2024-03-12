// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.lib.config.CTREConfigs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  Compressor c = new Compressor(PneumaticsModuleType.REVPH);
TalonSRX kol=new TalonSRX(3);
WPI_VictorSPX alis = new WPI_VictorSPX(26);
WPI_VictorSPX sol_atis = new WPI_VictorSPX(57);
  WPI_VictorSPX sag_atis = new WPI_VictorSPX(13);
  private final Joystick kol2 = new Joystick(1);
  private final Joystick kol3 = new Joystick(0);
  Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
  Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (kol3.getRawButton(4)){
    sol_atis.set(ControlMode.PercentOutput,1); 
    sag_atis.set(ControlMode.PercentOutput,1); 
    }
   
    else  {
      sag_atis.set(ControlMode.PercentOutput,0);
      sol_atis.set(ControlMode.PercentOutput,0);
    }
    
    if (kol2.getRawButton(2)){
     alis.set(ControlMode.PercentOutput,-0.5); 
      
      }
      else if (kol2.getRawButton(3)){
        alis.set(ControlMode.PercentOutput,1); 
         
         }
     
      else  {
        alis.set(ControlMode.PercentOutput,0);
        
      }
      
      if (kol2.getRawButton(4)){
        kol.set(ControlMode.PercentOutput,-1); 
         
         }
        else if (kol2.getRawButton(1)){
          kol.set(ControlMode.PercentOutput,1); 
           
           }
        
         else  {
           kol.set(ControlMode.PercentOutput,0);
           
         }

 if(kol2.getRawButton(6)) solenoid2.set(true);
    else solenoid2.set(false);
    if(kol2.getRawButton(5)) solenoid1.set(true);
    else solenoid1.set(false);  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}




