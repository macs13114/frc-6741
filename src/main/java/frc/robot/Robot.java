// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//////
import edu.wpi.first.wpilibj.Encoder;
// Joysticks Import
import edu.wpi.first.wpilibj.Joystick;

// Basic Needed WPILIB Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Motor Controlers Import
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// Data On Screen Output / Dashboards Import
import edu.wpi.first.cameraserver.CameraServer;

// NavX / Gyros
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  // ID Check And MC Set
  VictorSPX RightF = new VictorSPX(1);
  VictorSPX RightB = new VictorSPX(2);
  VictorSPX LeftB = new VictorSPX(3);
  VictorSPX LeftF = new VictorSPX(4);
  VictorSPX angleMotor1 = new VictorSPX(5);
  VictorSPX angleMotor2 = new VictorSPX(6);
  VictorSPX zvatMotor1 = new VictorSPX(7);
  VictorSPX zvatMotor2 = new VictorSPX(8);
  
  Encoder encoder = new Encoder(2, 1, false, EncodingType.k2X);
  Encoder encoder2 = new Encoder(5, 4, true, EncodingType.k2X);
  Encoder encoder3 = new Encoder(8, 7, false, EncodingType.k2X);

  AHRS NavX = new AHRS(Port.kMXP);

  Joystick driverJS = new Joystick(0);
  Joystick pilotJS = new Joystick(2);
  Joystick operatorJS = new Joystick(1);

  double angle1 = Math.round(NavX.getAngle());

  private static final String kWallAuto = "Wall Ride";
  private static final String kRegularAuto = "Default";
  private static final String kRampAuto = "Ramp Mode";

  private String m_autoSelect;
  private final SendableChooser<String> m_Chooser = new SendableChooser<>();
  
  public void move_forword(Double speed)
  {
    RightF.set(ControlMode.PercentOutput, speed);
    LeftF.set(ControlMode.PercentOutput, speed);
  }

  public void Arcade_mode()
  { 
    double throttle = driverJS.getRawAxis(1) / 1.5;
    double turn = driverJS.getRawAxis(4);
    
    // Throtle math
    if ( Math.abs(throttle) < 0.1 )
    {
      throttle = 0;
    }
    // Turn Math
    if ( Math.abs(turn) < 0.2 )
    {
      turn = 0;
    }
    // non-zero Math
    if ( Math.abs(throttle) > 0.05 )
    {
      turn *= Math.abs(throttle);
    }

    double l = -throttle + turn;
    double r = -throttle - turn;

    LeftF.set(ControlMode.PercentOutput, l);
    RightF.set(ControlMode.PercentOutput, r);  
  }

  public void Tank_mode()
  {
    double throttle1 = driverJS.getRawAxis(1) / 1.5;
    double throttle2 = driverJS.getRawAxis(5) / 1.5;
    
    LeftF.set(ControlMode.PercentOutput, -throttle1);
    RightF.set(ControlMode.PercentOutput, -throttle2);
  }

  public void Zvat_Control()
  {
    Double setpoint = 0.0;
    Double kP = 0.05;
    double m_dist = encoder3.getDistance() + 70;
    
    boolean b1 = operatorJS.getRawButton(1);
    boolean b2 = operatorJS.getRawButton(2);
    boolean b3 = operatorJS.getRawButton(3);
    boolean b4 = operatorJS.getRawButton(4);

    if(b1)
    {
      setpoint = 70.0;
    }
    if(b2)
    {
      setpoint = 5.0;
    }
    
    // // // // // // // // // // // // // // // // // // // //
    double error = setpoint - m_dist;
    Double outP = error * kP;
    if(outP > 1)
    {
      outP = 1.0;
    }
    else if(outP < -1)
    {
      outP = -1.0;
    }
    angleMotor1.set(ControlMode.PercentOutput, outP);
    // // // // // // // // // // // // // // // // // // // //

    if(b3)
    {
      zvatMotor1.set(ControlMode.PercentOutput, 0.7);
    }
    else
    {
      zvatMotor1.set(ControlMode.PercentOutput, 0.0);
    }
    if(b4)
    {
      zvatMotor1.set(ControlMode.PercentOutput, -0.7);
    }
    else
    {
      zvatMotor1.set(ControlMode.PercentOutput, 0.0);
    }
  }
  
  @Override
  public void robotInit() 
  {
    LeftB.follow(LeftF);
    RightB.follow(RightF);
    zvatMotor2.follow(zvatMotor1);
    angleMotor2.follow(angleMotor1);

    LeftB.setInverted(false);
    LeftF.setInverted(false);
    RightB.setInverted(true);
    RightF.setInverted(true);
    zvatMotor1.setInverted(false);
    zvatMotor2.setInverted(true);
    angleMotor1.setInverted(false);
    angleMotor2.setInverted(true);

    encoder.reset();
    encoder2.reset();
    //encoder3.reset();
    NavX.reset();

    //encoder.setSamplesToAverage(10);
    //encoder.setMinRate(0.1);
    encoder.setDistancePerPulse(0.1/256.0);
    //encoder2.setSamplesToAverage(10);
    //encoder2.setMinRate(0.1);
    encoder2.setDistancePerPulse(0.1/256.0);
    //encoder3.setSamplesToAverage(10);
    //encoder3.setMinRate(0.1);
    //encoder3.setDistancePerPulse(0.1/256.0);

    m_Chooser.setDefaultOption("Regular Auto", kRegularAuto);
    m_Chooser.addOption("Wall Auto", kWallAuto);
    m_Chooser.addOption("Ramp Auto", kRampAuto);
    SmartDashboard.putData("Choise", m_Chooser);
  }

  @Override
  public void robotPeriodic()
   {
    // Encoder Data
    //double m_dist = encoder3.getDistance() + 70;

    // NavX Controls
    double roller = NavX.getRoll();

    // SmartDash Output
    SmartDashboard.putNumber("Robot Roll", roller);
    SmartDashboard.putNumber("Robot Direction", angle1);
    SmartDashboard.putNumber("Left Encoder", encoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", encoder2.getDistance());
    //SmartDashboard.putNumber("Angle Encoder VALUE", m_dist);

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit()
   {
    encoder.reset();
    encoder2.reset();
    NavX.reset();
    m_autoSelect = m_Chooser.getSelected();
    SmartDashboard.putString("Selected Auto", m_autoSelect);
    }

  /** This function is called periodically during autonomous. */
  @Override
  
  public void autonomousPeriodic() 
  {
    double time = Timer.getFPGATimestamp();
    Double speed = 1.0;
    Double speed2 = 0.5;
    Double zvat_s = 0.7;
    double roll = NavX.getRoll();
    Double kp = 0.1;
    Double sRoll = roll - 7;
    Double nRoll = roll + 7;
    Double power3 = nRoll * kp;
    Double power2 = sRoll * kp;

    switch(m_autoSelect)
    {
      case kRegularAuto:
      {
        while(time <= 2)
        {
          zvatMotor1.set(ControlMode.PercentOutput, zvat_s);
        }
        while(time <= 5)
        {
          LeftF.set(ControlMode.PercentOutput, speed);
          RightF.set(ControlMode.PercentOutput, speed);  
        }
        while(time <= 6)
        {
          if(angle1 < 90)
          {
            LeftF.set(ControlMode.PercentOutput, speed2);
            RightF.set(ControlMode.PercentOutput, -speed2);  
          }
        }
        while(time <= 8)
        {
          LeftF.set(ControlMode.PercentOutput, speed);
          RightF.set(ControlMode.PercentOutput, speed);  
        }
        while(time <= 9)
        {
          if(angle1 < 180)
          {
            LeftF.set(ControlMode.PercentOutput, speed2);
            RightF.set(ControlMode.PercentOutput, -speed2);  
          }
        }
        while(time <= 11)
        {
          LeftF.set(ControlMode.PercentOutput, speed);
          RightF.set(ControlMode.PercentOutput, speed);  
        }
        while(time <= 15)
        {
          if(roll >= 7)
          {
            LeftF.set(ControlMode.PercentOutput, power2);
            RightF.set(ControlMode.PercentOutput, power2);  
          }
          else if(roll <= -7)
          {
            LeftF.set(ControlMode.PercentOutput, power3);
            RightF.set(ControlMode.PercentOutput, power3);  
          }
          else
          {
            LeftF.set(ControlMode.PercentOutput, 0);
            RightF.set(ControlMode.PercentOutput, 0);  
          }
        }
      }

      case kWallAuto:
      {
        while(time <= 2)
        {
          zvatMotor1.set(ControlMode.PercentOutput, zvat_s);
        }
        while(time <= 5)
        {
          LeftF.set(ControlMode.PercentOutput, speed);
          RightF.set(ControlMode.PercentOutput, speed);  
        }
        while(time <= 5.2)
        {
            LeftF.set(ControlMode.PercentOutput, -speed2 / 2);
            RightF.set(ControlMode.PercentOutput, speed2 / 2);  
        }
        while(time <= 6)
        {
          if(angle1 > 270)
          {
            LeftF.set(ControlMode.PercentOutput, -speed2);
            RightF.set(ControlMode.PercentOutput, speed2);  
          }
        }
        while(time <= 8)
        {
          LeftF.set(ControlMode.PercentOutput, speed);
          RightF.set(ControlMode.PercentOutput, speed);  
        }
        while(time <= 9)
        {
          if(angle1 > 180)
          {
            LeftF.set(ControlMode.PercentOutput, -speed2);
            RightF.set(ControlMode.PercentOutput, speed2);  
          }
        }
        while(time <= 11)
        {
          LeftF.set(ControlMode.PercentOutput, speed);
          RightF.set(ControlMode.PercentOutput, speed);  
        }
        while(time <= 15)
        {
          if(roll >= 7)
          {
            LeftF.set(ControlMode.PercentOutput, power2);
            RightF.set(ControlMode.PercentOutput, power2);  
          }
          else if(roll <= -7)
          {
            LeftF.set(ControlMode.PercentOutput, power3);
            RightF.set(ControlMode.PercentOutput, power3);  
          }
          else
          {
            LeftF.set(ControlMode.PercentOutput, 0);
            //LeftB.set(ControlMode.PercentOutput, 0); 
            RightF.set(ControlMode.PercentOutput, 0);  
            //RightB.set(ControlMode.PercentOutput, 0);
          }
        }
      }

      case kRampAuto:
      {
        while(time <= 2)
        {
          zvatMotor1.set(ControlMode.PercentOutput, zvat_s);
        }
        while(time <= 5)
        {
          LeftF.set(ControlMode.PercentOutput, speed2);
          RightF.set(ControlMode.PercentOutput, speed2);  
        }
        
        if(roll >= 7)
        {
          LeftF.set(ControlMode.PercentOutput, power2);
          RightF.set(ControlMode.PercentOutput, power2);  
        }
        else if(roll <= -7)
        {
          LeftF.set(ControlMode.PercentOutput, power3);
          RightF.set(ControlMode.PercentOutput, power3);  
        }
        else
        {
          LeftF.set(ControlMode.PercentOutput, 0);
          RightF.set(ControlMode.PercentOutput, 0);  
        }
      }
    }  
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CameraServer.startAutomaticCapture();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    // Zvat And Angle Control Method
    Zvat_Control();
    
    // Arcade Mode Method
    Arcade_mode();

    // Tank Mode Method
    //Tank_mode();
    }
 
  @Override
  public void testInit() 
  {
    encoder.reset();
    encoder2.reset();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() 
  {/** 
    double n = 0.3;
      
    LeftF.set(ControlMode.PercentOutput, n);
    RightF.set(ControlMode.PercentOutput, n);  
    zvatMotor1.set(ControlMode.PercentOutput, n);*/
  }

  @Override
  public void simulationPeriodic() {}
}