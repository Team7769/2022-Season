// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ISubsystem;
import frc.robot.Subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController _driverController;
  private XboxController _operatorController;
  private static Drivetrain _drivetrain;
  private static Collector _collector;
  private static Climber _climber;
  private static Shooter _shooter;
  private ArrayList<ISubsystem> _subsystems;

  private int _autonomousCase = 0;
  private int _autonomousLoops = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);
    _drivetrain = Drivetrain.GetInstance();
    _collector = Collector.GetInstance();
    _climber = Climber.GetInstance();
    _shooter = Shooter.GetInstance();

    _subsystems = new ArrayList<ISubsystem>();

    _subsystems.add(_drivetrain);
    _subsystems.add(_collector);
    _subsystems.add(_climber);
    _subsystems.add(_shooter);
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
    // This is the robot periodic method.

    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    _drivetrain.updatePose();
    _shooter.zeroHood();

    _subsystems.forEach(s -> {
      s.LogTelemetry();
      s.ReadDashboardData();
    });
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kCustomAuto);
    m_autoSelected = kDefaultAuto;
    System.out.println("Auto selected: " + m_autoSelected);
    
    _autonomousCase = 0;
    _autonomousLoops = 0;
    _drivetrain.resetGyro();
    resetOdometry();
  }
  
  public void resetOdometry()
  {
    //_drivetrain.resetOdometry();
    _drivetrain.resetEncoders();
    //_drivetrain.updatePose();
    //_drivetrain.resetPIDControllers();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        testAuto();
        break;
      case kDefaultAuto:
      default:
        //turnTest();
        driveBackAndShootAuto();
        //fiveBallAuto();
        // Put default auto code here
        break;
    }
    _autonomousLoops++;
  }

  public void testAuto()
  {
    switch(_autonomousCase)
    {
      case 0:
        _drivetrain.setTestPath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();
        
        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }

  public void turnTest()
  {
    switch (_autonomousCase)
    {
      case 0:
        _drivetrain.resetPID();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.turnToAngle(90);

        if (_drivetrain.isTurnFinished())
        {
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }
  
  public void driveBackAndShootAuto()
  {
    switch(_autonomousCase)
    {
      case 0:
        _drivetrain.setDriveForwardAndShootPath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();
        
        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setCollectTwoFromTerminalPath();
        _autonomousCase++;
        //_autonomousCase = 9000;
        break;
      case 3:
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops >= 100)
        {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setDriveBackFromTerminalPath();
        _autonomousCase++;
        break;
      case 6:
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops >= 150)
        {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }

  public void fiveBallAuto()
  {
    switch (_autonomousCase)
    {
      case 0:
        _drivetrain.setFiveBallPartOnePath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setFiveBallPartTwoToTerminalPath();
        _autonomousCase++;
        break;
      case 3:
        _drivetrain.tankDriveVolts(0, 0);

        if (_autonomousLoops >= 100)
        {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setFiveBallPartTwoFromTerminalPath();
        _autonomousCase++;
        break;
      case 6:
        _drivetrain.tankDriveVolts(0, 0);

        if (_autonomousLoops >= 50)
        {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    teleopDrive();
    teleopShoot();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void teleopDrive() {

    var throttle = -_driverController.getLeftY();
    var turn = _driverController.getRightX();
    
    _drivetrain.drive(throttle, turn);
  }

  private void teleopShoot() {
    if (_operatorController.getAButtonPressed())
    {
      _shooter.setQuarterShot();
    } else if (_operatorController.getXButtonPressed())
    {
      _shooter.setHalfShot();
    } else if (_operatorController.getYButtonPressed())
    {
      _shooter.setThreeQuarterShot();
    }

    if (_driverController.getLeftTriggerAxis() >= 0.5)
    {
      _shooter.readyShot();
    } else {
      _shooter.stop();
    }


  }
}
