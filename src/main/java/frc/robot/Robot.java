// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;

// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//import com.revrobotics.CANSparkBase;
//rev imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
//import com.revrobotics.CANSparkBase.ExternalFollower;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kRed_Amp = "Red Amp";
  private static final String kRed_Source = "Red Source";
  private static final String kBlue_Amp = "Blue Amp";
  private static final String kBlue_Source = "Blue Source";
  private static final String kCenter = "Center";
  private static final String kShoot_Only = "Shoot Only";
  private static final String kTESTING = "TESTING ONLY";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // for arc turn
  private static final String kRight = "Turn Right";
  private static final String kLeft = "Turn Left";

  // motor declarations
  public final CANSparkMax m_leftMotorLEAD = new CANSparkMax(1, MotorType.kBrushless);
  public final CANSparkMax m_leftMotorFOLLOW = new CANSparkMax(2, MotorType.kBrushless);
  public final CANSparkMax m_rightMotorLEAD = new CANSparkMax(3, MotorType.kBrushless);
  public final CANSparkMax m_RightMotorFOLLOW = new CANSparkMax(4, MotorType.kBrushless);
  public final CANSparkMax m_Feeder = new CANSparkMax(5, MotorType.kBrushless);
  public final CANSparkMax m_Shooter = new CANSparkMax(6, MotorType.kBrushless);
  public final CANSparkMax m_AngleLEAD = new CANSparkMax(7, MotorType.kBrushless);
  public final CANSparkMax m_AngleFOLLOW = new CANSparkMax(8, MotorType.kBrushless);
  public static final SparkAbsoluteEncoder.Type kDutyCycle = Type.kDutyCycle;
  // Joysticks
  private final Joystick flight = new Joystick(0);
  // Flight Buttons
  private static final int flight1 = 1; // trigger
  private static final int flight2 = 2; // thumb
  private static final int flight3 = 3;
  private static final int flight4 = 4;
  private static final int flight5 = 5;
  private static final int flight6 = 6;
  private static final int flight7 = 7;
  private static final int flight8 = 8;
  private static final int flight9 = 9;
  // private static final int flight10 = 10;
  // private static final int flight11 = 11;
  private static final int flight12 = 12;
  // Flight Other
  private static final int flightPaddle = 3;

  // Drive Control
  private final DifferentialDrive Drive_Main = new DifferentialDrive(m_leftMotorLEAD::set, m_rightMotorLEAD::set);
  private double speedMultiplier = 0;
  double xCorrect = .65; // used to smooth out turning
  boolean forwardDriveToggle = true;

  // USB Camera
  UsbCamera USBCamera;
  UsbCamera USBCamera1;
  NetworkTableEntry cameraSelection;

  // Shooting Var
  double intakeSpeed = 0.2;
  double shootSpeed = -1;
  double raw_speed;
  double currentDif;
  double maxAngle = 0.1727; // Can't be less than
  double minAngle = 0.6425; // Can't be greater than
  // tilt values
  double shooterTiltSpeaker = 0.6488;
  double shooterTiltAmp = 0.1599;
  double shooterTiltClimb = 0.2185;
  boolean initAutoReturn = false;

  // auto angle calculator public values
  double interpolatedAngle;
  boolean shootSafe;

  // Reference points for close distance
  double limecloseDistanceRef = 1.3169254107224366;
  double limecloseAngleRef = 0.1610;

  // Reference points for far distance
  double limefarDistanceRef = 2.8232573610798135;
  double limefarAngleRef = 0.3467;

  // Different auto angle speeds
  double normalAngleSpeed = 0.375;
  double normalAngleSlope = 125;
  double hangAngleSpeed = 0.4;
  double hangAngleSlope = 90;

  // autonomous
  boolean hasShot;
  boolean hasDriveS1;
  boolean hasTurnS2;
  boolean hasDriveS3;
  boolean hasTurnS4;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Autonomous choices
    m_chooser.addOption("Red Amp", kRed_Amp);
    m_chooser.addOption("Red Source", kRed_Source);
    m_chooser.setDefaultOption("Center", kCenter);
    m_chooser.addOption("Blue Amp", kBlue_Amp);
    m_chooser.addOption("Blue Source", kBlue_Source);
    m_chooser.addOption("Shoot Only", kShoot_Only);
    m_chooser.addOption("TESTING ONLY", kTESTING);
    SmartDashboard.putData("Auto choices", m_chooser);
    // Drive motors init
    m_RightMotorFOLLOW.follow(m_rightMotorLEAD);
    m_leftMotorFOLLOW.follow(m_leftMotorLEAD);
    m_AngleFOLLOW.follow(m_AngleLEAD, true);
    m_leftMotorLEAD.setInverted(true);
    m_rightMotorLEAD.setInverted(false);
    Drive_Main.setSafetyEnabled(false);
    // Camera Server enable
    USBCamera = CameraServer.startAutomaticCapture(0);
    USBCamera1 = CameraServer.startAutomaticCapture(1);
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Encoder Details
    SmartDashboard.putNumber("Encoder Right", m_rightMotorLEAD.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Left", m_leftMotorLEAD.getEncoder().getPosition());
    SmartDashboard.putNumber("Current Difference", currentDif);
    SmartDashboard.putNumber("Angle", m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition());
    SmartDashboard.putNumber("Lime Distance", getLimeZ());
    SmartDashboard.putNumber("Angle Speed", raw_speed);

    // Drive Helpers
    SmartDashboard.putNumber("speedMultiplier", speedMultiplier);
    SmartDashboard.putBoolean("Forward Drive", forwardDriveToggle);
    SmartDashboard.putNumber("Y", flight.getY());
    SmartDashboard.putNumber("X", flight.getX());

    // Ultrasonic
    SmartDashboard.putBoolean("Shoot Safe", getShootSafe());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kAmp_side);
    System.out.println("Auto selected: " + m_autoSelected);
    m_rightMotorLEAD.getEncoder().setPosition(0);
    m_leftMotorLEAD.getEncoder().setPosition(0);
    m_leftMotorLEAD.setInverted(true);
    m_rightMotorLEAD.setInverted(false);
    hasShot = false;
    hasDriveS1 = false;
    hasTurnS2 = false;
    hasDriveS3 = false;
    hasTurnS4 = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kTESTING:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        } else if (!hasDriveS1) {
          driveStraightWithEncoder(.35, 2);
        } else if (!hasTurnS2) {
          ArcLengthTurnWithEncoder(.35, kRight, 2);
        } else if (!hasDriveS3) {
          driveStraightWithEncoder(.35, 2);
        } else if (!hasTurnS4) {
          ArcLengthTurnWithEncoder(.35, kLeft, 2);
        }
        break;
      case kRed_Source:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        } else if (!hasDriveS1) {
          driveStraightWithEncoder(.4, 10);
        } else if (!hasTurnS2) {
          ArcLengthTurnWithEncoder(.4, kRight, 2.1583);
        } else if (!hasDriveS3) {
          driveStraightWithEncoder(.4, 2.5);
        }
        break;
      case kRed_Amp:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        } else if (!hasDriveS1) {
          driveStraightWithEncoder(.4, 4.1667);
        } else if (!hasTurnS2) {
          ArcLengthTurnWithEncoder(.4, kLeft, 1.2917);
        } else if (!hasDriveS3) {
          driveStraightWithEncoder(.4, 5.8333);
        }
        break;
      case kCenter:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        } else if (!hasDriveS1) {
          driveStraightWithEncoder(.4, 5);
        }
        break;
      case kBlue_Source:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        } else if (!hasDriveS1) {
          driveStraightWithEncoder(.4, 10);
        }
        break;
      case kBlue_Amp:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        } else if (!hasDriveS1) {
          driveStraightWithEncoder(.4, 2.9167);
        } else if (!hasTurnS2) {
          ArcLengthTurnWithEncoder(.4, kRight, 2.15);
        } else if (!hasDriveS3) {
          driveStraightWithEncoder(.4, 4.1667);
        }
        break;
      case kShoot_Only:
        if (!hasShot) {
          m_Shooter.set(shootSpeed);
          wait(500);
          m_Feeder.set(shootSpeed);
          wait(200);
          m_Feeder.set(0);
          m_Shooter.set(0);
          hasShot = true;
        }
        break;
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_rightMotorLEAD.getEncoder().setPosition(0);
    m_leftMotorLEAD.getEncoder().setPosition(0);
    m_leftMotorLEAD.setInverted(true);
    m_rightMotorLEAD.setInverted(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // ********************************
    // * DRIVE CODE
    // ********************************
    // Drive with arcade drive. Y axis drives forward and backward, and the X turns
    // left and right.
    // Shooter side is the front of the robot.
    speedMultiplier = (((flight.getRawAxis(flightPaddle) * -1) * 0.2) + .6);
    Drive_Main.arcadeDrive(flight.getY() * speedMultiplier, flight.getX() * xCorrect);

    // Flip Logic
    if (flight.getRawButton(flight2)) {
      if (forwardDriveToggle == true) {
        forwardDriveToggle = false;
        m_leftMotorLEAD.setInverted(false);
        m_rightMotorLEAD.setInverted(true);
        xCorrect = Math.abs(xCorrect) * -1;
      } else {
        forwardDriveToggle = true;
        m_leftMotorLEAD.setInverted(true);
        m_rightMotorLEAD.setInverted(false);
        xCorrect = Math.abs(xCorrect);
      }
      // prevent doubleclick of the flip button
      try {
        Thread.sleep(300);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    // ********************************
    // * SHOOT CODE FOR FLIGHT STICK
    // ********************************
    // LIMELIGHT REF
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    if (flight.getRawButton(flight1) == true && (m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition() < 0.531)) { // Speaker
      initAutoReturn = true;
      // System.out.println(getLimeZ());
      // m_Shooter.set(shootSpeed);
      // wait(500);
      // m_Feeder.set(shootSpeed);
      m_Shooter.set(shootSpeed);
      AngleControl(calculateMotorAngle(), .435, 180);
      var calcs2 = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
      if ((currentDif > -0.007 && currentDif < 0.007)
          || (currentDif > -0.01 && currentDif < 0.01 && calcs2.getZ() < 1.84)) {
        wait(250);
        m_Feeder.set(shootSpeed);
      }
    } else if (flight.getRawButton(flight1) == true
        && (m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition() >= 0.531)) { // Amp
      m_AngleLEAD.set(0);
      m_Shooter.set(shootSpeed);
      wait(500);
      m_Feeder.set(shootSpeed);
    } else if (flight.getRawButton(flight4) == true) { // Intake
      m_Feeder.set(intakeSpeed);
      m_Shooter.set(intakeSpeed + 0.2);
    } else {
      m_Feeder.set(0);
      m_Shooter.set(0);
    }

    // ********************************
    // * ANGLE CODE FOR FLIGHT STICK
    // ********************************
    if (flight.getRawButton(flight7) == true || flight.getRawButton(flight5) == true) { // Angle speaker preset
      AngleControl(shooterTiltSpeaker, normalAngleSpeed, normalAngleSlope);
      initAutoReturn = false;
    } else if (flight.getRawButton(flight8) == true || flight.getRawButton(flight3) == true) { // Angle amp preset
      AngleControl(shooterTiltAmp, normalAngleSpeed - .125, normalAngleSlope);
      initAutoReturn = false;
    } else if ((flight.getRawButton(flight9) == true) && (flight.getRawButton(flight12) == true)) { // Auto hang preset
      AutoHang();
      initAutoReturn = false;
    } else if (flight.getRawButton(flight1) == true) {
      // This is intentionally left empty to allow "Auto Aim" to operate correctly.
    } else if (flight.getRawButton(flight6) == true) {
      initAutoReturn = true;
      AngleControl(limecloseAngleRef, .435, 180);
      if (currentDif > -0.01 && currentDif < 0.01) {
        initAutoReturn = false;
      }
    } else if (initAutoReturn) {
      AngleControl(limecloseAngleRef, .435, 180);
      if (currentDif > -0.01 && currentDif < 0.01) {
        initAutoReturn = false;
      }
    } else {
      m_AngleLEAD.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // ********************************
  // * CUSTOM FUNCTIONS
  // ********************************

  /**
   * This function creates a wait
   * 
   * @param time = Time to wait in milliseconds.
   */
  public static void wait(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }

  /**
   * Calculates the angle for shooting using the distance sensor.
   */
  public double calculateMotorAngle() {
    var calcs1 = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    double distanceToTarget = calcs1.getZ();
    double interpolatedAngle;
    // Check if the distance is within the range of reference points
    if (distanceToTarget <= limecloseDistanceRef) {
      interpolatedAngle = limecloseAngleRef; // Use the close angle directly
      return interpolatedAngle;
    } else if (distanceToTarget >= limefarDistanceRef) {
      interpolatedAngle = limefarAngleRef; // Use the far angle directly
      return interpolatedAngle;
    } else {
      // Perform linear interpolation for angles between close and far distances
      double closeToFarRatio = (distanceToTarget - limecloseDistanceRef) / (limefarDistanceRef - limecloseDistanceRef);
      interpolatedAngle = limecloseAngleRef + closeToFarRatio * (limefarAngleRef - limecloseAngleRef);
      return interpolatedAngle;
    }
  }

  /**
   * Determines if speaker is within shooting distance.
   */
  public boolean getShootSafe() {
    double distanceToTarget = getLimeZ();
    if (distanceToTarget > 2.57 || distanceToTarget == 0) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Gets the current Limelight Z distance.
   */
  public double getLimeZ() {
    var calcs1 = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    double z = calcs1.getZ();
    return z;
  }

  /**
   * This function controls the tilt of the m_Angle motor.
   * 
   * @param DesiredPos  = Position m_Angle should be set to.
   * @param Angle_Speed = Angle speed can be "normalAngleSpeed" or
   *                    "hangAngleSpeed."
   * @param Slope       = Slope to be used with equation.
   */
  public void AngleControl(double DesiredPos, double Angle_Speed, double Slope) {
    double currentPos = m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition();
    currentDif = (DesiredPos - currentPos);
    if (DesiredPos == currentPos) {
      m_AngleLEAD.set(0);
    } else {
      if (currentDif >= 0) {
        raw_speed = (Slope * (Math.pow(currentDif, 2))); // Initial speed calculation
        if (raw_speed > Angle_Speed) { // Check to ensure speed is not exceeding 100%
          m_AngleLEAD.set(Angle_Speed);
        } else {
          m_AngleLEAD.set(1 * raw_speed);
        }
      } else {
        raw_speed = (Slope * (Math.pow(currentDif, 2))); // Initial speed calculation
        if (raw_speed > Angle_Speed) { // Check to ensure speed is not exceeding 100%
          m_AngleLEAD.set(-1 * Angle_Speed);
        } else {
          m_AngleLEAD.set(-1 * raw_speed);
        }
      }
    }
  }

  /**
   * Automatically hangs the robot.
   */
  public void AutoHang() {
    AngleControl(shooterTiltClimb, hangAngleSpeed, hangAngleSlope);
    SmartDashboard.putNumber("Angle", m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition());
    SmartDashboard.putNumber("Angle Speed", raw_speed);
  }

  /**
   * Drive straight with encoders
   * 
   * @param power    = Speed to drive the drivetrain
   *                 (negative drives lime side, positive drives battery side)
   * @param distance = How far to drive in feet
   */
  public void driveStraightWithEncoder(double power, double distance) {
    double error = Math.abs(m_leftMotorLEAD.getEncoder().getPosition()
        - m_rightMotorLEAD.getEncoder().getPosition()); // Difference between the encoder on the left and rights sides.
    double kP = 0.05; // Proportional control for driving straight.
    double feet = Math.abs(((m_rightMotorLEAD.getEncoder().getPosition() + m_leftMotorLEAD.getEncoder().getPosition()) / 2)
            / 5.380945682525635);
    if (Math.abs(feet) < distance) {
      Drive_Main.tankDrive((power + (kP * error)), (power + (kP * error)));
    } else {
      Drive_Main.stopMotor();
      if (!hasDriveS1) {
        hasDriveS1 = true;
        System.out.println("STEP 1 DONE");
        m_rightMotorLEAD.getEncoder().setPosition(0);
        m_leftMotorLEAD.getEncoder().setPosition(0);
      } else if (hasTurnS2) {
        hasDriveS3 = true;
        System.out.println("STEP 3 DONE");
        m_rightMotorLEAD.getEncoder().setPosition(0);
        m_leftMotorLEAD.getEncoder().setPosition(0);
      }
    }
  }

  /**
   * Turn based on arc length
   * 
   * @param power      = Speed to drive
   * @param arc_length = How far to drive in feet
   * @param direction  = Turning direction, "kLeft" or "kRight"
   */
  public void ArcLengthTurnWithEncoder(double power, String direction, double arc_length) {
    double error = 0;
    double kP = 1; // Proportional control for driving straight
    double Lfeet = m_leftMotorLEAD.getEncoder().getPosition() / 5.380945682525635;
    double Rfeet = m_rightMotorLEAD.getEncoder().getPosition() / 5.380945682525635;

    if (direction == kRight && Math.abs(Rfeet) < arc_length) {
      Drive_Main.tankDrive(0, power + (kP * error));
    } else if (direction == kLeft && Math.abs(Lfeet) < arc_length) {
      Drive_Main.tankDrive(power + (kP * error), 0);
    } else {
      Drive_Main.stopMotor();
      if (!hasTurnS2) {
        hasTurnS2 = true;
        System.out.println("STEP 2 DONE");
        m_rightMotorLEAD.getEncoder().setPosition(0);
        m_leftMotorLEAD.getEncoder().setPosition(0);
      } else if (hasDriveS3) {
        hasTurnS4 = true;
        System.out.println("STEP 4 DONE");
        m_rightMotorLEAD.getEncoder().setPosition(0);
        m_leftMotorLEAD.getEncoder().setPosition(0);
      }
    }
  }
}