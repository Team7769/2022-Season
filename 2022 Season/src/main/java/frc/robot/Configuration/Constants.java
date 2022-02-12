package frc.robot.Configuration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

    // Usb Slots
    public static final int kDriverControllerUsbSlot = 0;
    public static final int kOperatorControllerUsbSlot = 1;
    
    // CAN Device Ids
    public static final int kLeftFrontDriveDeviceId = 2;
    public static final int kLeftRearDriveDeviceId = 3;
    public static final int kRightFrontDriveDeviceId = 4;
    public static final int kRightRearDriveDeviceId = 5;
    public static final int kHoodMotorDeviceId = 10; 
    public static final int kHoodEncoderDeviceId = 4;
    
    // Talon FX Device Ids
    public static final int kLeftShooterMotorDeviceId = 15; 
    public static final int kRightShooterMotorDeviceId= 14;

    // Left Shooter Motor Config Values
    public static final double kLeftMotorKp= .015;
    public static final double kLeftMotorKf= .0473;
    // Hood Target Points
    public static final double kHalfShotValue = 0.5;
    public static final double kQuarterShotValue = 0.25;
    public static final double kThreeQuarterShotValue =0.75;
    // Drive Forward and Shoot Values
    public static final double kDriveForwardStartx= 6.71;
    public static final double kDriveForwardStarty= 2.440;
    public static final double kDriveForwardStartRotation= -2.437994417066567;
    public static final double kDriveForwardMidX = 5.874;
    public static final double kDriveForwardMidY= 2.135;
    public static final double kDriveForwardEndX= 5.08;
    public static final double kDriveForwardEndY= 1.831;
    public static final double kDriveForwardEndRotation= -2.7716865861332294;

    // Collect Two From Terminal Values
    public static final double kCollectTwoWayPointX = 2.6;
    public static final double kCollectTwoWayPointY = 2.078;
    public static final double kCollectTwoEndPointX= 1.128;
    public static final double kCollectTwoEndPointY= 1.156;
    public static final double kCollectTwoEndRotation= -2.442932828868268;
   
    // Drive Back From Terminal Values
    public static final double kDriveBackWayPointX= 2.6;
    public static final double kDriveBackWayPointY= 2.078;
    public static final double kDriveBackEndPointX= 5.08;
    public static final double kDriveBackEndPointY= 1.83;
    public static final double kDriveBackEndPointRotation= -2.7716865861332294;
    
    //Five Ball Part One Values
    public static final double kFiveBallPartOneStartX= 8.663;
    public static final double kFiveBallPartOneStartY= 1.730;
    public static final double kFiveBallPartOneStartRotation= -1.1722738811284743;
   
    public static final double kFiveBallPartOneFirstX= 7.707; 
    public static final double kFiveBallPartOneFirstY= 0.054; 
    public static final double kFiveBallPartOneSecondX= 6.886;
    public static final double kFiveBallPartOneSecondY= 0.740;
    
    public static final double kFiveBallPartOneEndX= 5.818; 
    public static final double kFiveBallPartOneEndY= 0.864; 
    public static final double kFiveBallPartOneEndRotation= -2.4346845928296763;
    
    //Five Ball Part Two To Terminal Values
    public static final double kFiveBallPartTwoToTerminalStartX= 5.818;
    public static final double kFiveBallPartTwoToTerminalStartY= 0.864;

    public static final double kFiveBallPartTwoToTerminalEndX= 1.173;
    public static final double kFiveBallPartTwoToTerminalEndY= 2; 
    public static final double kFiveBallPartTwoToTerminalEndRotation= -2.442932828868268;
    // Five Ball Part Two From Terminal Values
    public static final double kFiveBallPartTwoFromTerminalStartX= 1.173; 
    public static final double kFiveBallPartTwoFromTerminalStartY= 2;
    
    public static final double kFiveBallPartTwoFromTerminalEndX= 5.975;
    public static final double kFiveBallPartTwoFromTerminalEndY= 1.156;
    public static final double kFiveBallPartTwoFromTerminalEndRotation= -2.5878351309802463;
    //DIO
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;

    //Hood PID Values
    public static final double hoodKp= 4.5; 
    public static final double hoodKi= 0;
    public static final double hoodKd= 0;

    // Magnetic Limit Switch Id 
    public static final int magneticLimitSwitchId = 5;

    // Test Robot Configuration
    public static final int kDriveEncoderTicksPerRevolution = 2048;
    public static final double kDriveWheelDiameter = 0.15;
    public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;
    
    //Drive Characteristics
    public static final double ksVolts = 0.149;
    public static final double kvVoltSecondsPerMeter = 2.86;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0;

    // public static final double ksVolts = 0.20123;
    // public static final double kvVoltSecondsPerMeter = 3;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.0;
    
    //
    //public static final double kTrackwidthMeters = 0.6223;
    //public static final double kTrackwidthMeters = 0.9081;
    public static final double kTrackwidthMeters = 1.4;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //public static final double kMaxSpeedMetersPerSecond = 1.25;
    //public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;

    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;

    //Path Following
    public static final double kPathFollowingkP = 4.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
