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
    
    //DIO
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;

    // Test Robot Configuration

    public static final int kDriveEncoderTicksPerRevolution = 2048;
    public static final double kDriveWheelDiameter = 0.15;
    public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;
    
    //Drive Characteristics
    public static final double ksVolts = 0.149;
    public static final double kvVoltSecondsPerMeter = 2.86;
    public static final double kaVoltSecondsSquaredPerMeter = 0.252;
    
    //
    public static final double kTrackwidthMeters = 0.6223;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //public static final double kMaxSpeedMetersPerSecond = 1.25;
    //public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;

    public static final double kMaxSpeedMetersPerSecond = 2.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.25;

    //Path Following
    public static final double kPathFollowingkP = 4.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
