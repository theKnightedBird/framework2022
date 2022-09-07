package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI
import kotlin.math.abs

open class SwerveDrive(
  private val modules: List<SwerveModule>,
  val ahrs: AHRS,
  private val turnPID: PIDController,
  override val maxLinearSpeed: Double,
  override val maxRotSpeed: Double,
  var driveStraight: () -> Boolean = { true }
) : SubsystemBase(), HolonomicDrive {
  init {
    // Zero out the gyro
    ahrs.calibrate()
    // set up turning PID
    turnPID.enableContinuousInput(-PI, PI)
    turnPID.setTolerance(2 * PI / 180) // Tolerance of 2 degrees for the navx noise
  }

  val kinematics = SwerveDriveKinematics(
    *this.modules
      .map { it.location }.toTypedArray()
  )

  private var desiredHeading = ahrs.heading
  private var cachedDriveStraight = driveStraight()
  private var prevTime = Double.NaN
  private val odometry = SwerveDriveOdometry(this.kinematics, -ahrs.heading)

  @Log.ToString
  var desiredSpeeds = ChassisSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
  }

  override var heading: Rotation2d
    @Log.ToString
    get() {
      return -ahrs.heading
    }
    set(newHeading) {
      ahrs.heading = newHeading
    }

  override var pose: Pose2d
    @Log.ToString
    get() {
      return this.odometry.poseMeters
    }
    set(newPose) {
      this.odometry.resetPosition(newPose, heading)
    }

  override fun stop() {
    this.set(ChassisSpeeds())
  }

  override fun periodic() {
    val currTime = Timer.getFPGATimestamp()

    if (prevTime.isNaN()) {
      prevTime = currTime - .02
    }

    val dt = currTime - prevTime

    // If we want to drive straight and the chassis is not trying to turn, then don't let it
    if (driveStraight() && abs(desiredSpeeds.omegaRadiansPerSecond) <= 0.001) { // mess with this threshold
      if (!cachedDriveStraight) desiredHeading = heading
      desiredHeading = desiredHeading.plus(Rotation2d(desiredSpeeds.omegaRadiansPerSecond * dt))
      val adjustedOmegaPerSecond = turnPID.calculate(heading.radians, desiredHeading.radians)
      desiredSpeeds.omegaRadiansPerSecond =
        clamp(
          adjustedOmegaPerSecond,
          -maxRotSpeed, maxRotSpeed
        )
    }

    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      DriveConstants.MAX_ATTAINABLE_SPEED
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    this.odometry.update(
      heading,
      *this.modules
        .map { it.state }.toTypedArray()
    )

    for (module in modules)
      module.update()

    cachedDriveStraight = driveStraight()
    prevTime = currTime
  }

  companion object {
    /**
     * Create a square swerve drivetrain
     *
     * @param ahrs Gyro used for robot heading
     * @param maxLinearSpeed Max speed (m/s) at which the robot can translate
     * @param maxRotSpeed Max speed (rad/s) at which the robot can turn in place
     * @param turnPID PID controller for the heading of the robot
     * @param frontLeftDriveMotor
     * @param frontRightDriveMotor
     * @param backLeftDriveMotor
     * @param backRightDriveMotor
     * @param frontLeftTurnMotor
     * @param frontRightTurnMotor
     * @param backLeftTurnMotor
     * @param backRightTurnMotor
     * @param frontLeftLocation Location of the front left module
     * @param driveMotorController Supplier to make copies of the same driving PID controllers for all
     *     the modules
     * @param turnMotorController Supplier to make copies of the same turning PID controllers for all
     *     the modules
     * @param driveFeedforward Driving feedforward for all the modules
     * @param turnFeedforward Turning feedforward for all the modules
     * @param driveStraight turn on=true or off=false function to command robot to maintain heading
     */
    fun squareDrive(
      ahrs: AHRS,
      maxLinearSpeed: Double,
      maxRotSpeed: Double,
      turnPID: PIDController,
      frontLeftDriveMotor: WrappedMotor,
      frontRightDriveMotor: WrappedMotor,
      backLeftDriveMotor: WrappedMotor,
      backRightDriveMotor: WrappedMotor,
      frontLeftTurnMotor: WrappedMotor,
      frontRightTurnMotor: WrappedMotor,
      backLeftTurnMotor: WrappedMotor,
      backRightTurnMotor: WrappedMotor,
      frontLeftLocation: Translation2d,
      driveMotorController: () -> PIDController,
      turnMotorController: () -> PIDController,
      driveFeedforward: SimpleMotorFeedforward,
      turnFeedforward: SimpleMotorFeedforward,
      driveStraight: () -> Boolean = { true }
    ): SwerveDrive {
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          frontLeftDriveMotor,
          frontLeftTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation
        ),
        SwerveModule.create(
          "FRModule",
          frontRightDriveMotor,
          frontRightTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation.rotateBy(Rotation2d.fromDegrees(-90.0))
        ),
        SwerveModule.create(
          "BLModule",
          backLeftDriveMotor,
          backLeftTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation.rotateBy(Rotation2d.fromDegrees(90.0))
        ),
        SwerveModule.create(
          "BRModule",
          backRightDriveMotor,
          backRightTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation.rotateBy(Rotation2d.fromDegrees(180.0))
        )
      )
      return SwerveDrive(
        modules,
        ahrs,
        turnPID,
        maxLinearSpeed,
        maxRotSpeed,
        driveStraight
      )
    }

    /**
     * @return the sim version of this drive
     */
    fun simOf(swerve: SwerveDrive): SwerveSim {
      return SwerveSim(swerve.modules, swerve.ahrs, swerve.maxLinearSpeed, swerve.turnPID, swerve.maxRotSpeed)
    }
  }
}
