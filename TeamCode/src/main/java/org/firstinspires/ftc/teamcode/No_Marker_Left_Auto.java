//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.BNO055IMUImpl;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
//import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
//import edu.spa.ftclib.internal.controller.PIDController;
//import edu.spa.ftclib.internal.drivetrain.HeadingableTankDrivetrain;
//import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;
//
///**
// * Created by Gabriel on 2018-06-12.
// * UNTESTED.
// * Demonstrates the use of a HeadingableMecanumDrivetrain to autonomously rotate a robot to specific headings and hold those headings.
// * Tested and found fully functional by Gabriel on 2018-8-4.
// */
//
//@Autonomous(name = "Headingable Tank Rotation Autonomous", group = "sample")
//
//public class No_Marker_Left_Auto extends LinearOpMode {
//    public DcMotor FL;
//    public DcMotor ML;
//    public DcMotor BL;
//    public DcMotor FR;
//    public DcMotor MR;
//    public DcMotor BR;
//
//    public HeadingableTankDrivetrain drivetrain;
//
//    public FinishableIntegratedController controller;
//
//    public BNO055IMUImpl imu;
//
//    /**
//     * Override this method and place your code here.
//     * <p>
//     * Please do not swallow the InterruptedException, as it is used in cases
//     * where the op mode needs to be terminated early.
//     *
//     * @throws InterruptedException
//     */
//    @Override
//    public void runOpMode() throws InterruptedException {   //Notice that this is almost the exact same code as in HeadingableOmniwheelRotationAutonomous and in HeadingableMecanumRotationAutonomous.
////        FL = hardwareMap.get(DcMotor.class, "FL");
//        ML = hardwareMap.get(DcMotor.class, "ML");
////        BL = hardwareMap.get(DcMotor.class, "BL");
////        FR = hardwareMap.get(DcMotor.class, "FR");
//        MR = hardwareMap.get(DcMotor.class, "MR");
////        BR = hardwareMap.get(DcMotor.class, "BR");
////        FL.setDirection(DcMotor.Direction.FORWARD);
////        FR.setDirection(DcMotor.Direction.REVERSE);
//        ML.setDirection(DcMotor.Direction.FORWARD);
//        MR.setDirection(DcMotor.Direction.REVERSE);
////        BL.setDirection(DcMotor.Direction.FORWARD);
////        BR.setDirection(DcMotor.Direction.REVERSE);
//
//        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        //Add calibration file?
//        parameters.loggingEnabled = true;   //For debugging
//        parameters.loggingTag = "IMU";      //For debugging
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
//        imu.initialize(parameters);
//        while (!imu.isGyroCalibrated());
//
//
//        PIDController pid = new PIDController(1.5, 0.05, 0);
//        pid.setMaxErrorForIntegral(0.002);
//
//        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(imu), pid, new ErrorTimeThresholdFinishingAlgorithm(Math.PI/50, 1));
//        //drivetrain = new HeadingableTankDrivetrain(new DcMotor[]{FL, ML, BL, FR, MR, BR}, controller);
//        drivetrain = new HeadingableTankDrivetrain(ML, MR, controller);
//        for (DcMotor motor : drivetrain.motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        waitForStart();
//
////        drivetrain.setTargetHeading(-Math.PI/2);
////        while (drivetrain.isRotating()) {
////            drivetrain.updateHeading();
////            telemetry.addData("Heading", drivetrain.getCurrentHeading());
////            telemetry.update();
////        }
////        sleep(1000);
////
////        drivetrain.setTargetHeading(0);
////        while (opModeIsActive()) drivetrain.updateHeading();
//
//        double driveDistance = drivetrain.getTicksPerUnit()*2;
//        telemetry.addData("Drive Distance",driveDistance);
//        telemetry.addData("Pi", Math.PI);
//        telemetry.update();
////
//        drivetrain.setTargetHeading(-Math.PI/2);
//        drivetrain.rotate();
//        drivetrain.setVelocity(0.5);
//        drivetrain.setTargetPosition(driveDistance);
//        drivetrain.position();
//        drivetrain.setVelocity(0);
////
////        drivetrain.setTargetHeading(-Math.PI);
////        drivetrain.rotate();
////        drivetrain.setVelocity(0.5);
////        drivetrain.setTargetPosition(driveDistance);
////        drivetrain.position();
////        drivetrain.setVelocity(0);
////
////        drivetrain.setTargetHeading(-Math.PI/2);
////        drivetrain.rotate();
////        drivetrain.setVelocity(0.5);
////        drivetrain.setTargetPosition(-driveDistance);
////        drivetrain.position();
////        drivetrain.setVelocity(0);
////
////        drivetrain.setTargetHeading(-Math.PI);
////        drivetrain.rotate();
////        drivetrain.setVelocity(0.5);
////        drivetrain.setTargetPosition(-driveDistance);
////        drivetrain.position();
////        drivetrain.setVelocity(0);
//}
//
//    void doTelemetry() {
//        PIDController pid = (PIDController) drivetrain.controller.algorithm;
//        telemetry.addData("heading, target", drivetrain.controller.getSensorValue()+","+pid.getTarget());
//        telemetry.addData("KP", pid.getKP());
//        telemetry.addData("KI", pid.getKI());
//        telemetry.addData("KD", pid.getKD());
//        telemetry.addData("error", pid.getError());
//        telemetry.addData("integral", pid.getIntegral());
//        telemetry.addData("derivative", pid.getDerivative());
//        telemetry.addData("random", Math.random());
//        telemetry.update();
//    }
//}
