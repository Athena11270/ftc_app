///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//@TeleOp(name="RegularOldDriverFriend", group="Linear Opmode")
//public class Minion extends LinearOpMode {
//
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor FL = null;
//    private DcMotor FR = null;
//    private DcMotor ML = null;
//    private DcMotor MR = null;
//    private DcMotor BL = null;
//    private DcMotor BR = null;
//
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        FL = hardwareMap.get(DcMotor.class, "FL");
//        FR = hardwareMap.get(DcMotor.class, "FR");
//        ML = hardwareMap.get(DcMotor.class, "ML");
//        MR = hardwareMap.get(DcMotor.class, "MR");
//        BL = hardwareMap.get(DcMotor.class, "BL");
//        BR = hardwareMap.get(DcMotor.class, "BR");
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        FL.setDirection(DcMotor.Direction.FORWARD);
//        FR.setDirection(DcMotor.Direction.REVERSE);
//        ML.setDirection(DcMotor.Direction.FORWARD);
//        MR.setDirection(DcMotor.Direction.REVERSE);
//        BL.setDirection(DcMotor.Direction.FORWARD);
//        BR.setDirection(DcMotor.Direction.REVERSE);
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower = gamepad1.left_stick_y;
//            double rightPower = gamepad1.right_stick_y;
//
//            // Tank Mode uses one stick to control each wheel.
//            FL.setPower(leftPower);
//            FR.setPower(rightPower);
//            ML.setPower(leftPower);
//            MR.setPower(rightPower);
//            BL.setPower(leftPower);
//            BR.setPower(rightPower);
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "FL (%.2f), FR (%.2f)", leftPower, rightPower);
//            telemetry.update();
//        }
//    }
//}
