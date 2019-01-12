package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

@TeleOp(name="SLinKo the Elf", group="Linear Opmode")
public class SLinKo extends LinearOpMode {

    // Declare OpMode members.
    private CRServo upDown;
    private CRServo spinner;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        upDown = hardwareMap.get(CRServo.class, "upDown");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.b == true) {
                spinner.setPower(1);
            } else if (gamepad2.a == true) {
                spinner.setPower(-1);
            } else if (gamepad2.x == true) {
                upDown.setPower(1);
            } else if (gamepad2.y == true) {
                upDown.setPower(-1);
            } else {
                spinner.setPower(0);
                upDown.setPower(0);
            }


//            telemetry.addData("Light Var", lightvar);
            telemetry.update();
        }
    }
//    protected void setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind displayKind)
//    {
//        this.displayKind = displayKind;
//        display.setValue(displayKind.toString());
//    }
}
