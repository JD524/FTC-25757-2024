package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.Timer;
import java.util.TimerTask;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOp 2025", group = "Iterative Opmode")
public class TeleOp25 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Timer timer = new Timer();

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx elbow;
    private DcMotorEx lift;

    private Servo claw;
    private Servo wrist;
    private double motorPower = 0.7;

    private double Kp = 0.5;
    private double Ki = 0.0;
    private double Kd = 0.0;

    private int elbowPosition;
    private int liftPosition;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP... maybe 1??
    static final double WHEEL_DIAMETER_INCHES = 10 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (12 / 9.6)/* (12/10.25) */ * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing ...");
        telemetry.update();

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        runtime.reset();

        claw.setPosition(0.5); //change later
        wrist.setPosition(0.5); //also change later
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //DO NOT TOUCH: DEBUGGER
        telemetry.addData("motorPower: ", motorPower);
        telemetry.addData("lift.currentPosition(): ", lift.getCurrentPosition());
        telemetry.addData("elbow.getCurrentPosition(): ", elbow.getCurrentPosition());
        telemetry.addData("claw.getPosition(): ", claw.getPosition());
        telemetry.addData("wrist.getPosition()", wrist.getPosition());

        telemetry.update();

        //drive and strafe
        if (gamepad1.left_stick_x < -0.85) {
            strafeRight(motorPower);
        }
        else if (gamepad1.left_stick_x > 0.85) {
            strafeLeft(motorPower);
        }
        else {
            if (Math.abs(this.gamepad1.right_stick_x) > 0.3) {
                frontRight.setPower(motorPower * 0.775 * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                frontLeft.setPower(motorPower * 0.775 * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                backRight.setPower(motorPower * 0.775 * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
                backLeft.setPower(motorPower * 0.775 * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
            }
            else {
                frontRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                frontLeft.setPower(motorPower * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                backRight.setPower(motorPower * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
                backLeft.setPower(motorPower * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x));
            }
        }

        if(gamepad2.right_stick_y > 0.2) {
            elbowPosition = elbow.getCurrentPosition() - 100;

            elbow.setTargetPosition(elbowPosition);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(0.7);

            telemetry.addData("elbow.getCurrentPosition() right stick:", elbow.getCurrentPosition());
            telemetry.update();
        } else if (gamepad2.right_stick_y < -0.2) {
            elbowPosition = elbow.getCurrentPosition() + 100;

            elbow.setTargetPosition(elbowPosition);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(-0.7);

            telemetry.addData("elbow.getCurrentPosition() right stick:", elbow.getCurrentPosition());
            telemetry.update();
        }

        if (gamepad2.left_stick_y > 0.2) {
            liftPosition = lift.getCurrentPosition() - 100;

            lift.setTargetPosition(liftPosition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.95);

            telemetry.addData("lift.getCurrentPosition() left stick:", lift.getCurrentPosition());
            telemetry.update();
        } else if (gamepad2.left_stick_y < -0.2) {
            liftPosition = lift.getCurrentPosition() + 100;

            lift.setTargetPosition(liftPosition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(-0.95);

            telemetry.addData("lift.getCurrentPosition() left stick:", lift.getCurrentPosition());
            telemetry.update();
        }

        if (gamepad1.left_bumper) {
            claw.setPosition(claw.getPosition() + 0.4);

            telemetry.addData("claw.getPosition() left bumper:", claw.getPosition());
            telemetry.update();
        }

        if (gamepad1.right_bumper) {
            claw.setPosition(claw.getPosition() - 0.4);

            telemetry.addData("claw.getPosition() right bumper:", claw.getPosition());
            telemetry.update();
        }

        if (gamepad2.left_bumper) {
            wrist.setPosition(0.0);

            telemetry.addData("wrist.getPosition() left bumper:", wrist.getPosition());
            telemetry.update();
        }

        if (gamepad2.right_bumper) {
            wrist.setPosition(0.5);

            telemetry.addData("wrist.getPosition() right bumper:", wrist.getPosition());
            telemetry.update();
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    private void encoderDrive(int liftInches, double speed) {
        lift.setTargetPosition(liftInches);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while(lift.getCurrentPosition() >= liftInches) {
            lift.setPower(speed);
        }
    }

    protected void drive(double speed,
                         double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches,
                         double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        final double MINSPEED = 0.30; // Start at this power
        final double SPEEDINCR = 0.2; // And increment by this much each cycle
        double curSpeed; // Keep track of speed as we ramp

        //DUE TO ORIENTATION OF MOTORS, LEFT MOTORS HAVE TO HAVE SIGN REVERSED FOR DISTANCES. maybe.
        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (-FrontLeftInches * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (FrontRightInches * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (-BackLeftInches * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (BackRightInches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        while (frontLeft.getTargetPosition() != newFrontLeftTarget) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
        }
        while (frontRight.getTargetPosition() != newFrontRightTarget) {
            frontRight.setTargetPosition(newFrontRightTarget);
        }
        while (backLeft.getTargetPosition() != newBackLeftTarget) {
            backLeft.setTargetPosition(newBackLeftTarget);
        }
        while (backRight.getTargetPosition() != newBackRightTarget) {
            backRight.setTargetPosition(newBackRightTarget);
        }

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // curSpeed = Math.abs(speed); // Make sure its positive
        curSpeed = 0.3;
        //curSpeed = Math.min(MINSPEED, speed);

        frontLeft.setPower(curSpeed);
        backLeft.setPower(curSpeed);
        frontRight.setPower(curSpeed);
        backRight.setPower(curSpeed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy()&& backRight.isBusy())) { //might need to change this to all being busy..
        }
        // And rewrite the motor speeds
        frontLeft.setPower(curSpeed);
        backLeft.setPower(curSpeed);
        frontRight.setPower(curSpeed);
        backRight.setPower(curSpeed);

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeRight(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }

    protected void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}