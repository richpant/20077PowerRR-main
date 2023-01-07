package org.firstinspires.ftc.teamcode.drive; //.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    private DcMotor leftFront;    // 0
    private DcMotor leftRear;     // 2
    private DcMotor rightFront;   // 1
    private DcMotor rightRear;    // 3
    private DcMotor lift;         // 0 expansion hub
    private Servo claw;           // s4
    private Servo claw2;          // s5
    private Servo flag;           // expansion port 0
    private DigitalChannel touch; // expansion hub digital device-port 0
    private ColorSensor color;    // 12C port 2 control hub

    public void runOpMode() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Encoder value", lift.getCurrentPosition());
        telemetry.update();
    }


    @Override
    public void init() {
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear    = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear   = hardwareMap.get(DcMotor.class, "rightRear");
        lift        = hardwareMap.get(DcMotor.class, "lift");
        flag        = hardwareMap.get(Servo.class,"flag");
        claw        = hardwareMap.get(Servo.class, "claw");
        claw2       = hardwareMap.get(Servo.class, "claw2");
        touch       = hardwareMap.get(DigitalChannel.class, "touch");
        color       = hardwareMap.get(ColorSensor.class,"color");

        touch.setMode(DigitalChannel.Mode.INPUT);

        claw.setPosition(0.64);
        claw2.setPosition(0.36);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //----------------------------Mecanum-Drive-Code----------------------------\\

        float lift_power = -gamepad2.left_stick_y;
        double y = -gamepad1.left_stick_y; //Forward/Backward
        double x = gamepad1.left_stick_x * 1.1; //Strafe Left/Right * Counteract imperfect strafing
        double rx = gamepad1.right_stick_x; //Turn Left/Right

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower   = (y + x + rx) / denominator;
        double backLeftPower    = (y - x + rx) / denominator;
        double frontRightPower  = (y - x - rx) / denominator;
        double backRightPower   = (y + x - rx) / denominator;

        //----------------------------Drive-Speed----------------------------\\

        leftFront.setPower(0.7 * frontLeftPower);
        leftRear.setPower(0.7 * backLeftPower);
        rightFront.setPower(0.7 * frontRightPower);
        rightRear.setPower(0.7 * backRightPower);

        if (gamepad1.left_bumper) {//speed for going across field
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }
        if (gamepad1.right_bumper) {//slow for corrections
            leftFront.setPower(0.2 * frontLeftPower);
            leftRear.setPower(0.2 * backLeftPower);
            rightFront.setPower(0.2 * frontRightPower);
            rightRear.setPower(0.2 * backRightPower);
        }

        //----------------------------Lift----------------------------\\

        lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        //----------------------------Lift-And-Touch-Sensor----------------------------\\

        if (touch.getState() == true) {
            telemetry.addData("touch", "is not pressed");
            lift.setPower(lift_power);
        }

        /*if (touch.getState() == false) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setPower(0);
            resetRuntime();
            while (getRuntime() < 500 && touch.getState() == false) {
                lift.setPower(-1);
            }

            telemetry.addData("touch", "pressed");
        } else {
            touch.setState(true);
        }*/

        //----------------------------Claw----------------------------\\

        if (gamepad2.b) {//open
            claw.setPosition(0.64);//white
            telemetry.addData("claw1", claw.getPosition());
            claw2.setPosition(0.36);//black
            telemetry.addData("claw2", claw.getPosition());
        }
        if (gamepad2.y) {//close
            claw.setPosition(0.495);
            claw2.setPosition(0.505);
        }

        //----------------------------Light-Sensor-And-Flag----------------------------\\

        if (color.red()>=200 || color.blue()>=250) {
            flag.setPosition(0);
            telemetry.addData("bluevalue", color.blue());
            telemetry.addData("redvalue", color.red());
            telemetry.addData("cone","seen");
        } else {
            flag.setPosition(0.5);
            telemetry.addData("bluevalue", color.blue());
            telemetry.addData("redvalue", color.red());
        }

        /*----------------------------Telemetry----------------------------\\

        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("leftRear", leftRear.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("rightRear", rightRear.getCurrentPosition());
        telemetry.addData("claw1", claw.getPosition());
        telemetry.addData("claw2", claw2.getPosition()); */
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.update();

    }
}