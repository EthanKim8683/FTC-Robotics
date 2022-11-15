# Basic Features

## Basic Practices

### Chaining

```java
// The block below:
objectHere.setAttribute1(10);
objectHere.setAttribute2(12);
objectHere.subscribeEvent1(32, 92);
objectHere.subscribeEvent2(10);
objectHere.setAttribute(76);

// Can be simplified to the one-liner below due to
// chaining:
objectHere
  .setAttribute1(10)
  .setAttribute2(12)
  .subscribeEvent1(32, 92)
  .subscribeEvent2(10)
  .setAttribute(76);
```

## Using the Event-Driven API

### Async Object: Asynchronous event object

Initialization
```java
// Create a new Async object; A function is passed that
// is called whenever the object is `.execute()`-ed:
Async example = new Async(async -> {
  telemetry.addData("Status", "Asynchronously doing stuff!");
  telemetry.update();
  async.finish(); // When the execution is complete,
                  // `async.finish()` must be called
});

// Create and link two Async objects; once the left-hand
// Async object is `finish()`-ed, the right-hand Async
// object is executed:

// Here, eventB only executes once eventA is finished
Async eventA = new Async(async -> {
  telemetry.addData("Status", "Do first!");
  telemetry.update();
  async.finish();
});
Async eventB = new Async(async -> {
  telemetry.addData("Status", "Do second!");
  telemetry.update();
  async.finish();
});
Async.addLink(eventA, eventB); // Linking happens here!

// Link multiple left-hand Async objects to one 
// right-hand Async object; Once all left-hand Async
// objects have been finished, the right-hand Async
// object may be executed:

// Here, eventE only executes once eventC and eventD are
// finished
Async eventC = new Async(async -> {
  telemetry.addData("Status", "Do before eventE!");
  telemetry.update();
  async.finish();
});
Async eventD = new Async(async -> {
  telemetry.addData("Status", "Also do before eventE!");
  telemetry.update();
  async.finish();
});
Async eventE = new Async(async -> {
  telemetry.addData("Status", "Do once eventC and eventD are done!");
  telemetry.update();
  async.finish();
});
Async.addLink(eventC, eventE); // Linking happens here!
Async.addLink(eventD, eventE);
```
Runtime:
```java
// An asynchronous event may be executed by calling
// `.execute()`:
example.execute();
```

### EventManager.EventHandler: Basic event handler

Initialization
```java
// You will rarely encounter this feature on its own, but
// here it is for the sake of thoroughness:
EventManager.EventHandler example = () -> {
  telemetry.addData("Status", "Hello from an EventHandler!");
  telemetry.update();
  return false; // If true, do not call again
                // If false, keep calling if event
                // happens again
};

// Most likely you would write it into a `subscribe`
// function, like:

// For DcMotorWrapper:
DcMotorWrapper dcMotorWrapperExample = new DcMotorWrapper()
  .setDcMotor(hardwareMap.dcMotor.get("ExampleMotor"))
  .subscribeBecomeBusyEvent(() -> {
    telemetry.addData("Status", "Hello from a DcMotorWrapper EventHandler!");
    telemetry.update();
    return false;
  });
  
// For GamepadWrapper:
GamepadWrapper gamepadWrapperExample = new GamepadWrapper()
  .setGamepad(gamepad1)
  .subscribeXPressedEvent(() -> {
    telemetry.addData("Status", "Hello from a GamepadWrapper EventHandler!");
    telemetry.update();
    return false;
  });

// ...and others
```

### OdometryWrapper.PoseEventHandler: Pose event handler

```java
// You will also rarely encounter this feature on its
// own, but here it is for the sake of thoroughness:
OdometryWrapper.PoseEventHandler example = (x, y, r) -> {
  // In this case, position checking happens from within
  // the function; the line below terminates the handler
  // once the OdometryWrapper object's x reading is
  // greater than 60.0 inches:
  if (x > 60.0) return true;

  telemetry.addData("Status", "Hello from a PoseEventHandler!");
  telemetry.update();
  return false; // If true, do not call again
                // If false, keep calling if event
                // happens again
};
  
// Subscription looks like this:
OdometryWrapper odometryWrapperExample = new GamepadWrapper()
  .setLeftEncoder(hardwareMap.dcMotor.get("LeftEncoder"))
  .setRightEncoder(hardwareMap.dcMotor.get("RightEncoder"))
  .setFrontEncoder(hardwareMap.dcMotor.get("FrontEncoder"))
  .subscribePoseEvent((x, y, r) -> {
    if (x > 60.0) return true;
    telemetry.addData("Status", "Hello from a GamepadWrapper EventHandler!");
    telemetry.update();
    return false;
  });
```

### TimeManager.TimeEventHandler: Time event handler

```java
// Currently there are no ways to access the
// TimeEventHandler directly + may be due for refactoring
```

## Using the Wrappers

### DcMotorWrapper: Wrapper for DcMotor object

Initialization:
```java
// Set DcMotor of DcMotorWrapper
DcMotorWrapper example = new DcMotorWrapper()
  .setDcMotor(hardwareMap.dcMotor.get("ExampleMotor"));

// For setting lower and upper bounds individually
example.setLowerBound(0);   // Default is 0
example.setUpperBound(100); // Default is 1

// For setting lower and upper bound at the same time
example.setRange(0, 100);

// For setting forward (towards positive) and reverse
// (towards negative) powers individually
example.setForwardPower(0.2); // Default is 0.5
example.setReversePower(0.8); // Default is 0.5

// For setting forward and reverse powers each to their
// own power
example.setPowers(0.2, 0.8);

// For setting forward and reverse powers to the same
// power
example.setPower(0.2);
```
Events:
```java
// Create an EventManager.EventHandler that is called
// whenever the DcMotor becomes busy
example.subscribeBecomeBusyEvent(() -> {
  telemetry.addData("Status", "Now busy!");
  telemetry.update();
  return false;
});

// Create an EventManager.EventHandler that is called
// whenever the DcMotor becomes idle
example.subscribeBecomeIdleEvent(() -> {
  telemetry.addData("Status", "Now idle!");
  telemetry.update();
  return false;
});
```
Runtime:
```java
// Set position to value between lower and upper bound,
// range is [0.0, 1.0]
example.setPosition(0.5);

// Set position to any value, range is any value the
// DcMotor is capable of handling
example.setPosition(200);

// Go to position, returns an Async object that finishes
// when the motor has reached that position
Async gotoComplete = new Async(example.gotoPosition(0.5));
```

### GamepadWrapper: Wrapper for Gamepad object

Initialization:
```java
// Set Gamepad of GamepadWrapper
GamepadWrapper example = new GamepadWrapper()
  .setGamepad(gamepad1);
```
Events:
```java
// Create an EventManager.EventHandler that is called
// whenever the the X button is pressed
example.subscribeXPressedEvent(() -> {
  telemetry.addData("Status", "X button pressed!");
  telemetry.update();
  return false;
});

// ...and others:
// - subscribeYPressedEvent
// - subscribeAPressedEvent
// - subscribeBPressedEvent
// - subscribeUPressedEvent (dpad_up)
// - subscribeDPressedEvent (dpad_down)
// - subscribeLPressedEvent (dpad_left)
// - subscribeRPressedEvent (dpad_right)
// - subscribeLbPressedEvent (left_bumper)
// - subscribeRbPressedEvent (right_bumper)
// - subscribeXReleasedEvent
// - subscribeYReleasedEvent
// - subscribeAReleasedEvent
// - subscribeBReleasedEvent
// - subscribeUReleasedEvent (dpad_up)
// - subscribeDReleasedEvent (dpad_down)
// - subscribeLReleasedEvent (dpad_left)
// - subscribeRReleasedEvent (dpad_right)
// - subscribeLbReleasedEvent (left_bumper)
// - subscribeRbReleasedEvent (right_bumper)
```

Runtime:
```java
// To-do: Implement macros
```

### MecanumWrapper: Wrapper for mecanum drivetrain

### OdometryWrapper: Wrapper for 3-wheel odometry

### PIDWrapper: Wrapper for PID controller

### ServoWrapper: Wrapper for Servo object