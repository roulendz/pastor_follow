You’re thinking in exactly the right direction:
motor feedback + camera frame must be treated as a paired measurement, not two worlds that “hopefully” line up.

Let’s design this cleanly so:
	•	We never assume the motor reached the command angle.
	•	Every frame is tagged with the actual angle.
	•	FOV learning and centering stay modular (single responsibility).
	•	Code stays DRY, not a ball of spaghetti.

I’ll split it into:
	1.	Core idea: how to sync motor + camera
	2.	High-level architecture (small, focused modules)
	3.	Concrete data flow per frame
	4.	Where and how we compute Δθ & Δx
	5.	How to keep code clean (SRP/DRY)

⸻

1. Core idea: every frame gets a motor angle snapshot

Instead of:
	•	“I send move 5°, then later I hope it stopped, then I read frame”

We do this:
	•	Camera is the heartbeat (e.g. 30 FPS).
	•	For each frame:
	1.	Capture frame
	2.	Read the latest motor state (angle, timestamp) from the motor module
	3.	Bundle them into a Sample object

This Sample is your single source of truth:

struct Sample {
    double t;          // timestamp when frame was captured
    float angle_deg;   // motor-reported angle at (or very near) that time
    float x_person_px; // person center in pixels
    bool  person_valid;
};

Now:
	•	FOV learning uses pairs of Samples → no ambiguity.
	•	You don’t care when the motor “stops”; you just use its reported angle for each frame.

⸻

2. Architecture: separate modules, no monolith

Think in terms of 4 main responsibilities:
	1.	MotorInterface
	•	Talks to the stepper controller.
	•	Knows target angle, current angle, timestamps.
	•	API examples:

struct MotorState { float angle_deg; double t; };
void sendTargetAngle(float angle_deg);
MotorState getLatestState();


	2.	CameraInterface
	•	Captures frames from the camera.
	•	API:

cv::Mat grabFrame(double& timestamp_out);


	3.	PoseTracker
	•	Takes a frame, returns “where is the person horizontally?”.
	•	API:

struct PoseResult { float x_person_px; float confidence; };
PoseResult estimatePose(const cv::Mat& frame);


	4.	TrackerController
	•	High-level brain:
	•	builds Sample
	•	updates FOV estimate
	•	computes angle error
	•	sends motor commands
	•	Uses MotorInterface + CameraInterface + PoseTracker + FovEstimator.
	5.	FovEstimator (small, self-contained)
	•	Responsible ONLY for estimating anglePerPixel / fov_est.
	•	API:

class FovEstimator {
public:
    void reset(float initial_anglePerPixel);
    void updateWithSample(const Sample& s);
    float getAnglePerPixel() const;
    float getFovDeg(int imgWidth) const;
};



Now your code isn’t monolithic; each piece does one thing.

⸻

3. Per-frame flow (main loop)

Pseudo-loop in TrackerController:

void TrackerController::tick() {
    // 1. Grab frame + timestamp
    double t_frame;
    cv::Mat frame = camera.grabFrame(t_frame);

    // 2. Get current motor angle (snapshot)
    MotorState m = motor.getLatestState(); // angle_deg + t

    // 3. Run pose estimation
    PoseResult pose = poseTracker.estimatePose(frame);

    // 4. Build sample
    Sample s;
    s.t           = t_frame;
    s.angle_deg   = m.angle_deg;      // latest known angle
    s.person_valid= pose.confidence > 0.5f;
    s.x_person_px = pose.x_person_px; // anything if !person_valid

    // 5. Update FOV estimator (learning loop)
    fovEstimator.updateWithSample(s);

    // 6. Compute control action (centering loop)
    controlFromSample(s);
}

Notice:
	•	We never wait for “motor stopped”.
	•	We always use the current reported angle when the frame arrives.

If motor lags or never stops, we still have: “frame at angle X, next frame at angle Y” → still valid Δθ.

⸻

4. How to compute Δθ & Δx cleanly (inside FovEstimator)

Inside FovEstimator, we keep its own state, independent of motor or camera:

class FovEstimator {
public:
    void reset(float initial_anglePerPixel) {
        anglePerPixel_est = initial_anglePerPixel;
        hasPrev = false;
    }

    void updateWithSample(const Sample& s) {
        if (!s.person_valid) {
            hasPrev = false; // can't learn without a stable target
            return;
        }

        if (!hasPrev) {
            prev = s;
            hasPrev = true;
            return;
        }

        float dAngle = s.angle_deg - prev.angle_deg;
        float dX     = s.x_person_px - prev.x_person_px;

        // Only learn when there is significant motion
        if (fabs(dAngle) > minAngleDeg && fabs(dX) > minPixels) {
            float sample_anglePerPixel = dAngle / dX;

            // Exponential moving average (smooth+stable)
            float alpha = 0.05f; // small → slow changes
            anglePerPixel_est = (1.0f - alpha) * anglePerPixel_est
                              + alpha * sample_anglePerPixel;

            prev = s; // update baseline
        }
    }

    float getAnglePerPixel() const { return anglePerPixel_est; }
    float getFovDeg(int imgWidth) const { return anglePerPixel_est * imgWidth; }

private:
    Sample prev;
    bool   hasPrev = false;
    float  anglePerPixel_est = 0.05f; // some default
    const float minAngleDeg = 1.0f;
    const float minPixels   = 10.0f;
};

Key points:
	•	All the learning logic is encapsulated here → single responsibility.
	•	It just needs Samples, nothing else.
	•	You can unit-test FovEstimator separately.

⸻

5. Control loop: separate from FOV learning

In TrackerController::controlFromSample(s):

void TrackerController::controlFromSample(const Sample& s) {
    if (!s.person_valid) {
        // no person → maybe hold current angle or slowly return to neutral
        return;
    }

    float anglePerPixel = fovEstimator.getAnglePerPixel();
    float x_center      = imgWidth * 0.5f;
    float pixelOffset   = s.x_person_px - x_center;
    float angle_error   = pixelOffset * anglePerPixel; // deg

    // Deadband
    const float deadband = 0.3f;
    if (fabs(angle_error) < deadband) angle_error = 0.0f;

    // P control on angle (or on speed)
    float Kp = 1.0f;
    float correction = Kp * angle_error;

    float target_angle = s.angle_deg + correction;
    target_angle = std::clamp(target_angle, minAngle, maxAngle);

    motor.sendTargetAngle(target_angle);
}

Again:
	•	Control loop doesn’t know how anglePerPixel is learned.
	•	FovEstimator doesn’t know how motor is controlled.
	•	Separation of concerns → easier to maintain and refactor.

⸻

6. Handling “motor never stops” & sync best practices

“we don’t know when the motor stops… we must wait for response”

With the design above, you don’t need to wait for a “stopped” event for syncing:
	•	You always log actual angle at frame time.
	•	Δθ and Δx are based on real motion, not commands.

Still, some good practices:
	1.	MotorState with timestamps
If your motor controller reports angle at its own rate, log:

struct MotorState { float angle_deg; double t; };

Then when you grab a frame at t_frame, you can:
	•	Take the latest motor state whose timestamp ≤ t_frame (or closest).
	•	Or interpolate between two states if you want extra accuracy.

	2.	Asynchronous motor communication
MotorInterface can run in its own thread or event loop:
	•	Receives periodic “angle_deg, t” from the MCU.
	•	Stores latest MotorState in an atomic or mutex-protected variable.
	•	getLatestState() returns a copy.
	3.	Camera loop stays simple
It just:
	•	Gets frame + time.
	•	Reads last MotorState.
	•	Builds Sample.
	•	Hands Sample to controller+FovEstimator.

This way, your code is:
	•	DRY: Sample is your core abstraction, not duplicated state.
	•	SRP: each module has a clean, narrow API.
	•	No monolith: logic is sliced by responsibility.

⸻

7. TL;DR tailored to your question

How do we sync motor degree movement and the frame so they correspond?

	•	For each frame, immediately read current motor angle and store them together in a Sample.
	•	FOV learning and centering both work on these Samples – never on commands.

How do we avoid messy, monolithic code?

	•	Split into:
	•	MotorInterface
	•	CameraInterface
	•	PoseTracker
	•	FovEstimator
	•	TrackerController
	•	Each has one job and a small API.
	•	FovEstimator has no idea about motor protocol, camera, UI, etc. It just sees Samples.

If you want, next I can sketch the folder structure / class headers for a real C++/Python project so this stays tidy as it grows.