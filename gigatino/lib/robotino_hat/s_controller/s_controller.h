class SCurveMotionController {
public:
  SCurveMotionController(float max_speed, float max_accel, float max_jerk)
      : max_speed_(max_speed), max_accel_(max_accel), max_jerk_(max_jerk),
        speed_(0), accel_(0), peak_speed_(0) {}

  float compute(float target, float current, float dt);
  void reset();
  void set_dist(float dist);
  float max_speed_, max_accel_, max_jerk_;
  float speed_, accel_, peak_speed_;
};
