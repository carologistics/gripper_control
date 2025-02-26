class PIDController {
public:
  PIDController(float kp, float ki, float kd, float max_accel, float max_speed,
                float min_speed)
      : Kp_(kp), Ki_(ki), Kd_(kd), max_accel_(max_accel), max_speed_(max_speed),
        min_speed_(min_speed), integral_(0), prev_error_(0), prev_speed_(0) {}

  float compute(float target, float current, float dt);

  float Kp_, Ki_, Kd_, max_accel_, max_speed_, min_speed_, integral_,
      prev_error_, prev_speed_;
};
