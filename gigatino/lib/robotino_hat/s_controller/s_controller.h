class SCurveMotionController {
public:
  SCurveMotionController(float min_speed, float max_speed, float max_accel,
                         float max_jerk, float short_dist, float threshold)
      : min_speed_(min_speed), max_speed_(max_speed), max_accel_(max_accel),
        max_jerk_(max_jerk), short_dist_(short_dist), threshold_(threshold),
        speed_(0), accel_(0), peak_speed_(0) {}

  float compute(float target, float current, float dt);
  void reset();
  void plan_curve(float start_pos, float target_pos);

  float min_speed_, max_speed_, max_accel_, max_jerk_, short_dist_, threshold_;
  float speed_, accel_, peak_speed_, peak_accel_;

  bool need_accel;
  bool need_decel;
  float dist_phase_1_, dist_phase_2_, dist_phase_3_, dist_phase_4_,
      dist_phase_5_, dist_phase_6_, dist_phase_7_;
  float start_pos_;
  int curr_phase_;
  bool forward_;

private:
  inline void accelerate_from_zero(float dt);
  inline void accelerate_to_zero(float dt);
  inline void decelerate_to_zero(float dt);
  inline void decelerate_from_zero(float dt);
  inline void max_change(float dt);
};
