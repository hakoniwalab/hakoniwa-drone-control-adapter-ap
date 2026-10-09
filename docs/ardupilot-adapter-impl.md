# ArduPilot adapter implementation

The adapter targets ArduCopter 4.6.3 and the public Control Link interface at
revision `1814967`.

| Stage | Implementation | Notes |
|---|---|---|
| Altitude | Upstream `AC_P_1D` and two `AC_PID` instances with the `AC_PosControl` z orchestration extracted | Position sqrt control, velocity/acceleration filters and feed-forward, speed/acceleration/jerk limits, hover conversion, and angle boost are retained. |
| Horizontal | Upstream `AC_P_2D` and `AC_PID_2D` with the `AC_PosControl` xy orchestration extracted | Position sqrt control, filtered velocity PID, acceleration/jerk/lean limits, and NED-to-FRD lean conversion are retained. |
| Position3D | Not provided | The split backends cover PID-tuning phases; a faithful coupled facade was not completed. |
| Attitude | Quaternion controller extracted from `AC_AttitudeControl` | Produces FRD body-rate targets. |
| Rate | Three upstream `AC_PID` objects | `landed` clears integrators; `dt_sec`, filters, D/FF, slew max and slew tau are honored. |
| Allocation | `AP_MotorsMatrix` normalization/desaturation extraction | Fixed coplanar downward rotors, at most 12; public thrust is normalized by hover. |
| Feedback | Unallocated torque sign to per-axis saturation | Caller supplies the previous allocation status, giving the required one-cycle delay. |
| EKF | Upstream `NavEKF3` through DAL | One IMU/GPS/baro/compass; reset recreates the frontend and core. |

`set_armed_status`, `set_in_air_status`, and `set_vehicle_at_rest` are stored
independently. Drone PRO currently never calls `set_armed_status`; consequently
EKF3 observes the default disarmed state even after takeoff indication, which
can keep armed-only EKF behavior disabled. Integration must explicitly forward
the vehicle arming state.

The allocation output omits PWM conversion, thrust expo, spool state, battery
compensation, and lost-motor boost. The plant/caller converts normalized rotor
thrust to its actuator command.

Full `AC_PosControl` is not instantiated because its constructor requires the
vehicle AHRS, inertial navigation, motors, attitude controller, scheduler, and
parameter store. The controller primitives are linked directly; only the
vehicle-dependent stage orchestration is extracted.
