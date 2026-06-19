# Example support crates

Our own `imu` (MPU-6050-class driver) and `attitude` (Mahony complementary filter) crates, kept here
in-repo so the `imu_tilt` example builds standalone with no cross-repo path dependency and the
runtime-hal repo pushes cleanly. They are `no_std`, fixed-point, and generic over `embedded-hal`
(the driver takes any `embedded-hal` 1.0 I2C), so they are not specific to runtime-hal.

These are NOT vendored third-party code: they are ours, parked here for now. When the wider project
firms up they will move to their own repo (and likely crates.io), at which point this folder becomes
a normal git/version dependency. They currently also live in `hoverboard-firmware/crates/{imu,attitude}`;
keep the two in sync until the split happens.
