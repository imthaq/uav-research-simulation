# Dynamic Trust Model

**Trust calculation:** Each sensor gets a trust score based on how well it agrees with other sensors, how fresh its data is, how often it drops out, and its confidence/covariance quality.

**Trust update rule:** Every step, these signals are averaged into a "target" trust value, and the sensor's trust moves toward that target.

**Trust decay:** Trust drops quickly when a sensor disagrees with others, goes stale, drops out often, or looks like a false alarm.

**Trust recovery:** Trust rises again once the sensor is consistent, fresh, and confident - but only gradually, not instantly.

**Limitations:** There's no real ground truth or false-alarm label to check against, so a group of sensors agreeing on the same wrong answer can still look "trustworthy," and trust resets to default at the start of every run.