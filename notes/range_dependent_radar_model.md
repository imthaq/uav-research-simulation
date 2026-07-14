# Range Dependent Radar Model

### - how P_D changes with distance
- If the target is very close then P_D is very high, else if it's near the edge of the radar range then it is low but still present, else depending on the value of SNR it is varies.
### - how SNR changes with distance
- The value of SNR depends on the distance after it bounces back of the target. If the range is very high then signal gets weaker as the time progress and by adding other factors such as background noise we get the actual value.
### - how measurement noise changes with distance
-  Even if at long distance the radar detects something the signal is very weak and less precise, so it is more harder to pin down the actual location of the target. Thus the variance in the bearing , range and  radial velocity increase as the SNR decreases.
### - why range-dependent modeling is more realistic
-  Range-dependent modeling is more realistic because real sensors get weaker and noisier the farther away a target is a fixed noise value ignores that and treats near and far detections as equally trustworthy.