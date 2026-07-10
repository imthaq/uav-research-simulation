# Target Detection using Radar models

## Introduction: 
This file will discuss about how different UAV drones rely on the measurement of the radar sensor models to reach its target rather than knowing the exact location of the target. However, uncertainty in measurement of radar also effects the decision made by the UAV, moreover it increases the chances of mission failure and formation break down as well.

## Radar like sensor model

### - What is ground truth?
- The concept of “ground truth” does not apply only to mapping. It is an umbrella term that refers to a “fundamental truth”  a value that has been directly observed, measured, and proven to be true.
### - Where ground truth is used ? 
- It is mostly used when we are trying to train our ML models or UAVs on certain algorithm such that we are able to determine its behavior. For each test-case we run we know the outcome of our results, however in real life this is not often the case and results can vary when we consider different factors such as uncertainty in radar measurement , environmental factors , different perception errors.


## Radar measurement generation and uncertainty injection.

### - Why radar measurements are often incorrect ? 
- Most of the measurement generated from radar models often contain measurement error,  the reason can be due to different factors which often includes  noise, which can distort the signals being measured. Additionally, factors like systematic measurement errors and the inherent limitations of the radar technology contribute to inaccuracies in the data collected. Moreover the quality and model of radar can also vary the results.


## UAV  decision


### - How UAV  finds its target position ?
- Conventional UAV mostly rely on predetermined path to reach its goal which ends up being energy inefficient because of the unnecessary path it takes to reach its target and sometimes it missies the target because of lack of localization. Advance UAV integrates the radar sensor data to reach it’s target. UAVs find their targets using radar by transmitting electromagnetic waves that bounce off objects, allowing the system to detect and identify the target based on the reflected signals . However due  to lack of precision it often ends up getting collided into obstacles and causes formation errors as well.	 


## Swarm Metrics 

### -  How does swarm metrics measure the effects of radar uncertainty ? 
- Swarm metrics measure the effects of radar uncertainty by incorporating point uncertainty in radar SLAM systems, which enhances the accuracy and robustness of radar-based navigation. This approach models the uncertainty of radar points and uses probabilistic data association to improve performance in challenging conditions.


