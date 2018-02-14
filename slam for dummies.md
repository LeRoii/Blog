## About SLAM
Simultaneous Localization And Mapping
is concerned with the problem of building a map of an unknown environment by a mobile robot while at the same time navigating the environment using the map.

consists of:

* Landmark extraction
* Data association
* State estimation
* State update 
* Landmark update


The goal of is to use the environment to update the position of the robot. 


* SLAM can be implemented in many ways using vari hardware&software.
* SLAM is more like a concept than a single algorithm. 



An EKF (Extended Kalman Filter) is the heart of the SLAM process. It is responsible for updating where the robot thinks it is based on the features extracted from the environment. 

The EKF keeps track of an estimate of the uncertainty in the robots position and also the uncertainty in these landmarks it has seen in the environment.

### Landmark Extraction

Two landmark extraction algorithm

* Spikes
* RANSAC

suitable for indoor environments. 

#### Spikes

The spike landmark extraction uses extrema to find landmarks. They are identified by finding values in the range of a laser scan where two values differ by more than a certain amount, e.g. 0.5 meters. This will find big changes in the laser scan from e.g. when some of the laser scanner beams reflect from a wall and some of the laser scanner beams do not hit this wall, but are reflected from some things further behind the wall.


Spike landmarks rely on the landscape changing a lot between two laser beams. This means that the algorithm will fail in smooth environments.

simple and is not robust against environments with people. The reason for this is that Spikes picks up people as spikes as they theoretically are good landmarks (they stand out from the environment).

#### RANSAC

RANSAC (Random Sampling Consensus) is a method which can be used to extract lines from a laser scan. These lines can in turn be used as landmarks. In indoor environments straight lines are often observed by laser scans as these are characteristic of straight walls which usually are common.
RANSAC finds these line landmarks by randomly taking a sample of the laser readings and then using a least squares approximation to find the best fit line that runs through these readings. Once this is done RANSAC checks how many laser readings lie close to this best fit line. If the number is above some threshold we can safely assume that we have seen a line (and thus seen a wall segment). This threshold is called the consensus.

##### Least squares approximation 


#### Scan-matching


### Data Association

The problem of data association is that of matching observed landmarks from different (laser) scans with each other. We have also referred to this as re-observing landmarks.

In practice the following problems can arise in data association:
- You might not re-observe landmarks every time step.
- You might observe something as being a landmark but fail to ever see it again.
- You might wrongly associate a landmark to a previously seen landmark.

As stated in the landmarks chapter it should be easy to re-observe landmarks. As such the first two cases above are not acceptable for a landmark. In other words they are bad landmarks. Even if you have a very good landmark extraction algorithm you may run into these so it is best to define a suitable data-association policy to minimize this.

#### Data association policy

##### Nearest-neighbor approach
associate a landmark with the nearest landmark in the database.

to calculate the nearest landmark, use
Euclidean distance(simple)
Mahalanobis distance(better but complicated)

1. For a new extracted landmark, associate it to the closest landmark we have seen more than N times in the database.
2. Pass the pair of association(extracted landmark, landmark in database) through a validation gate.
a. If the pair passes the validation gate it must be the same landmark we have re-observed so increment the number of times we have seen it in the database.
b. If the pair fails the validation gate add this landmark as a new landmark in the database and set the number of times we have seen it to 1.

The validation gate uses the fact that our EKF implementation gives a bound on the uncertainty of an observation of a landmark. Thus we can determine if an observed landmark is a landmark in the database by checking if the landmark lies within the area of uncertainty. This area can actually be drawn graphically and is known as an error ellipse.


#### Innovation

is basically the difference between the estimated robot position and the actual robot position, based on what the robot is able to see. 




























