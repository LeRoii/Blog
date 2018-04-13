### Introduction
Most openly available frameworks for visual and visualinertial
SLAM either focus on a single-session case or
only provide large-scale batch optimization without an online
frontend. Usually, they are crafted for a very specific
pipeline without a separation between the map structure and algorithms.They often lack completeness and will not offer a
full workflow such that a map can be created, manipulated,
merged with previous sessions and reused in the frontend
within a single framework. This impairs the flexibility of
such systems, a key for rapid development and research.

maplab does not only provide tools to create and localize from
visual-inertial maps but also provides map maintenance and
processing capabilities

These tools involve multi-session merging, sparsification, loop closing, dense reconstruction and visualization of maps

deployed on a variety of robotic platforms including<br>
microaerial vehicles, <br>
autonomous planes, <br>
autonomous cars, <br>
autonomous underwater vehicles, <br>
walking robots. <br>
It has also served as a research platform for map summarization, map quality evaluation, multi-session 3d reconstruction, topological mapping,visual localization, and decentralized mapping.


### Related Work
PTAM(AR)<br>
OKVIS, a visual-inertial keyframe-based estimator<br>
Svo: Fast semi-direct monocular visual odometry<br>
LSD-SLAM: Large-scale direct monocular slam<br>

None of these methods supports global localization against a previously recorded map

ORB_SLAM or ORB_SLAM2, closely related to maplab<br>
However, maplab offers an offline processing toolkit centered around a console user interface, which guarantees high flexibility and permits users to add their own extensions or modify the processing pipelines.

The ability to merge multiple mapping sessions into a single, consistent
map and to refine it using a visual-inertial least-squares optimization

Online frontend, ROVIOLI, using image intensity within patches instead of point features guarantees a high level of robustness, even in the presence of motion blur

maplab stores a unified localization map allowing to use a carefully selected
subset of features, e.g. based on the current appearance conditions

### The Maplab framework
Two major parts<br>

* The online VIO and localization frontend, ROVIOLI,
that takes raw visual-inertial sensor data. It outputs (global) pose estimates and can be used to build visualinertial maps.

* The (offline) maplab-console that lets the user apply various algorithms on maps in an offline batch fashion. It does also serve as a research testbed for new algorithms that operate on visual-inertial data

#### Workflow in maplab
* In VIO mode, ROVIOLI estimates the pose of an agent w.r.t. a (drifting) local frame; additionally a map is built based on these estimates
* Resulting maps can be loaded in the maplab-console where all of the available
algorithms can be applied, e.g. map alignment and merging, VI optimization, loop-closure.
* In LOC mode, ROVIOLI can load the updated map to track a global (drift-free) pose online.

#### Maplab console
Manipulate maps using various algorithms, combined with RViz, facilitates algorithm prototyping and parameter tuning

#### Map structure
The framework uses a data structure, called VI-map, for visual-inertial mapping data

* Each map may contain multiple missions where each is based on a single recording session.
* The core structure of a mission is a graph consisting of vertices and edges
* A vertex corresponds to a state captured at a certain point in time
It contains a state estimate (pose, IMU biases, velocity) and visual information from the (multi-)camera system including keypoints, descriptors,tracking information and images
* An edge connects two neighboring vertices. While there are a few different types of edges in maplab
* The most common type is the IMU edge. It contains the inertial measurements recorded between the vertices
* Visual observations tracked by multiple vertices are triangulated as 3d landmarks. The landmark itself is stored within the vertex that first observed it

#### Core packages of maplab
##### VIWLS
Visual-inertial weighted least-squares optimization
with cost terms similar to OKVIS, by default, the optimization problem is constructed using visual and inertial data, but optionally it can include wheel odometry, GPS measurements or other
types of pose priors
##### Loop closure/localization
A complete loop closure and localization system based on binary descriptors, 
the search backend uses an inverted multi-index for efficient nearest
neighbor retrieval on projected binary descriptors
##### ROVIOLI
online visual-inertial mapping and localization frontend
##### Posegraph relaxation
posegraph optimization using edges introduced by the loop closure system
##### aslam cv2
a collection of computer vision data structures and algorithms. It includes various camera and distortion models as well as algorithms for feature detection, extraction, tracking and geometric vision.
##### Map sparsification
algorithms to select the best landmarks for localization and keyframe selection to sparsify the pose graph
##### Dense reconstruction
a collection of dense reconstruction,depth fusion and surface reconstruction algorithms

#### ROVIOLI(ROVIO with Localization Integration)
build maps from raw visual and inertial data and also localize w.r.t. existing maps online<br>
Two modes available:<br>
##### VIO mode
build a map based on the VIO estimates
##### LOC mode
additionally localization constraints are processed to track a (drift-free) global pose estimate w.r.t. a given map

### Use cases
online mapping and localization,<br>
multi-session mapping, <br>
map maintenance, <br>
large-scale mapping<br>
dense reconstruction.<br>