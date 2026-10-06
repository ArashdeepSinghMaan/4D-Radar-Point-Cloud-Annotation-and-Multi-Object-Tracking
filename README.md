# 4D Radar Point-Cloud Annotation and Multi-Object Tracking

A perception-engineering project for **automotive radar point-cloud annotation, radar object detection, Doppler-based motion estimation, and multi-object tracking**.

The project focuses on understanding Radar as a first-class perception sensor rather than treating it only as an additional input to a camera or LiDAR system.

---

## 1. Project Overview

Automotive Radar provides information that is fundamentally different from camera and LiDAR.

A camera provides:

- Appearance
- Semantic information
- Texture
- 2D localization

LiDAR provides:

- Accurate 3D geometry
- Depth
- Spatial structure

Radar provides:

- Range
- Azimuth
- Elevation
- Radial velocity
- Doppler information
- Radar cross section (RCS)
- Robust measurements under challenging visibility conditions

This project develops a complete Radar perception pipeline:

```text
Radar Raw Measurements
        |
        v
Point-Cloud Generation
        |
        v
Filtering
        |
        v
Clustering
        |
        v
Radar Object Candidates
        |
        v
Object Annotation
        |
        v
Object-Level Features
        |
        v
Data Association
        |
        v
Multi-Object Tracking
        |
        v
Velocity Estimation
        |
        v
Evaluation
```

The final system should be capable of maintaining persistent tracks for:

- Cars
- Trucks
- Buses
- Motorcycles
- Bicycles
- Pedestrians
- Static objects

---

# 2. Objectives

The main objectives are:

- Understand automotive Radar measurements
- Build a Radar point-cloud visualization pipeline
- Develop a custom Radar annotation workflow
- Define a structured Radar annotation format
- Implement point-cloud filtering
- Implement Radar point clustering
- Extract Radar object-level features
- Estimate static/dynamic state
- Track multiple Radar objects
- Estimate object velocity
- Handle missed detections
- Evaluate tracking performance
- Prepare the system for Radar-camera and Radar-LiDAR fusion

---

# 3. Radar Measurement Model

A Radar point may contain:

```text
x
y
z
range
azimuth
elevation
radial_velocity
RCS
SNR
timestamp
```

Not every Radar sensor exposes every field.

A typical Radar point can be represented as:

```text
RadarPoint
{
    x,
    y,
    z,
    range,
    azimuth,
    elevation,
    radial_velocity,
    rcs,
    snr,
    timestamp
}
```

---

# 4. Radar Coordinate System

A typical coordinate system is:

```text
                 Y
                 ^
                 |
                 |
                 |
Radar ----------+--------------> X
                |
                |
                v
                Z
```

The exact convention depends on the sensor and ROS TF configuration.

All Radar measurements should eventually be represented in a consistent ROS coordinate frame.

Example:

```text
radar_link
    |
    +---- radar_front
```

---

# 5. Range and Angle Representation

For a Radar point:

\[
r = \sqrt{x^2+y^2+z^2}
\]

Azimuth:

\[
\theta = \tan^{-1}\left(\frac{y}{x}\right)
\]

Elevation:

\[
\phi =
\tan^{-1}
\left(
\frac{z}{\sqrt{x^2+y^2}}
\right)
\]

Cartesian coordinates can be reconstructed from spherical measurements:

\[
x=r\cos(\phi)\cos(\theta)
\]

\[
y=r\cos(\phi)\sin(\theta)
\]

\[
z=r\sin(\phi)
\]

---

# 6. Doppler Velocity

Radar measures velocity along the line of sight.

The radial velocity is:

\[
v_r = v\cos(\theta)
\]

where:

- \(v_r\) = measured radial velocity
- \(v\) = object's true velocity
- \(\theta\) = angle between the object's velocity and Radar line of sight

Therefore:

```text
Object moving directly toward Radar
        |
        | maximum Doppler
        v

Object moving perpendicular to Radar
        |
        | approximately zero radial velocity
```

This makes Radar especially useful for detecting motion.

---

# 7. Radar Cross Section

Radar Cross Section (RCS) represents the apparent reflectivity of an object to Radar.

Different objects can produce different RCS characteristics.

Potential features:

```text
mean RCS
maximum RCS
minimum RCS
RCS variance
RCS distribution
```

RCS can be used as an additional feature for:

- Object classification
- Object association
- Track confidence

---

# 8. Point-Cloud Annotation

The project will include a dedicated annotation pipeline.

The annotation tool should support:

```text
3D point-cloud visualization
Object selection
3D bounding boxes
Object classes
Object IDs
Velocity
RCS
Radar confidence
Temporal navigation
Frame navigation
Annotation export
```

Example annotation:

```yaml
object:
  id: 17
  class: car

  position:
    x: 24.3
    y: -2.1
    z: 0.7

  dimensions:
    length: 4.4
    width: 1.8
    height: 1.6

  yaw: 1.57

  velocity:
    vx: -3.2
    vy: 0.1

  radar:
    mean_rcs: 11.2
    mean_radial_velocity: -3.1
    point_count: 18
```

---

# 9. Annotation Classes

Initial classes:

```text
Car
Truck
Bus
Motorcycle
Bicycle
Pedestrian
Unknown
StaticObject
```

The class list should remain configurable.

---

# 10. Annotation Levels

The dataset should support multiple levels of annotation.

### Point-level

```text
point_id
class
object_id
```

### Object-level

```text
object_id
class
3D bounding box
position
orientation
velocity
```

### Frame-level

```text
timestamp
sensor pose
number of objects
weather
scene type
```

---

# 11. Temporal Annotation

Radar is inherently temporal.

Therefore, annotations should preserve identity:

```text
Frame 001
    Car → ID 12

Frame 002
    Car → ID 12

Frame 003
    Car → ID 12

Frame 004
    Car → ID 12
```

This allows the annotated dataset to be used for tracking research rather than only object detection.

---

# 12. Radar Preprocessing

Pipeline:

```text
Raw Radar
    |
    v
Range filtering
    |
    v
Velocity filtering
    |
    v
RCS filtering
    |
    v
Outlier removal
    |
    v
Clustering
```

Example filtering:

```text
Range:
0.5 m → 100 m

Radial velocity:
-40 m/s → +40 m/s

RCS:
sensor-dependent threshold
```

These values should be configurable.

---

# 13. Clustering

The initial clustering method will use DBSCAN.

Feature vector:

\[
P_i=[x_i,y_i,z_i,v_{r,i}]
\]

This allows clustering to consider both:

- Spatial proximity
- Doppler similarity

Example:

```text
Radar points

    • • •
  • • • • •
    • • •

        •
        •
        •

       • • •


      ↓ DBSCAN


Cluster 1       Cluster 2
  Vehicle        Pedestrian
```

---

# 14. DBSCAN

DBSCAN requires:

```text
eps
min_samples
```

The algorithm groups points based on density.

Advantages:

- Does not require number of objects in advance
- Handles noise
- Suitable for sparse Radar measurements

---

# 15. Radar Object Representation

Each cluster becomes an object candidate.

```cpp
struct RadarObject
{
    int id;

    int class_id;

    Eigen::Vector3d position;
    Eigen::Vector3d dimensions;

    double yaw;

    double radial_velocity;

    double mean_rcs;
    double mean_snr;

    int point_count;

    double confidence;

    rclcpp::Time timestamp;
};
```

---

# 16. Static vs Dynamic Classification

A Radar object can initially be classified using Doppler.

```text
Radar Object
     |
     v
Radial Velocity
     |
     +----------------+
     |                |
 near zero         significant
     |                |
     v                v
 STATIC            DYNAMIC
```

However, ego motion must be considered.

A stationary roadside object may show non-zero radial velocity when the vehicle is moving.

Therefore, static/dynamic classification should eventually use:

```text
Radar Doppler
+
Ego velocity
+
Object position
```

---

# 17. Ego-Motion Compensation

Observed Radar velocity contains both:

```text
Object motion
+
Ego motion
```

Therefore:

\[
v_{relative}
=
v_{object}-v_{ego}
\]

The system should obtain ego motion from:

- IMU
- Wheel odometry
- LiDAR odometry
- ROS TF

---

# 18. Multi-Object Tracking

The initial tracker will use a Kalman Filter.

State:

\[
x =
[x,y,v_x,v_y]^T
\]

Motion model:

\[
x_k=Fx_{k-1}+w_k
\]

with:

\[
F=
\begin{bmatrix}
1&0&\Delta t&0\\
0&1&0&\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}
\]

---

# 19. Measurement Model

Radar provides:

```text
x
y
radial velocity
```

The measurement model may therefore be:

\[
z=Hx+v
\]

The measurement model should be adapted depending on which Radar fields are available.

---

# 20. Data Association

For every predicted track:

```text
Predicted Track
       |
       v
Candidate Radar Objects
       |
       v
Cost Matrix
       |
       v
Hungarian Algorithm
       |
       v
Track ↔ Detection
```

Cost can include:

\[
C =
w_pC_{position}
+
w_vC_{velocity}
+
w_rC_{RCS}
\]

---

# 21. Track Lifecycle

```text
NEW
 |
 v
TENTATIVE
 |
 | enough observations
 v
CONFIRMED
 |
 | missed detections
 v
LOST
 |
 | timeout
 v
DELETED
```

Parameters:

```yaml
tracker:
  min_hits: 3
  max_age: 5
  max_match_distance: 3.0
```

---

# 22. ROS 2 Architecture

```text
radar_driver
      |
      v
/radar/points
      |
      v
radar_preprocessor
      |
      v
/radar/filtered_points
      |
      v
radar_clusterer
      |
      v
/radar/objects
      |
      v
radar_tracker
      |
      v
/radar/tracked_objects
```

---

# 23. Proposed ROS 2 Topics

```text
/radar/points
/radar/filtered_points
/radar/clusters
/radar/objects
/radar/tracked_objects
/radar/markers
/tf
/tf_static
```

---

# 24. Annotation Dataset Format

Recommended directory:

```text
dataset/
├── sequences/
│   ├── sequence_001/
│   │   ├── radar/
│   │   ├── timestamps.txt
│   │   └── annotations/
│   │
│   └── sequence_002/
│
├── calibration/
│   └── radar.yaml
│
└── metadata/
    └── scenes.json
```

---

# 25. Evaluation Metrics

## Detection

```text
Precision
Recall
F1
3D IoU
```

## Tracking

```text
MOTA
MOTP
IDF1
HOTA
ID Switches
Fragmentation
```

## Radar-specific

```text
Range MAE
Velocity MAE
Radial velocity error
Static/dynamic accuracy
```

---

# 26. Performance Metrics

Measure:

```text
Radar processing FPS
Clustering latency
Tracking latency
End-to-end latency
CPU utilization
Memory usage
```

Target:

```text
>20 FPS
<50 ms end-to-end latency
```

The final values must be experimentally measured.

---

# 27. Development Roadmap

## Phase 1 — Radar Understanding

```text
[ ] Understand Radar measurements
[ ] Parse Radar data
[ ] Visualize point clouds
[ ] Study Doppler
[ ] Study RCS
```

## Phase 2 — Annotation

```text
[ ] Build annotation interface
[ ] Implement 3D bounding boxes
[ ] Add object IDs
[ ] Add class labels
[ ] Add velocity labels
[ ] Export annotations
```

## Phase 3 — Detection

```text
[ ] Filtering
[ ] DBSCAN
[ ] Object extraction
[ ] Static/dynamic classification
```

## Phase 4 — Tracking

```text
[ ] Kalman Filter
[ ] Data association
[ ] Track lifecycle
[ ] Velocity estimation
[ ] Ego-motion compensation
```

## Phase 5 — Evaluation

```text
[ ] Generate ground truth
[ ] Calculate detection metrics
[ ] Calculate tracking metrics
[ ] Benchmark latency
```

## Phase 6 — Sensor Fusion

```text
[ ] Radar + Camera
[ ] Radar + LiDAR
[ ] Radar + Camera + LiDAR
```

---

# 28. Future Extensions

- Radar-camera fusion
- Radar-LiDAR fusion
- Radar-camera-LiDAR fusion
- Learned Radar object detection
- Radar BEV representation
- Transformer-based Radar perception
- Radar trajectory prediction
- Occupancy estimation
- Radar-based collision prediction

---

# 29. Expected Skills

This project demonstrates:

- Radar perception
- Point-cloud processing
- Doppler interpretation
- RCS analysis
- 3D annotation
- DBSCAN
- Kalman filtering
- Multi-object tracking
- Data association
- Ego-motion compensation
- ROS2
- Dataset engineering
- Perception evaluation

---

# 30. Definition of Done

```text
[ ] Radar point-cloud visualization
[ ] Custom annotation tool
[ ] Structured Radar dataset
[ ] 3D object annotations
[ ] Doppler-aware clustering
[ ] Static/dynamic classification
[ ] Multi-object tracking
[ ] Persistent track IDs
[ ] Velocity estimation
[ ] Ego-motion compensation
[ ] ROS2 integration
[ ] Quantitative evaluation
[ ] Benchmarking
[ ] Radar-camera fusion prototype
```

---

# 31. Portfolio Outcome

The final project should demonstrate that the developer can work from:

```text
Raw Radar
   ↓
Point Cloud
   ↓
Annotation
   ↓
Filtering
   ↓
Clustering
   ↓
Object Detection
   ↓
Doppler Analysis
   ↓
Tracking
   ↓
Ego Motion
   ↓
Sensor Fusion
```

This project is intended to establish **Radar perception and tracking** as a core engineering skill.
