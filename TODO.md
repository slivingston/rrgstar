* **libbot extensions**: visualization of trajectories: The viewer in the libbot
  extensions does not visualize the trajectories published by the planners.

* **Collision checker bounding box**: /components/collision_checkers/standard,
  along with obstacles, should also take in a bounding box as a parameter and
  ensure that the trajectories lie inside the bounding box. Currently it only
  ensures that the samples lie in a bounding box (by the sampler), but the
  trajectories connecting the samples may go out of this bounding box.

* **Exception handling**: Not done at all.

* Add in an example with multiple inheritance to create two components (and
  mention it in the documentation).

* Add LICENSE.  The original code by Sertac Karaman and Emilio Frazzoli is under
  the [MIT license](http://opensource.org/licenses/MIT).
