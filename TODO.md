Below is a list of important tasks that are broadly relevant to this
project. Additional tasks may also be described directly in the source code
using comment blocks marked with "TODO".

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
