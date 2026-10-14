# Birds

Bird meshes for distractors and training images (`swarm_backend/birds.py`, `./mount-backend.sh bird`). They are loaded from `/workspace/worlds/birds/...`, the path where the start scripts mount the repo inside the container, so no Gazebo model folders are needed.

| File | Model | Author | Licence | Notes |
|---|---|---|---|---|
| `dove.glb` | [Dove Bird Rigged](https://sketchfab.com/3d-models/dove-bird-rigged-6b91be2a28fb4404a2d57d5ca98bd4dc) | [FourthGreen](https://sketchfab.com/FourthGreen) | [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/) | Textured, rigged, one animation ("Take 001") |
| `mourning_dove.glb` | [Mourning Dove - in Flight](https://sketchfab.com/3d-models/mourning-dove-in-flight-37f9b2cc8b174fc3a2d8ba9f5b467b1a) | [osuecampus](https://sketchfab.com/osuecampus) | [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/) | Textured, wings-flapping animation |
| `flock.glb` | [Birds](https://sketchfab.com/3d-models/birds-3a9bb97be78944f9bffc23fb25c2154e) | [Zacxophone](https://sketchfab.com/Zacxophone) | [Sketchfab Standard](https://sketchfab.com/licenses) | Several untextured birds in one model, animated |

CC BY 4.0 requires the credit above wherever the models (or images made with them) are shared, e.g. in the report. The Sketchfab Standard licence allows using the model in our project but not redistributing the file itself, so `flock.glb` must stay out of anything public (the repo is private).

Gazebo shows them without their animation (a still pose). Sizes in the files differ (one is in centimetres); `birds.py` scales each to a realistic wingspan, see `KINDS` there.
