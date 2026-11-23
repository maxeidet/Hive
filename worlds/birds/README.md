# Birds

Bird meshes for distractors and training images (`swarm_backend/sim/birds.py`, `./mount-backend.sh bird`). They are loaded from `/workspace/worlds/birds/...`, the path where the start scripts mount the repo inside the container, so no Gazebo model folders are needed.

| File | Model | Author | Licence | Notes |
|---|---|---|---|---|
| `dove.glb` | [Dove Bird Rigged](https://sketchfab.com/3d-models/dove-bird-rigged-6b91be2a28fb4404a2d57d5ca98bd4dc) | [FourthGreen](https://sketchfab.com/FourthGreen) | [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/) | Textured, rigged, one animation ("Take 001") |
| `mourning_dove.glb` | [Mourning Dove - in Flight](https://sketchfab.com/3d-models/mourning-dove-in-flight-37f9b2cc8b174fc3a2d8ba9f5b467b1a) | [osuecampus](https://sketchfab.com/osuecampus) | [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/) | Textured, wings-flapping animation |
| `seagull.glb` | [Flying seagull](https://poly.pizza/m/6Tpj_vcWP3f) | Poly by Google | [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) | Low poly, wings raised; added 18 Oct |
| `gull.glb` | [Flying gull](https://poly.pizza/m/eMNhHDZakYp) | Poly by Google | [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) | Same shape as the seagull; added 18 Oct |
| `eagle.glb` | [Eagle](https://poly.pizza/m/1Z5L1v0bfu7) | Robert Mirabelle | [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) | Gliding, wings spread flat; added 18 Oct |
| `soaring_bird.glb` | [Bird](https://poly.pizza/m/8Ph79kHbt9s) | Poly by Google | [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) | Soaring, fingered wing tips, modelled z-up; added 18 Oct |
| `flock.glb` | [Birds](https://sketchfab.com/3d-models/birds-3a9bb97be78944f9bffc23fb25c2154e) | [Zacxophone](https://sketchfab.com/Zacxophone) | [Sketchfab Standard](https://sketchfab.com/licenses) | Several untextured birds in one model, animated |

CC BY 3.0 and 4.0 require the credit above wherever the models (or images made with them) are shared, e.g. in the report. The Sketchfab Standard licence allows using the model in our project but not redistributing the file itself, so `flock.glb` must stay out of anything public (the repo is private).

Gazebo shows them without their animation (a still pose). The data generators use only the ones in a flying pose (`FLYING` in `birds.py`: dove, seagull, gull, eagle, soaring_bird); the mourning dove renders as a thin sliver and the flock is one model of five birds. A perched crow from Poly Pizza was looked at on 18 Oct and left out. Sizes in the files differ (one is in centimetres); `birds.py` scales each to a realistic wingspan, see `KINDS` there.
