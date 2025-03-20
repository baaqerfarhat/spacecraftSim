# Environment Configuration — Particles

Space Robotics Bench can simulate liquid and granular materials like regolith, sand, and dust using particle-based physics. This is particularly relevant for space applications where interaction with loose granular material is common.

## Enabling Particles

You can enable particles in any environment by setting the `_particles` parameter to `true`:

```bash
srb agent teleop -e _manipulation env._particles=true env.robot=+scoop
```

## Particle Configuration Parameters

| Parameter              | Description                 | Default |
| ---------------------- | --------------------------- | ------- |
| `env._particles`       | Enable/disable particles    | `false` |
| `env._particles_size`  | Particle diameter (meters)  | `0.025` |
| `env._particles_ratio` | Particle density multiplier | `0.001` |

```bash
srb agent teleop -e _manipulation env._particles=true env._particles_size=0.01 env._particles_ratio=0.1
```

## Particle Behavior

By default, the particle system uses a pyramid distribution to create natural-looking piles of granular material with higher density at the center. Particles interact with other objects through physical collision and settle over time due to gravity. Robots can push, scoop, or otherwise interact with particles.

> **Note:** When particles are enabled, Fabric is disabled via `env.sim.use_fabric=false`.
