# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.2] - 2025-03-20

### Added
- Add git-cliff configuration by @AndrejOrsula
- Add templates for mobile manipulation tasks by @AndrejOrsula
- Docker: Add option to ensure Docker and NVIDIA toolkit are installed by @AndrejOrsula
- Devcontainer: Add default extensions by @AndrejOrsula

### Changed
- Bump to 0.0.2 by @AndrejOrsula
- Docs: Update towards 0.0.2 by @AndrejOrsula
- Bump MSRV to 1.84 by @AndrejOrsula
- Update dependencies (Python & Rust) by @AndrejOrsula
- Deny: Ignore RUSTSEC-2024-0436 by @AndrejOrsula
- Docker: Update commits of dev dependencies by @AndrejOrsula
- Tests: Update to match CLI changes by @AndrejOrsula
- CLI: Streamline usage for readable documentation by @AndrejOrsula
- Locomotion_velocity_tracking: Adjust reward by @AndrejOrsula
- Mobile_debris_capture: Use default robot of the base environment class by @AndrejOrsula
- Orbital_evasion: Update observation and reward by @AndrejOrsula
- Excavation: Default to Spot robot mobile base by @AndrejOrsula
- Refactor environment classes to improve naming consistency across manipulation tasks by @AndrejOrsula
- Simplify base parameter naming in environment config by @AndrejOrsula
- CLI: Improve command-line overrides for environment config by @AndrejOrsula
- Improve asset configuration consistency by @AndrejOrsula
- Docker: Update the default development volumes by @AndrejOrsula
- Docker: Ensure assets are initialized when building the image by @AndrejOrsula
- Update installation scripts by @AndrejOrsula
- CI: Checkout submodules recursively to build Docker with assets by @AndrejOrsula
- Build(deps): bump typed-builder from 0.20.0 to 0.20.1 by @dependabot[bot] in [#52](https://github.com/AndrejOrsula/space_robotics_bench/pull/52)
- Build(deps): bump egui_extras from 0.31.0 to 0.31.1 by @dependabot[bot] in [#51](https://github.com/AndrejOrsula/space_robotics_bench/pull/51)
- Build(deps): bump eframe from 0.31.0 to 0.31.1 by @dependabot[bot] in [#48](https://github.com/AndrejOrsula/space_robotics_bench/pull/48)
- Build(deps): bump serde from 1.0.218 to 1.0.219 by @dependabot[bot] in [#49](https://github.com/AndrejOrsula/space_robotics_bench/pull/49)

### Fixed
- Correct action term/group naming by @AndrejOrsula

### Removed
- Remove redundant event by @AndrejOrsula

## [0.0.1] - 2025-03-04

### Added
- Add barebones mobile_debris_capture task by @AndrejOrsula
- Add barebones excavation task by @AndrejOrsula
- Add orbital_evasion task by @AndrejOrsula
- Add locomotion_velocity_tracking task by @AndrejOrsula
- Add solar_panel_assembly task by @AndrejOrsula
- Add peg_in_hole_assembly task by @AndrejOrsula
- Add sample_collection task by @AndrejOrsula
- Add debris_capture task by @AndrejOrsula
- Add basic tests by @AndrejOrsula
- Add unified entrypoint script by @AndrejOrsula
- Add config and hyparparam utils by @AndrejOrsula
- Add mobile manipulation base envs by @AndrejOrsula
- Add mobile manipulation base envs by @AndrejOrsula
- Add manipulation base env and task template by @AndrejOrsula
- Add ROS 2 interface by @AndrejOrsula
- Add skrl integration by @AndrejOrsula
- Add SB3 and SBX integrations by @AndrejOrsula
- Add Dreamer integration by @AndrejOrsula
- Add shape and ground plane assets by @AndrejOrsula
- Add object/scenery assets from srb_assets by @AndrejOrsula
- Add initial procedural SimForge assets by @AndrejOrsula
- Add initial tools (end-effectors) by @AndrejOrsula
- Add initial robot assets by @AndrejOrsula
- Add custom Franka arm and FrankaHand tool (separate) by @AndrejOrsula
- Add AnyEnv/AnyEnvCfg type aliases by @AndrejOrsula
- Add GUI interface by @AndrejOrsula
- Add teleop interfaces by @AndrejOrsula
- Add visual environment extension by @AndrejOrsula
- Add common environment base classes by @AndrejOrsula
- Add oxidasim sampling utils by @AndrejOrsula
- Add mobile robot action terms/groups by @AndrejOrsula
- Add task space manipulation action terms/groups by @AndrejOrsula
- Add ParticleSpawner by @AndrejOrsula
- Add common action terms and groups by @AndrejOrsula
- Add custom events by @AndrejOrsula
- Add custom visualization markers by @AndrejOrsula
- Add custom RobotAssembler by @AndrejOrsula
- Add extra shape spawners by @AndrejOrsula
- Add Domain enum with utils by @AndrejOrsula
- Add config for RTX visuals and post-processing by @AndrejOrsula
- Add logging and tracing utils by @AndrejOrsula
- Add common utils by @AndrejOrsula
- Docs: Add button for suggesting edits by @AndrejOrsula
- Docs: Add link to Discord by @AndrejOrsula
- Docker: Add option to install Space ROS by @AndrejOrsula
- CLI: Add short args and update environ for extension module update by @AndrejOrsula

### Changed
- CI: Build Python package with uv by @AndrejOrsula
- CI: Disable llvm-cov in Rust workflow by @AndrejOrsula
- Pre-commit: Downgrade mdformat by @AndrejOrsula
- GUI: Replace missing image by @AndrejOrsula
- CI: Update Python/Rust workflows by @AndrejOrsula
- Update badges in README by @AndrejOrsula
- Patch ActionManager to improve compatibility with ActionGroup by @AndrejOrsula
- Wrap around Isaac Lab core modules by @AndrejOrsula
- Define ActionGroup model by @AndrejOrsula
- Define the full asset hierarchy model by @AndrejOrsula
- Integrate uv by @AndrejOrsula
- Update Docker setup by @AndrejOrsula
- Integrate SimForge by @AndrejOrsula
- Update srb_assets by @AndrejOrsula
- Update pre-commit hooks by @AndrejOrsula
- Update copyright year to 2025 by @AndrejOrsula
- Update to Isaac Sim 4.5 by @AndrejOrsula
- Update module name from space_robotics_bench to srb by @AndrejOrsula
- Build(deps): bump chrono from 0.4.39 to 0.4.40 by @dependabot[bot] in [#46](https://github.com/AndrejOrsula/space_robotics_bench/pull/46)
- Build(deps): bump serde from 1.0.217 to 1.0.218 by @dependabot[bot] in [#45](https://github.com/AndrejOrsula/space_robotics_bench/pull/45)
- Build(deps): bump AdityaGarg8/remove-unwanted-software from 4 to 5 by @dependabot[bot] in [#44](https://github.com/AndrejOrsula/space_robotics_bench/pull/44)
- Build(deps): bump winit from 0.30.8 to 0.30.9 by @dependabot[bot] in [#43](https://github.com/AndrejOrsula/space_robotics_bench/pull/43)
- Build(deps): bump serde_json from 1.0.137 to 1.0.138 by @dependabot[bot] in [#38](https://github.com/AndrejOrsula/space_robotics_bench/pull/38)
- CI: Exclude GUI from llvm-cov by @AndrejOrsula
- Bump MSRV to 1.82 by @AndrejOrsula
- Docker: Skip GUI build by @AndrejOrsula
- Build(deps): bump serde_json from 1.0.135 to 1.0.137 by @dependabot[bot] in [#36](https://github.com/AndrejOrsula/space_robotics_bench/pull/36)
- Build(deps): bump thiserror from 2.0.9 to 2.0.11 by @dependabot[bot] in [#34](https://github.com/AndrejOrsula/space_robotics_bench/pull/34)
- Build(deps): bump pyo3 from 0.23.3 to 0.23.4 by @dependabot[bot] in [#35](https://github.com/AndrejOrsula/space_robotics_bench/pull/35)
- Build(deps): bump serde_json from 1.0.134 to 1.0.135 by @dependabot[bot] in [#33](https://github.com/AndrejOrsula/space_robotics_bench/pull/33)
- Build(deps): bump home from 0.5.9 to 0.5.11 by @dependabot[bot] in [#31](https://github.com/AndrejOrsula/space_robotics_bench/pull/31)
- Build(deps): bump egui from 0.29.1 to 0.30.0 by @dependabot[bot] in [#29](https://github.com/AndrejOrsula/space_robotics_bench/pull/29)
- Build(deps): bump serde from 1.0.216 to 1.0.217 by @dependabot[bot] in [#32](https://github.com/AndrejOrsula/space_robotics_bench/pull/32)
- Build(deps): bump sysinfo from 0.33.0 to 0.33.1 by @dependabot[bot] in [#30](https://github.com/AndrejOrsula/space_robotics_bench/pull/30)
- Build(deps): bump egui_commonmark from 0.18.0 to 0.19.0 by @dependabot[bot] in [#27](https://github.com/AndrejOrsula/space_robotics_bench/pull/27)
- Build(deps): bump eframe from 0.29.1 to 0.30.0 by @dependabot[bot] in [#28](https://github.com/AndrejOrsula/space_robotics_bench/pull/28)
- Build(deps): bump egui_extras from 0.29.1 to 0.30.0 by @dependabot[bot] in [#25](https://github.com/AndrejOrsula/space_robotics_bench/pull/25)
- Build(deps): bump serde_json from 1.0.133 to 1.0.134 by @dependabot[bot] in [#26](https://github.com/AndrejOrsula/space_robotics_bench/pull/26)
- Build(deps): bump thiserror from 2.0.7 to 2.0.9 by @dependabot[bot] in [#24](https://github.com/AndrejOrsula/space_robotics_bench/pull/24)
- Docs: Update Discord invite link by @AndrejOrsula
- Build(deps): bump thiserror from 2.0.6 to 2.0.7 by @dependabot[bot] in [#23](https://github.com/AndrejOrsula/space_robotics_bench/pull/23)
- Build(deps): bump serde from 1.0.215 to 1.0.216 by @dependabot[bot] in [#22](https://github.com/AndrejOrsula/space_robotics_bench/pull/22)
- Build(deps): bump chrono from 0.4.38 to 0.4.39 by @dependabot[bot] in [#21](https://github.com/AndrejOrsula/space_robotics_bench/pull/21)
- Build(deps): bump thiserror from 2.0.4 to 2.0.6 by @dependabot[bot] in [#20](https://github.com/AndrejOrsula/space_robotics_bench/pull/20)
- Build(deps): bump const_format from 0.2.33 to 0.2.34 by @dependabot[bot] in [#19](https://github.com/AndrejOrsula/space_robotics_bench/pull/19)
- Refactor: Improve organization by @AndrejOrsula
- Update dependencies (Blender 4.3.0, Isaac Lab 1.3.0, ...) by @AndrejOrsula
- Docker: Improve handling of DDS config for ROS 2 and Space ROS by @AndrejOrsula
- Build(deps): bump sysinfo from 0.32.0 to 0.32.1 by @dependabot[bot] in [#17](https://github.com/AndrejOrsula/space_robotics_bench/pull/17)
- Build(deps): bump tracing-subscriber from 0.3.18 to 0.3.19 by @dependabot[bot] in [#18](https://github.com/AndrejOrsula/space_robotics_bench/pull/18)
- Build(deps): bump tracing from 0.1.40 to 0.1.41 by @dependabot[bot] in [#16](https://github.com/AndrejOrsula/space_robotics_bench/pull/16)
- Build(deps): bump r2r from 0.9.3 to 0.9.4 by @dependabot[bot] in [#15](https://github.com/AndrejOrsula/space_robotics_bench/pull/15)
- Build(deps): bump serde from 1.0.214 to 1.0.215 by @dependabot[bot] in [#11](https://github.com/AndrejOrsula/space_robotics_bench/pull/11)
- Build(deps): bump serde_json from 1.0.132 to 1.0.133 by @dependabot[bot] in [#10](https://github.com/AndrejOrsula/space_robotics_bench/pull/10)
- Build(deps): bump codecov/codecov-action from 4 to 5 by @dependabot[bot] in [#9](https://github.com/AndrejOrsula/space_robotics_bench/pull/9)
- Build(deps): bump r2r from 0.9.2 to 0.9.3 by @dependabot[bot] in [#8](https://github.com/AndrejOrsula/space_robotics_bench/pull/8)
- Build(deps): bump image from 0.25.4 to 0.25.5 by @dependabot[bot] in [#7](https://github.com/AndrejOrsula/space_robotics_bench/pull/7)
- Build(deps): bump pyo3 from 0.22.5 to 0.22.6 by @dependabot[bot] in [#6](https://github.com/AndrejOrsula/space_robotics_bench/pull/6)
- Update rendering settings by @AndrejOrsula
- Build(deps): bump serde from 1.0.210 to 1.0.214 by @dependabot[bot] in [#5](https://github.com/AndrejOrsula/space_robotics_bench/pull/5)
- Bump thiserror from 1.0.64 to 1.0.65 by @dependabot[bot] in [#3](https://github.com/AndrejOrsula/space_robotics_bench/pull/3)
- Bump EmbarkStudios/cargo-deny-action from 1 to 2 by @dependabot[bot] in [#2](https://github.com/AndrejOrsula/space_robotics_bench/pull/2)
- Transfer script for automated procgen with Blender to `srb_assets` submodule by @AndrejOrsula
- CI: Disable docker job for Dependabot PRs by @AndrejOrsula
- Docker: Use local Rust extension module if the project is mounted as a volume by @AndrejOrsula
- Big Bang by @AndrejOrsula

### Fixed
- CI: Fix Rust workflow by @AndrejOrsula
- GUI: Fix winit initialization by @AndrejOrsula

### Removed
- Remove direct reference dependencies by @AndrejOrsula
- Docs: Remove instructions about NGC Docker login by @AndrejOrsula
- Cargo-deny: Remove deprecated keys by @AndrejOrsula
- Pre-commit: Remove redundant excludes by @AndrejOrsula

## New Contributors
* @AndrejOrsula made their first contribution
* @dependabot[bot] made their first contribution in [#46](https://github.com/AndrejOrsula/space_robotics_bench/pull/46)
[0.0.2]: https://github.com/AndrejOrsula/space_robotics_bench/compare/0.0.1..0.0.2

<!-- generated by git-cliff -->
