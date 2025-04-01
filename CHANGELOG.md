# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.3] - 2025-04-01

### Added
- Add landing task ([59f87a9](https://github.com/AndrejOrsula/space_robotics_bench/commit/59f87a98b920cc7a13831257886b9df9731ecc6b))
- Add thrust action term and group ([3e98a74](https://github.com/AndrejOrsula/space_robotics_bench/commit/3e98a7460af03470f3ce2676a2b2193a224328fe))
- Add support for OSC action term ([152973b](https://github.com/AndrejOrsula/space_robotics_bench/commit/152973b42a4e84e81181cd00dbf496e9013ec099))
- Git-cliff: Add commits to the changelog ([6521a4a](https://github.com/AndrejOrsula/space_robotics_bench/commit/6521a4a2ae2c213086fd2a01412756ef93b352f1)) by @AndrejOrsula

### Changed
- Bump to 0.0.3
- Docs: Update envs, robots, attributes ([73b6462](https://github.com/AndrejOrsula/space_robotics_bench/commit/73b64622f1cb7a6aa168be91354cf71dee2f85b1))
- Improve default hyperparameters of Dreamer ([7a97996](https://github.com/AndrejOrsula/space_robotics_bench/commit/7a97996a3ca25939e3a6398db86ee51766a6eeb5))
- Update assets ([7988615](https://github.com/AndrejOrsula/space_robotics_bench/commit/7988615ab3c4b2ec371ea26a7f7ce9c056f793ab))
- Tune peg_in_hole rewards ([5144243](https://github.com/AndrejOrsula/space_robotics_bench/commit/51442431629760fd669d3451ebfb0cba338ec83f))
- Update environment and agent rates in base environments ([cde5eb5](https://github.com/AndrejOrsula/space_robotics_bench/commit/cde5eb5dc17a4c507017b54713baa1fc393cd1fd))
- Update skydome asset directory path ([5c1efee](https://github.com/AndrejOrsula/space_robotics_bench/commit/5c1efee9006d30033a30a5ee0f0a2d873f7ea75d))
- Tests: Skip CLI agent train test ([b523ef4](https://github.com/AndrejOrsula/space_robotics_bench/commit/b523ef4c8b90d5f271cc66526b7d290a98696fa6))
- Standardize teleop device sensitivity ([288f8eb](https://github.com/AndrejOrsula/space_robotics_bench/commit/288f8ebffec6766772b936af1c9e80ab6b786cd8))
- Docker: Update development script with DEBUG_VIS environ ([3b46457](https://github.com/AndrejOrsula/space_robotics_bench/commit/3b46457cdfdc55387af41d39cb9bbe2e1b7e8402))
- Update base events and skydomes ([d8172d2](https://github.com/AndrejOrsula/space_robotics_bench/commit/d8172d22a99eb121f2f9a513eb2dc3e4d26f5d59))
- Autoreset environment instances that explode due to physics ([ed88f2a](https://github.com/AndrejOrsula/space_robotics_bench/commit/ed88f2aa5d59ac51dcfd575f38fc9d7f6ef7060b))
- GiGeneralize action term for all wheeled robots ([c5b38e8](https://github.com/AndrejOrsula/space_robotics_bench/commit/c5b38e88a75e07d012ec078da199f4061da26740))
- Docker: Mount Omniverse data volume ([502d5b3](https://github.com/AndrejOrsula/space_robotics_bench/commit/502d5b37ee48003eec5e55f425b047360dc4dcac))
- Build(deps): bump image from 0.25.5 to 0.25.6 ([eab74fc](https://github.com/AndrejOrsula/space_robotics_bench/commit/eab74fc93b74a19f9641fe632a340edb8864f82f)) by @dependabot[bot] in [#55](https://github.com/AndrejOrsula/space_robotics_bench/pull/55)
- Build(deps): bump sysinfo from 0.33.1 to 0.34.1 ([8db4e77](https://github.com/AndrejOrsula/space_robotics_bench/commit/8db4e7762cfd3a5d1446f1d5d4244ed6f52d6a42)) by @dependabot[bot] in [#54](https://github.com/AndrejOrsula/space_robotics_bench/pull/54)
- Build(deps): bump typed-builder from 0.20.1 to 0.21.0 ([a2d90a7](https://github.com/AndrejOrsula/space_robotics_bench/commit/a2d90a7231e466e252295f6b1daa4da27dcb3bba)) by @dependabot[bot] in [#53](https://github.com/AndrejOrsula/space_robotics_bench/pull/53)

### Fixed
- Fix naming of IMU visualization marker ([4558a28](https://github.com/AndrejOrsula/space_robotics_bench/commit/4558a28c415843ebf56ee0e6937906be9be65eaf))
- Fix contact sensor ([4e3cd28](https://github.com/AndrejOrsula/space_robotics_bench/commit/4e3cd289a6e4ae53461889333e4fcf8ac5478107))

## [0.0.2] - 2025-03-20

### Added
- Add git-cliff configuration ([ed79c90](https://github.com/AndrejOrsula/space_robotics_bench/commit/ed79c90d3d1494e4d6be5bac6f412b5e46be192d)) by @AndrejOrsula
- Add templates for mobile manipulation tasks ([6dcf11a](https://github.com/AndrejOrsula/space_robotics_bench/commit/6dcf11a1cdd36784c14a42c37b1fbe85f203d960)) by @AndrejOrsula
- Docker: Add option to ensure Docker and NVIDIA toolkit are installed ([4f0ce88](https://github.com/AndrejOrsula/space_robotics_bench/commit/4f0ce88c5cd14ff98cd91b89606b67f3a8fdd20e)) by @AndrejOrsula
- Devcontainer: Add default extensions ([6b2bc4d](https://github.com/AndrejOrsula/space_robotics_bench/commit/6b2bc4d52cdb74fa787e062d2185d45a362f299f)) by @AndrejOrsula

### Changed
- Bump to 0.0.2 ([b0a8a4b](https://github.com/AndrejOrsula/space_robotics_bench/commit/b0a8a4bc3c6ec40d41b53a89ce9da30319d13a4f)) by @AndrejOrsula
- Docs: Update towards 0.0.2 ([bf4fdb6](https://github.com/AndrejOrsula/space_robotics_bench/commit/bf4fdb610e301e464c5a8081169b13b19b2b36d5)) by @AndrejOrsula
- Bump MSRV to 1.84 ([bdceba6](https://github.com/AndrejOrsula/space_robotics_bench/commit/bdceba6fae7c292d02af3597bcef2a1ab1c99c6c)) by @AndrejOrsula
- Update dependencies (Python & Rust) ([9eea79d](https://github.com/AndrejOrsula/space_robotics_bench/commit/9eea79d00f97ba5cdfe5c996e5f56743388dfaf0)) by @AndrejOrsula
- Deny: Ignore RUSTSEC-2024-0436 ([22ee4b3](https://github.com/AndrejOrsula/space_robotics_bench/commit/22ee4b3045fbada214ebc6c0282a7d728db17bf4)) by @AndrejOrsula
- Docker: Update commits of dev dependencies ([3b69be8](https://github.com/AndrejOrsula/space_robotics_bench/commit/3b69be80ab65ff804a734050d35b8a70c8f92ae0)) by @AndrejOrsula
- Tests: Update to match CLI changes ([02e1c56](https://github.com/AndrejOrsula/space_robotics_bench/commit/02e1c5647c1249b5e144b54b85452c92cccd45e1)) by @AndrejOrsula
- CLI: Streamline usage for readable documentation ([1265f8d](https://github.com/AndrejOrsula/space_robotics_bench/commit/1265f8d163e09ee1a94d9c699907394daab57130)) by @AndrejOrsula
- Locomotion_velocity_tracking: Adjust reward ([4755ec8](https://github.com/AndrejOrsula/space_robotics_bench/commit/4755ec8a1d0cd23b85185a1c2e0e62a8627d436c)) by @AndrejOrsula
- Mobile_debris_capture: Use default robot of the base environment class ([7b00c2e](https://github.com/AndrejOrsula/space_robotics_bench/commit/7b00c2e98c125feed02c9c3dd53f412aac82376e)) by @AndrejOrsula
- Orbital_evasion: Update observation and reward ([578407a](https://github.com/AndrejOrsula/space_robotics_bench/commit/578407a4c818c2e53103a352c61acc5340e0f905)) by @AndrejOrsula
- Excavation: Default to Spot robot mobile base ([a96a135](https://github.com/AndrejOrsula/space_robotics_bench/commit/a96a135bd7009d9a3ebaa162e8760062c0416572)) by @AndrejOrsula
- Refactor environment classes to improve naming consistency across manipulation tasks ([f7176f2](https://github.com/AndrejOrsula/space_robotics_bench/commit/f7176f2b8205f71ef6a5beb8c7ee2593770d7af4)) by @AndrejOrsula
- Simplify base parameter naming in environment config ([1e996e5](https://github.com/AndrejOrsula/space_robotics_bench/commit/1e996e5aac92233dc4c3e0bbae127f03fb4d69b3)) by @AndrejOrsula
- CLI: Improve command-line overrides for environment config ([1f1895f](https://github.com/AndrejOrsula/space_robotics_bench/commit/1f1895fcfc5d909e009d2ad652167f8a2878de88)) by @AndrejOrsula
- Improve asset configuration consistency ([71746aa](https://github.com/AndrejOrsula/space_robotics_bench/commit/71746aa21b57ce0a8a81f8abf1d29ff916898e6b)) by @AndrejOrsula
- Docker: Update the default development volumes ([7acd717](https://github.com/AndrejOrsula/space_robotics_bench/commit/7acd71728e56f08692a9d4b0f4bd4751b0affa87)) by @AndrejOrsula
- Docker: Ensure assets are initialized when building the image ([dbd70fe](https://github.com/AndrejOrsula/space_robotics_bench/commit/dbd70fe9e6c7b45b8908666982d4a7b6de2da1bc)) by @AndrejOrsula
- Update installation scripts ([69b6227](https://github.com/AndrejOrsula/space_robotics_bench/commit/69b6227fe95ea6a09df747c6458e6b263ea396ab)) by @AndrejOrsula
- CI: Checkout submodules recursively to build Docker with assets ([3b351d7](https://github.com/AndrejOrsula/space_robotics_bench/commit/3b351d73c4c64501b1bee1fa2bdbe10332355c68)) by @AndrejOrsula
- Build(deps): bump typed-builder from 0.20.0 to 0.20.1 ([f91220d](https://github.com/AndrejOrsula/space_robotics_bench/commit/f91220de5bd8eab9ff41e505bc806cd10699bd5a)) by @dependabot[bot] in [#52](https://github.com/AndrejOrsula/space_robotics_bench/pull/52)
- Build(deps): bump egui_extras from 0.31.0 to 0.31.1 ([a7691b9](https://github.com/AndrejOrsula/space_robotics_bench/commit/a7691b91e7d0698d34d80729502cf1d6738f88e8)) by @dependabot[bot] in [#51](https://github.com/AndrejOrsula/space_robotics_bench/pull/51)
- Build(deps): bump eframe from 0.31.0 to 0.31.1 ([03615af](https://github.com/AndrejOrsula/space_robotics_bench/commit/03615af8960557a879b780d8f9376375acbd82c6)) by @dependabot[bot] in [#48](https://github.com/AndrejOrsula/space_robotics_bench/pull/48)
- Build(deps): bump serde from 1.0.218 to 1.0.219 ([0b80920](https://github.com/AndrejOrsula/space_robotics_bench/commit/0b80920086b57bb074e910d23251c471b696f535)) by @dependabot[bot] in [#49](https://github.com/AndrejOrsula/space_robotics_bench/pull/49)

### Fixed
- Correct action term/group naming ([7b30546](https://github.com/AndrejOrsula/space_robotics_bench/commit/7b305467f72bbf84509a267b3fa35968bd082a00)) by @AndrejOrsula

### Removed
- Remove redundant event ([92e4f7d](https://github.com/AndrejOrsula/space_robotics_bench/commit/92e4f7d44235816eebe02b9d642340fe66285cc7)) by @AndrejOrsula

## [0.0.1] - 2025-03-04

### Added
- Add barebones mobile_debris_capture task ([a387421](https://github.com/AndrejOrsula/space_robotics_bench/commit/a3874214f2564f96ef6d1bb969b5aaa5188343d4)) by @AndrejOrsula
- Add barebones excavation task ([9438761](https://github.com/AndrejOrsula/space_robotics_bench/commit/94387612a784b1f449d98dce3a3ea9ff3a61e07f)) by @AndrejOrsula
- Add orbital_evasion task ([fab2b5c](https://github.com/AndrejOrsula/space_robotics_bench/commit/fab2b5cd8a9025f0ae0f619d48c4b88121e8f8d6)) by @AndrejOrsula
- Add locomotion_velocity_tracking task ([1665201](https://github.com/AndrejOrsula/space_robotics_bench/commit/16652019b9497cd5d50cc0ac0bf097a95804180b)) by @AndrejOrsula
- Add solar_panel_assembly task ([4867c6b](https://github.com/AndrejOrsula/space_robotics_bench/commit/4867c6b1b349f73423417b23130abd5d1e360876)) by @AndrejOrsula
- Add peg_in_hole_assembly task ([bdb4113](https://github.com/AndrejOrsula/space_robotics_bench/commit/bdb411350ac00f4d32b34f6907e8fa0c20b6c3c7)) by @AndrejOrsula
- Add sample_collection task ([21e09d1](https://github.com/AndrejOrsula/space_robotics_bench/commit/21e09d15c05e3e34151c310f09a9e7ac99f8553f)) by @AndrejOrsula
- Add debris_capture task ([b73a09c](https://github.com/AndrejOrsula/space_robotics_bench/commit/b73a09ccc2ba7702e169c7d8ebc7aee9a60382e7)) by @AndrejOrsula
- Add basic tests ([764fbb8](https://github.com/AndrejOrsula/space_robotics_bench/commit/764fbb8336e4eb11f13cf52ed159cf09095a6336)) by @AndrejOrsula
- Add unified entrypoint script ([ab35b1b](https://github.com/AndrejOrsula/space_robotics_bench/commit/ab35b1b66020101a8c8a7d1786f19c93657af07d)) by @AndrejOrsula
- Add config and hyparparam utils ([444c99f](https://github.com/AndrejOrsula/space_robotics_bench/commit/444c99ff0a26538030a26913eb0eac9036999e95)) by @AndrejOrsula
- Add mobile manipulation base envs ([13d37ca](https://github.com/AndrejOrsula/space_robotics_bench/commit/13d37ca44323fa3b4c000e9ce4b3c67fbbce7152)) by @AndrejOrsula
- Add mobile manipulation base envs ([5eac2a8](https://github.com/AndrejOrsula/space_robotics_bench/commit/5eac2a8592a9a61a00cdda13d935a17913cf53a2)) by @AndrejOrsula
- Add manipulation base env and task template ([486da35](https://github.com/AndrejOrsula/space_robotics_bench/commit/486da35822a82b8e1c0762d1820e5512b2ab50a0)) by @AndrejOrsula
- Add ROS 2 interface ([7028fc7](https://github.com/AndrejOrsula/space_robotics_bench/commit/7028fc7e574762342560d23ec6c46eb9ac1b973c)) by @AndrejOrsula
- Add skrl integration ([52cde70](https://github.com/AndrejOrsula/space_robotics_bench/commit/52cde701564dfe0fde84bf0c2017e1ec43aab070)) by @AndrejOrsula
- Add SB3 and SBX integrations ([7cf19a3](https://github.com/AndrejOrsula/space_robotics_bench/commit/7cf19a3f761cf1c8c8593252d8b8fdfd85aebdff)) by @AndrejOrsula
- Add Dreamer integration ([56a1ccf](https://github.com/AndrejOrsula/space_robotics_bench/commit/56a1ccf5f51fc7065eec4cc2a196dd87fddc0f80)) by @AndrejOrsula
- Add shape and ground plane assets ([1294149](https://github.com/AndrejOrsula/space_robotics_bench/commit/1294149741302d9f01f300ba1e39c6e83876a980)) by @AndrejOrsula
- Add object/scenery assets from srb_assets ([1cb3933](https://github.com/AndrejOrsula/space_robotics_bench/commit/1cb39330564285d6c0ad9087c759a12599f73106)) by @AndrejOrsula
- Add initial procedural SimForge assets ([8a42593](https://github.com/AndrejOrsula/space_robotics_bench/commit/8a42593609a0db5a7250fcbf6e094b2d6389cfca)) by @AndrejOrsula
- Add initial tools (end-effectors) ([a1b686a](https://github.com/AndrejOrsula/space_robotics_bench/commit/a1b686a17a53ed980f2fad47afa1633f4f1ab03a)) by @AndrejOrsula
- Add initial robot assets ([2669fd1](https://github.com/AndrejOrsula/space_robotics_bench/commit/2669fd16c655de6adeb6a7458aa68aa2596d1157)) by @AndrejOrsula
- Add custom Franka arm and FrankaHand tool (separate) ([594cab1](https://github.com/AndrejOrsula/space_robotics_bench/commit/594cab164e9189daea23a22252bf5596360bc92d)) by @AndrejOrsula
- Add AnyEnv/AnyEnvCfg type aliases ([c5f7098](https://github.com/AndrejOrsula/space_robotics_bench/commit/c5f7098d9255f317cedeb34bb3b62b5fd5a8e1fd)) by @AndrejOrsula
- Add GUI interface ([e5739f3](https://github.com/AndrejOrsula/space_robotics_bench/commit/e5739f3927227be31ed73e9da659fd8ec306c0d5)) by @AndrejOrsula
- Add teleop interfaces ([bf803d9](https://github.com/AndrejOrsula/space_robotics_bench/commit/bf803d9e78cd3f4c337e59330543300af5402d3d)) by @AndrejOrsula
- Add visual environment extension ([87d5c30](https://github.com/AndrejOrsula/space_robotics_bench/commit/87d5c30b91d9ec7c3b751a4183c6b724dc060d45)) by @AndrejOrsula
- Add common environment base classes ([7564e4b](https://github.com/AndrejOrsula/space_robotics_bench/commit/7564e4bd6e5617d2ec2f22f5cea9d82f6a7802f2)) by @AndrejOrsula
- Add oxidasim sampling utils ([7875e52](https://github.com/AndrejOrsula/space_robotics_bench/commit/7875e52e0d845fd6882a54fa9999637424ab11b0)) by @AndrejOrsula
- Add mobile robot action terms/groups ([7de74a9](https://github.com/AndrejOrsula/space_robotics_bench/commit/7de74a943a70f367f61952c6199b5993343fdccb)) by @AndrejOrsula
- Add task space manipulation action terms/groups ([96c3433](https://github.com/AndrejOrsula/space_robotics_bench/commit/96c3433321ef3892faa50a7475febab61c231504)) by @AndrejOrsula
- Add ParticleSpawner ([53b53d9](https://github.com/AndrejOrsula/space_robotics_bench/commit/53b53d9f8b7a193fb18fa610ea41bf33eed3540f)) by @AndrejOrsula
- Add common action terms and groups ([c6e657f](https://github.com/AndrejOrsula/space_robotics_bench/commit/c6e657fe8a45f89f7ebcbaceca9f3a897d3be525)) by @AndrejOrsula
- Add custom events ([3bd0866](https://github.com/AndrejOrsula/space_robotics_bench/commit/3bd0866903fa94895fa09cc4a2079a8653246daf)) by @AndrejOrsula
- Add custom visualization markers ([06d2ec2](https://github.com/AndrejOrsula/space_robotics_bench/commit/06d2ec260472afdb465d127cba028abdc4a66eb0)) by @AndrejOrsula
- Add custom RobotAssembler ([71cf4cb](https://github.com/AndrejOrsula/space_robotics_bench/commit/71cf4cb3b1ce811aaeabff2902030ec47a24d284)) by @AndrejOrsula
- Add extra shape spawners ([119fb84](https://github.com/AndrejOrsula/space_robotics_bench/commit/119fb84dec69270ec06882437cbd7a5d83a08372)) by @AndrejOrsula
- Add Domain enum with utils ([474fcd0](https://github.com/AndrejOrsula/space_robotics_bench/commit/474fcd0cf724d7cbbeaaed6fe800c1c3d66209ef)) by @AndrejOrsula
- Add config for RTX visuals and post-processing ([20bbc67](https://github.com/AndrejOrsula/space_robotics_bench/commit/20bbc67546e890f7f08dc69eb3fb8d6796f36bd8)) by @AndrejOrsula
- Add logging and tracing utils ([6173aa3](https://github.com/AndrejOrsula/space_robotics_bench/commit/6173aa32befdc3d615e089a6a40b930fdd9ec07a)) by @AndrejOrsula
- Add common utils ([31ba454](https://github.com/AndrejOrsula/space_robotics_bench/commit/31ba45439a833c6f6f19dbd9a20873b2bfa7ef40)) by @AndrejOrsula
- Docs: Add button for suggesting edits ([360a8bf](https://github.com/AndrejOrsula/space_robotics_bench/commit/360a8bf9f41af609dde6db40b6f2b8b7f134ccff)) by @AndrejOrsula
- Docs: Add link to Discord ([d351948](https://github.com/AndrejOrsula/space_robotics_bench/commit/d3519486d17c792ccb429dc490802c2825938e83)) by @AndrejOrsula
- Docker: Add option to install Space ROS ([a788d05](https://github.com/AndrejOrsula/space_robotics_bench/commit/a788d056e6e24400bfdd58f85ba526c13ce02be4)) by @AndrejOrsula
- CLI: Add short args and update environ for extension module update ([a30cfe0](https://github.com/AndrejOrsula/space_robotics_bench/commit/a30cfe07bc02a56d771f1ccf6363893970289839)) by @AndrejOrsula

### Changed
- CI: Build Python package with uv ([b23724e](https://github.com/AndrejOrsula/space_robotics_bench/commit/b23724ee6aa641e55f81c71959246442146da7ea)) by @AndrejOrsula
- CI: Disable llvm-cov in Rust workflow ([8f68f99](https://github.com/AndrejOrsula/space_robotics_bench/commit/8f68f995e3515e7d24b89e1ede0126b5e5b4d9de)) by @AndrejOrsula
- Pre-commit: Downgrade mdformat ([c1c8c6c](https://github.com/AndrejOrsula/space_robotics_bench/commit/c1c8c6c36911dea05084061c42e3c5eaac34f9aa)) by @AndrejOrsula
- GUI: Replace missing image ([a64b316](https://github.com/AndrejOrsula/space_robotics_bench/commit/a64b31668d1a708d550c723ce8fefa0a2c4ede26)) by @AndrejOrsula
- CI: Update Python/Rust workflows ([aa83406](https://github.com/AndrejOrsula/space_robotics_bench/commit/aa83406f50846bb83ccdcd9c4fbc014e0c72e2b2)) by @AndrejOrsula
- Update badges in README ([6fa634f](https://github.com/AndrejOrsula/space_robotics_bench/commit/6fa634ff368abdc6cd11eb81bc23c6911fb45e4f)) by @AndrejOrsula
- Patch ActionManager to improve compatibility with ActionGroup ([e882776](https://github.com/AndrejOrsula/space_robotics_bench/commit/e882776c8dc74d9d3b86efe3c44c290baf7c9f6c)) by @AndrejOrsula
- Wrap around Isaac Lab core modules ([38455a1](https://github.com/AndrejOrsula/space_robotics_bench/commit/38455a1879501ccfd12f6d0518756b82c6db71f1)) by @AndrejOrsula
- Define ActionGroup model ([d7095a6](https://github.com/AndrejOrsula/space_robotics_bench/commit/d7095a6de15102f1c5ab4ec498b4ef26effd3c02)) by @AndrejOrsula
- Define the full asset hierarchy model ([f3511a7](https://github.com/AndrejOrsula/space_robotics_bench/commit/f3511a73e1f888f7839f8e35000c55be43670ee9)) by @AndrejOrsula
- Integrate uv ([950ef53](https://github.com/AndrejOrsula/space_robotics_bench/commit/950ef53b5d7b10219e10dac69ca23a1b9e023fb4)) by @AndrejOrsula
- Update Docker setup ([866287d](https://github.com/AndrejOrsula/space_robotics_bench/commit/866287d57cf7b873af2e6b1348cb3c222832988b)) by @AndrejOrsula
- Integrate SimForge ([64c9bb6](https://github.com/AndrejOrsula/space_robotics_bench/commit/64c9bb652cf994bec9ee19cca87c6b46e82d0a95)) by @AndrejOrsula
- Update srb_assets ([b2a3757](https://github.com/AndrejOrsula/space_robotics_bench/commit/b2a3757b75933c999d68de118fbfb23b0914eed7)) by @AndrejOrsula
- Update pre-commit hooks ([a2a4d69](https://github.com/AndrejOrsula/space_robotics_bench/commit/a2a4d69749d1483095c8d353eae5dd4810edefb8)) by @AndrejOrsula
- Update copyright year to 2025 ([2c379dc](https://github.com/AndrejOrsula/space_robotics_bench/commit/2c379dc33002174acdd20947f9786a0a991e0c9a)) by @AndrejOrsula
- Update to Isaac Sim 4.5 ([3b0ff36](https://github.com/AndrejOrsula/space_robotics_bench/commit/3b0ff36484313a897d4d3c6f495993b5cb9a9792)) by @AndrejOrsula
- Update module name from space_robotics_bench to srb ([803703b](https://github.com/AndrejOrsula/space_robotics_bench/commit/803703bcfb173bc387babd547b76db6ad6ba5b33)) by @AndrejOrsula
- Build(deps): bump chrono from 0.4.39 to 0.4.40 ([2dce970](https://github.com/AndrejOrsula/space_robotics_bench/commit/2dce970e89226d7e4fd8ebf040496045626fc23b)) by @dependabot[bot] in [#46](https://github.com/AndrejOrsula/space_robotics_bench/pull/46)
- Build(deps): bump serde from 1.0.217 to 1.0.218 ([2c33aaf](https://github.com/AndrejOrsula/space_robotics_bench/commit/2c33aaf1bef0e37d3371da16dd9c558bc70fb26b)) by @dependabot[bot] in [#45](https://github.com/AndrejOrsula/space_robotics_bench/pull/45)
- Build(deps): bump AdityaGarg8/remove-unwanted-software from 4 to 5 ([c25adbf](https://github.com/AndrejOrsula/space_robotics_bench/commit/c25adbfdd389955880a7ef90960329053d70e4cf)) by @dependabot[bot] in [#44](https://github.com/AndrejOrsula/space_robotics_bench/pull/44)
- Build(deps): bump winit from 0.30.8 to 0.30.9 ([f3013a0](https://github.com/AndrejOrsula/space_robotics_bench/commit/f3013a0f22baef476bfea9a1dadede18fd49ac16)) by @dependabot[bot] in [#43](https://github.com/AndrejOrsula/space_robotics_bench/pull/43)
- Build(deps): bump serde_json from 1.0.137 to 1.0.138 ([e90c8a2](https://github.com/AndrejOrsula/space_robotics_bench/commit/e90c8a24c1d9e2653d1e67a611cb434d731fe4aa)) by @dependabot[bot] in [#38](https://github.com/AndrejOrsula/space_robotics_bench/pull/38)
- CI: Exclude GUI from llvm-cov ([eb925d7](https://github.com/AndrejOrsula/space_robotics_bench/commit/eb925d7b670f1de68e73fad43666f40f05c3dbcd)) by @AndrejOrsula
- Bump MSRV to 1.82 ([ed6a5d7](https://github.com/AndrejOrsula/space_robotics_bench/commit/ed6a5d7b09ff83643286c5694b5bb68522c3e35d)) by @AndrejOrsula
- Docker: Skip GUI build ([4dfb4de](https://github.com/AndrejOrsula/space_robotics_bench/commit/4dfb4de5d931aa6a5cb011b755199ca455e66fa2)) by @AndrejOrsula
- Build(deps): bump serde_json from 1.0.135 to 1.0.137 ([8bcc239](https://github.com/AndrejOrsula/space_robotics_bench/commit/8bcc2390caa8c53bcbc7a460a17889ca449141f7)) by @dependabot[bot] in [#36](https://github.com/AndrejOrsula/space_robotics_bench/pull/36)
- Build(deps): bump thiserror from 2.0.9 to 2.0.11 ([651785c](https://github.com/AndrejOrsula/space_robotics_bench/commit/651785ca329b2feb2d1414443fe9e1e3dd930ba2)) by @dependabot[bot] in [#34](https://github.com/AndrejOrsula/space_robotics_bench/pull/34)
- Build(deps): bump pyo3 from 0.23.3 to 0.23.4 ([422adb7](https://github.com/AndrejOrsula/space_robotics_bench/commit/422adb708bdb17b618797cac95ce75d0a2a18678)) by @dependabot[bot] in [#35](https://github.com/AndrejOrsula/space_robotics_bench/pull/35)
- Build(deps): bump serde_json from 1.0.134 to 1.0.135 ([6bc337d](https://github.com/AndrejOrsula/space_robotics_bench/commit/6bc337d237968c0b8232293c89008e10dc68a8bd)) by @dependabot[bot] in [#33](https://github.com/AndrejOrsula/space_robotics_bench/pull/33)
- Build(deps): bump home from 0.5.9 to 0.5.11 ([0f115c9](https://github.com/AndrejOrsula/space_robotics_bench/commit/0f115c99ecdb76d9f74f48253eb1ac1d72f84b6a)) by @dependabot[bot] in [#31](https://github.com/AndrejOrsula/space_robotics_bench/pull/31)
- Build(deps): bump egui from 0.29.1 to 0.30.0 ([a712228](https://github.com/AndrejOrsula/space_robotics_bench/commit/a712228628872270b179dbb5d405da59b9d037bc)) by @dependabot[bot] in [#29](https://github.com/AndrejOrsula/space_robotics_bench/pull/29)
- Build(deps): bump serde from 1.0.216 to 1.0.217 ([c68f4a1](https://github.com/AndrejOrsula/space_robotics_bench/commit/c68f4a1848eca0a6e9066a20a8779106dcbb9176)) by @dependabot[bot] in [#32](https://github.com/AndrejOrsula/space_robotics_bench/pull/32)
- Build(deps): bump sysinfo from 0.33.0 to 0.33.1 ([e21bca9](https://github.com/AndrejOrsula/space_robotics_bench/commit/e21bca9808fef309b6843c138c026e8291dc97e5)) by @dependabot[bot] in [#30](https://github.com/AndrejOrsula/space_robotics_bench/pull/30)
- Build(deps): bump egui_commonmark from 0.18.0 to 0.19.0 ([883d29a](https://github.com/AndrejOrsula/space_robotics_bench/commit/883d29a45f3b17407f1b42cf9f2441b2c4cd7390)) by @dependabot[bot] in [#27](https://github.com/AndrejOrsula/space_robotics_bench/pull/27)
- Build(deps): bump eframe from 0.29.1 to 0.30.0 ([d3151dd](https://github.com/AndrejOrsula/space_robotics_bench/commit/d3151dde127915d0f5903caa88b824f0e2538cdd)) by @dependabot[bot] in [#28](https://github.com/AndrejOrsula/space_robotics_bench/pull/28)
- Build(deps): bump egui_extras from 0.29.1 to 0.30.0 ([964a35e](https://github.com/AndrejOrsula/space_robotics_bench/commit/964a35e95da16a322d5909b9143a7a833baaa20f)) by @dependabot[bot] in [#25](https://github.com/AndrejOrsula/space_robotics_bench/pull/25)
- Build(deps): bump serde_json from 1.0.133 to 1.0.134 ([7e80f2d](https://github.com/AndrejOrsula/space_robotics_bench/commit/7e80f2d2d58c620880d59ac7789c1168ea87f18e)) by @dependabot[bot] in [#26](https://github.com/AndrejOrsula/space_robotics_bench/pull/26)
- Build(deps): bump thiserror from 2.0.7 to 2.0.9 ([fe1dfb0](https://github.com/AndrejOrsula/space_robotics_bench/commit/fe1dfb0be9a4907bb60f1a1bba0585ff7f1d0a7b)) by @dependabot[bot] in [#24](https://github.com/AndrejOrsula/space_robotics_bench/pull/24)
- Docs: Update Discord invite link ([1674cd9](https://github.com/AndrejOrsula/space_robotics_bench/commit/1674cd91fe7b5600a818f85b5f9c66598000506f)) by @AndrejOrsula
- Build(deps): bump thiserror from 2.0.6 to 2.0.7 ([c3aaa47](https://github.com/AndrejOrsula/space_robotics_bench/commit/c3aaa47c8730d64c5cb294715d6e4e93839e65c7)) by @dependabot[bot] in [#23](https://github.com/AndrejOrsula/space_robotics_bench/pull/23)
- Build(deps): bump serde from 1.0.215 to 1.0.216 ([e30ccc9](https://github.com/AndrejOrsula/space_robotics_bench/commit/e30ccc9ea478bc55bf8175a1833830243fbad5f5)) by @dependabot[bot] in [#22](https://github.com/AndrejOrsula/space_robotics_bench/pull/22)
- Build(deps): bump chrono from 0.4.38 to 0.4.39 ([18c91aa](https://github.com/AndrejOrsula/space_robotics_bench/commit/18c91aa45ff6a6a27cfafbec9f1e2007cc7423dd)) by @dependabot[bot] in [#21](https://github.com/AndrejOrsula/space_robotics_bench/pull/21)
- Build(deps): bump thiserror from 2.0.4 to 2.0.6 ([a5801fc](https://github.com/AndrejOrsula/space_robotics_bench/commit/a5801fca2e5069ef514b229132452bda3d742b85)) by @dependabot[bot] in [#20](https://github.com/AndrejOrsula/space_robotics_bench/pull/20)
- Build(deps): bump const_format from 0.2.33 to 0.2.34 ([3aa9ab2](https://github.com/AndrejOrsula/space_robotics_bench/commit/3aa9ab249242cfd96cdebd411985ef0e09ca879e)) by @dependabot[bot] in [#19](https://github.com/AndrejOrsula/space_robotics_bench/pull/19)
- Refactor: Improve organization ([12d8179](https://github.com/AndrejOrsula/space_robotics_bench/commit/12d8179ffb5546896b645c2a37f8942320a840a0)) by @AndrejOrsula
- Update dependencies (Blender 4.3.0, Isaac Lab 1.3.0, ...) ([4432d7c](https://github.com/AndrejOrsula/space_robotics_bench/commit/4432d7c0eda0097a78531fc1c1b697030fa7e3e3)) by @AndrejOrsula
- Docker: Improve handling of DDS config for ROS 2 and Space ROS ([476d4bf](https://github.com/AndrejOrsula/space_robotics_bench/commit/476d4bf845187dc86406f9023e1df2779d85733e)) by @AndrejOrsula
- Build(deps): bump sysinfo from 0.32.0 to 0.32.1 ([735c1bc](https://github.com/AndrejOrsula/space_robotics_bench/commit/735c1bc815e33e3b7a779cf4254480074c4053cb)) by @dependabot[bot] in [#17](https://github.com/AndrejOrsula/space_robotics_bench/pull/17)
- Build(deps): bump tracing-subscriber from 0.3.18 to 0.3.19 ([585fd11](https://github.com/AndrejOrsula/space_robotics_bench/commit/585fd11270a9949332bc878c9d28552fc5bdbd40)) by @dependabot[bot] in [#18](https://github.com/AndrejOrsula/space_robotics_bench/pull/18)
- Build(deps): bump tracing from 0.1.40 to 0.1.41 ([1a54067](https://github.com/AndrejOrsula/space_robotics_bench/commit/1a540670839fbdc51298070e9e601dd02bb9f974)) by @dependabot[bot] in [#16](https://github.com/AndrejOrsula/space_robotics_bench/pull/16)
- Build(deps): bump r2r from 0.9.3 to 0.9.4 ([d4e2f29](https://github.com/AndrejOrsula/space_robotics_bench/commit/d4e2f292e7efad5e192a9c0d0ad2b76cde817d08)) by @dependabot[bot] in [#15](https://github.com/AndrejOrsula/space_robotics_bench/pull/15)
- Build(deps): bump serde from 1.0.214 to 1.0.215 ([1b309d3](https://github.com/AndrejOrsula/space_robotics_bench/commit/1b309d3d8540ddccb14029287c34b9be3030b36a)) by @dependabot[bot] in [#11](https://github.com/AndrejOrsula/space_robotics_bench/pull/11)
- Build(deps): bump serde_json from 1.0.132 to 1.0.133 ([a16c951](https://github.com/AndrejOrsula/space_robotics_bench/commit/a16c951b4fc672d2d55f54ccb5ebecc360abea13)) by @dependabot[bot] in [#10](https://github.com/AndrejOrsula/space_robotics_bench/pull/10)
- Build(deps): bump codecov/codecov-action from 4 to 5 ([b5636d7](https://github.com/AndrejOrsula/space_robotics_bench/commit/b5636d7a67971605777b7a8f9b9e17162b7744bc)) by @dependabot[bot] in [#9](https://github.com/AndrejOrsula/space_robotics_bench/pull/9)
- Build(deps): bump r2r from 0.9.2 to 0.9.3 ([b37a42d](https://github.com/AndrejOrsula/space_robotics_bench/commit/b37a42d0410e142a9365a9243d89e1b51d6118d2)) by @dependabot[bot] in [#8](https://github.com/AndrejOrsula/space_robotics_bench/pull/8)
- Build(deps): bump image from 0.25.4 to 0.25.5 ([f3ab525](https://github.com/AndrejOrsula/space_robotics_bench/commit/f3ab52569c9341389133deb586e553e4f2451c57)) by @dependabot[bot] in [#7](https://github.com/AndrejOrsula/space_robotics_bench/pull/7)
- Build(deps): bump pyo3 from 0.22.5 to 0.22.6 ([4d642ed](https://github.com/AndrejOrsula/space_robotics_bench/commit/4d642ed2c52e9d714638eee8a9a25587f330865c)) by @dependabot[bot] in [#6](https://github.com/AndrejOrsula/space_robotics_bench/pull/6)
- Update rendering settings ([62dbeb6](https://github.com/AndrejOrsula/space_robotics_bench/commit/62dbeb645530c254a471fa685491c1b6a34f1019)) by @AndrejOrsula
- Build(deps): bump serde from 1.0.210 to 1.0.214 ([90ccbbe](https://github.com/AndrejOrsula/space_robotics_bench/commit/90ccbbe2f7d21d434958da26fd118c77cff64ec7)) by @dependabot[bot] in [#5](https://github.com/AndrejOrsula/space_robotics_bench/pull/5)
- Bump thiserror from 1.0.64 to 1.0.65 ([c4800c0](https://github.com/AndrejOrsula/space_robotics_bench/commit/c4800c052a31ddf0df6da0d613d49cf7d2b5fd9b)) by @dependabot[bot] in [#3](https://github.com/AndrejOrsula/space_robotics_bench/pull/3)
- Bump EmbarkStudios/cargo-deny-action from 1 to 2 ([d497810](https://github.com/AndrejOrsula/space_robotics_bench/commit/d4978109533877a87ae1cb6ce9fd15f6d5e87531)) by @dependabot[bot] in [#2](https://github.com/AndrejOrsula/space_robotics_bench/pull/2)
- Transfer script for automated procgen with Blender to `srb_assets` submodule ([137e61a](https://github.com/AndrejOrsula/space_robotics_bench/commit/137e61ad3a523c1ea877e76e338c0b4f11e28d6f)) by @AndrejOrsula
- CI: Disable docker job for Dependabot PRs ([1a70c2d](https://github.com/AndrejOrsula/space_robotics_bench/commit/1a70c2de3539f9e9d768ba448f50305e768b92be)) by @AndrejOrsula
- Docker: Use local Rust extension module if the project is mounted as a volume ([be00d09](https://github.com/AndrejOrsula/space_robotics_bench/commit/be00d093ef3bff5d747a034bd1c2e2a78c9afdd0)) by @AndrejOrsula
- Big Bang ([c8528ce](https://github.com/AndrejOrsula/space_robotics_bench/commit/c8528ce0013f7a58f300cd8e0937b88542d0b752)) by @AndrejOrsula

### Fixed
- CI: Fix Rust workflow ([2b98866](https://github.com/AndrejOrsula/space_robotics_bench/commit/2b988668c74fa81750bd849b14fbc6077f171d3f)) by @AndrejOrsula
- GUI: Fix winit initialization ([d9a3ddd](https://github.com/AndrejOrsula/space_robotics_bench/commit/d9a3ddd6b01e644f30ef966552e97e34c9e72493)) by @AndrejOrsula

### Removed
- Remove direct reference dependencies ([7fb8e73](https://github.com/AndrejOrsula/space_robotics_bench/commit/7fb8e73756d0666adf0c435087d2270b6dddb8ce)) by @AndrejOrsula
- Docs: Remove instructions about NGC Docker login ([9347214](https://github.com/AndrejOrsula/space_robotics_bench/commit/934721484e28993120dca20a0430e6426ead33fe)) by @AndrejOrsula
- Cargo-deny: Remove deprecated keys ([bd0d5da](https://github.com/AndrejOrsula/space_robotics_bench/commit/bd0d5dac3c457a3f76b3ae530ff1678a58afff7b)) by @AndrejOrsula
- Pre-commit: Remove redundant excludes ([cf6bf40](https://github.com/AndrejOrsula/space_robotics_bench/commit/cf6bf4028dac3044a0cf9864c4a7eb1bebfbf416)) by @AndrejOrsula

## New Contributors
* @AndrejOrsula made their first contribution
* @dependabot[bot] made their first contribution in [#46](https://github.com/AndrejOrsula/space_robotics_bench/pull/46)
[0.0.3]: https://github.com/AndrejOrsula/space_robotics_bench/compare/0.0.2..0.0.3
[0.0.2]: https://github.com/AndrejOrsula/space_robotics_bench/compare/0.0.1..0.0.2

<!-- generated by git-cliff -->
