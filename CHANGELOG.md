## [1.0.6](https://github.com/hinata-koizumi/drone_avoidance_rl/compare/v1.0.5...v1.0.6) (2025-07-19)


### Bug Fixes

* optimize Docker images to reduce storage usage in GitHub Actions ([8c127e3](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/8c127e3c0db2e71ff9641c56c00f3bbecb9e5330))

## [1.0.5](https://github.com/hinata-koizumi/drone_avoidance_rl/compare/v1.0.4...v1.0.5) (2025-07-19)


### Bug Fixes

* remove empty sample_agent directory from Docker build ([ccbc1a8](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/ccbc1a83d01d244784289ef71816c78588ec1206))

## [1.0.4](https://github.com/hinata-koizumi/drone_avoidance_rl/compare/v1.0.3...v1.0.4) (2025-07-19)


### Bug Fixes

* resolve state_bridge build dependency and RL agent file paths ([9e97c92](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/9e97c9251fffb59779fd4f8c699d927fcb94b1ca))

## [1.0.3](https://github.com/hinata-koizumi/drone_avoidance_rl/compare/v1.0.2...v1.0.3) (2025-07-19)


### Bug Fixes

* resolve ROS 2 time type issue and RL agent Docker paths ([f536abe](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f536abef2a2540aa670af40cd4a725d0787d540e))

## [1.0.2](https://github.com/hinata-koizumi/drone_avoidance_rl/compare/v1.0.1...v1.0.2) (2025-07-19)


### Bug Fixes

* **docker:** correct paths in Dockerfile.bridge for repository root context ([7937bd0](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7937bd07135c4f692d64e90789d5e39032e29f87))

## [1.0.1](https://github.com/hinata-koizumi/drone_avoidance_rl/compare/v1.0.0...v1.0.1) (2025-07-19)


### Bug Fixes

* **integration:** correct Docker build context for integration tests ([0c37e2c](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/0c37e2cf44986ddbc49ba9ef4251d862ca2a39bd))

# 1.0.0 (2025-07-19)


### Bug Fixes

* add colcon dependencies for ROS 2 build tools ([a9ce481](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/a9ce4813ff35681bbde141bf2372b50bd242fafb))
* add colcon dependencies for ROS 2 build tools ([93c6d5c](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/93c6d5c8d3aee75cd1f7b24fac2ed28cb2e76603))
* add colcon dependencies for ROS 2 build tools ([a2eb307](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/a2eb307802ce9b55ee314659c94f8471b1440de5))
* add colcon dependencies for ROS 2 build tools ([5ac3347](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/5ac3347413837d99a02b94d29f0101ef1f6b7df8))
* add colcon dependencies for ROS 2 build tools ([27bc65d](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/27bc65df87f5770d311d2613da17c4b809e754a3))
* add colcon dependencies for ROS 2 build tools ([a9d1140](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/a9d1140321d11b3d0083ba283c53d37cad0ec6b3))
* add execute permission to tools/clean_workspace.sh for CI ([04c8033](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/04c80330913c6512e0b3fc6aa20d7d08ab4df346))
* Add runtime target stage to px4-simulator Dockerfile for CI compatibility ([6b41d36](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/6b41d36a95eff86dd7bbdd654eed07479d056fa7))
* Add wget and curl to base Dockerfile dependencies for Gazebo installation ([0c6f8f8](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/0c6f8f83a50b30806b7ec64a6db94bd89d43e1a6))
* Aggressively remove /usr/share/doc/* and related files at end of Docker build to prevent disk full errors in CI ([ca8cc99](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/ca8cc99b849b761294078700d148ad208b67d5ec))
* **base_image:** add sharing=locked to apt cache mounts to avoid concurrent apt locks during BuildKit parallel build ([7534c6b](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7534c6b0af70804bc4ddd5aec9297e7ff44dbbb7))
* CI/CD最適化 - Dockerfile target stage修正、キャッシュエラー解決、不要ファイル削除による容量最適化 ([461b93b](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/461b93b1ef029c4123968dfae5a161d83febe7ed))
* CI/CD統合 - 削除ファイル参照の修正、重複CI/CDファイル削除、統合CI/CDパイプライン一本化 ([01e9730](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/01e97309b824bedb1a89185a6568efe1353cb7db))
* **ci:** add Docker build test to GitHub Actions workflow ([d194e60](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/d194e605b8bd5593b7b8e46d4692ac91634cf274))
* **ci:** convert release.config.js to CommonJS format for semantic-release compatibility ([5d75dc7](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/5d75dc787e4f241e22c1ae66e872775dfb7de90b))
* **ci:** convert release.config.js to ES6 module format for semantic-release compatibility ([398f299](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/398f299671a76075df195c076e0ca6858b35db50))
* **ci:** em compatibility layer string() signature for rosidl_adapter compatibility ([ca17d69](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/ca17d696943804f9ac14e33538a0ff213eef3914))
* **ci:** ensure semantic-release runs from repo root for config discovery ([f0b1c07](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f0b1c07be6b846f7a5173724845a0b4a61172fa7))
* **ci:** install docker-compose in GitHub Actions environment ([2ecf5e9](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/2ecf5e9d37b26fd94cf2c008e4eab5f0fb7399dd))
* **ci:** keep msgs container alive to allow dependent services; move file check to healthcheck only ([f4c046b](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f4c046b0c9ee49d34907c514991b7ad3cb71d6a3))
* **ci:** override sim container entrypoint to prevent px4 startup in headless test ([edea027](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/edea027e03cfc73afa64d43e77037246972b2909))
* **ci:** remove /root/.cache from rm command to avoid device busy error with cache mount ([422ff81](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/422ff8166ee2123e4a2faf2fa632efcfb718fd10))
* **ci:** remove npm plugin from semantic-release for Python project ([80ca1f4](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/80ca1f4602c855a6683fb753a0828cc08bbbc781))
* **ci:** update model validation paths to assets/models/custom_model ([db76f69](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/db76f696b438deee91459340b09c8bf3e5b54f59))
* **ci:** use correct build context paths in tests/ci-compose.yml ([5d26230](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/5d26230e0851ae8683443f3c462a41600cbcd091))
* **ci:** use GitHub releases for docker-compose installation on Ubuntu 24.04 ([3c32ed3](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/3c32ed345ac09a7b05d30c77797c66db54289f48))
* **ci:** use inline config for semantic-release to avoid config file loading issues ([f175fb3](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f175fb32907f4487f7c5573494dff8e8223ba471))
* **ci:** use python:3.10-slim as base for rl-mini image to avoid missing local base image, and correct pip install line ([aebdbb1](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/aebdbb1e000fc980b1f5742e7c6654eafc24ee44))
* disable visualization tools (rqt_graph, rviz2) in CI environment to avoid missing package errors ([2e8cfac](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/2e8cfac3fa6f2faff176f8721629eaa24d375385))
* docker compose build context issue - ensure commands run from repository root ([496809b](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/496809ba5e2c574ffb06830e9ccc57d74c0a6acb))
* **docker:** adjust build context for GitHub Actions compatibility ([86ad17c](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/86ad17cfe0c9c3246f21f6059bd9a8db7a0c12b5))
* **docker:** correct build context and paths for interfaces/drone_msgs ([6bfa1e8](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/6bfa1e8a888c90ae76fb5e0a7aade32a1add7455))
* **docker:** correct drone_msgs path and restore message content ([e71db9f](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/e71db9f06a8ec090a687c80ffb9e615099375b0e))
* **docker:** revert to repository root context for GitHub Actions ([06718c7](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/06718c7625ca9425e19a1cce9fe4295f26210ba7))
* **docker:** revert to repository root context to resolve Docker Compose Bake path issue ([46ac75b](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/46ac75b6b0bb82501c0f13bd57adc5cb76b2537a))
* **docker:** set build context to current directory for GitHub Actions compatibility ([cc670ce](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/cc670ce510ada8bcdee9de5cb37f9080f17099fe))
* **drone_msgs:** remove invalid import statement from msg files (ROS2 msg import not needed) ([0c02008](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/0c02008cba1a16dccc9b68e09e5ed65873d24c4d))
* **drone_msgs:** use fully qualified std_msgs/Header in msg files for ROS2 compatibility ([0333460](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/0333460d98fed535726a1cf13593705ee0050af4))
* **drone-rl:** add dummy ROS message classes to allow ROS-less CI ([b4a8adf](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/b4a8adf248a5bd890a11b3e06b065094140e3689))
* implement actual tests in orchestrated CI for all repositories ([ffebbf4](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/ffebbf432c7cf20533c5f238f8e1dc649b035a68))
* **msgs:** restore corrupted package.xml and CMakeLists.txt files ([40ff0a7](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/40ff0a72a660299557aa72f3252279981bd89e9f))
* mypy type errors in test_simulation_launch.py for Python 3.13 compatibility ([1e921c8](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/1e921c8195a6bd476886fe8f6ae7831a2896a4ff))
* mypy type errors with explicit assert statements for Python 3.13 ([2799370](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/27993704feb348954848770924b227f7df2751ee))
* mypy type errors with explicit type assertions for Python 3.13 ([0553447](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/0553447968f2b9deead984ef4fe2509bb332222d))
* mypy variable redefinition and type errors with unique variable names ([a4ace37](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/a4ace37ae1630c43d758097206fabe5200b58f5f))
* Remove non-existent PX4 file copy and add build output structure debugging ([eee8545](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/eee8545ea6fc15152dbb10fac8207b958c127733))
* remove unused variables gz_world and physics_engine to resolve ruff linting errors ([237cbbe](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/237cbbe35312fa2ac2c6bccb9863e46b2cd4c756))
* replace JoinSubstitution with direct string concatenation for ROS 2 Humble compatibility ([e4e929d](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/e4e929d0f515d39fce45db772f98af7397d9caeb))
* rl-agent Dockerfile path and PYTHONPATH issues, optimize CI for manual control tests ([52ca828](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/52ca8282123536c1edcf235036168e8a48724fa7))
* rl-agent Dockerfile path and PYTHONPATH issues, optimize CI for manual control tests ([8fb6be1](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/8fb6be1e2671bb4198aeeaff040e026128c6ff3e))
* rl-agent Dockerfile path and PYTHONPATH issues, optimize CI for manual control tests ([7e0e090](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7e0e090af88e85af51671602aad3b42aa68b7a2c))
* **security:** GitHubコードスキャン設定エラーを修正 - CodeQL専用ワークフロー追加 - Trivy設定改善（severity指定・category追加） - Dependabot設定追加（GitHub Actions・Python・Docker自動更新） ([7540edb](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7540edbdc03f4dbd0f375b7560f6490a4c8c563d))
* **sim_all.launch:** build gz_args with JoinSubstitution to avoid invalid dict substitution ([f0f939e](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f0f939e2a7931f39e2881084ff7f111bbfa2b55c))
* **sim_all.launch:** use ConcatSubstitution (available in Humble) instead of JoinSubstitution ([7d8420c](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7d8420c4ff8771d6f10d57db7a30173df2a28787))
* **sim_image:** build /sim_ws with sim_launch so CI compose can source install/setup.sh ([cae9d1d](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/cae9d1d63e1dade3073aa2521b4e3f8f2c1770d1))
* **sim_image:** install launch files with real copies (no symlink-install) and retain build dir ([e890a66](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/e890a66fdd3bea774ba02159588e5f10b0c57acf))
* **sim_image:** prepend /sim_ws/install to AMENT_PREFIX_PATH and ROS_PACKAGE_PATH so sim_launch is discoverable at runtime ([c613e95](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/c613e95188e9371d419a97ab5cd0ee0cd0ebf4f4))
* **sim_launch:** install config directory correctly; update PX4 SITL build context and logs step path ([c8af838](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/c8af838f59d05165ca1b2427aa1d7f34bbf9f40b))
* **sim_launch:** set env vars with SetEnvironmentVariable to avoid dict substitution error ([58d6a29](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/58d6a29ec092e6c37ed888cb5f670ce5f7092b5c))
* Simplify rl-agent Dockerfile to single-stage build and update CI workflow ([54bccbf](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/54bccbfdb9a108cf16f610d47150a40ae81f715b))
* **state_bridge:** remove install of missing launch directory ([bbab896](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/bbab896d2f6054a4130ec8b0fd7f9efaf488821c))
* **state_bridge:** remove invalid install(TARGETS) for python node, rely on setup.py entry_points ([8beb0a2](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/8beb0a2a2c6ba68b15f9d41aacc47c4ab8fc976d))
* test and linter errors for drone_manual_control CI ([7c11ad1](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7c11ad1097f2e8a58eaee737e282edb81e1c1600))
* Update docker-compose to docker compose for Ubuntu 24.04 compatibility ([45c43a5](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/45c43a51d636648af5745eeead66488f106c1383))
* Update GitHub Actions and optimize integration tests ([b8fb590](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/b8fb590a45f4c9b1d4688098d68c1c1ce2a35d4d))
* Update integration test for local environment compatibility ([f740f47](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f740f478d18587d21f3ac6f745cc5ebcc2c4a7f1))
* Update integration tests for actual repository structure ([4db4992](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/4db49926fab28a4ad4e044b18b379b065be9d376))
* update ROS 2 Humble image references from ros-base to ros-desktop for long-term compatibility ([4f0babd](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/4f0babd5d4f2447a6d0099579eec1ea37137065c))
* update ROS 2 Humble image references to use correct ros:humble tag for long-term compatibility ([5454b57](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/5454b57e333ff05f07c063caf5b0717701651710))
* update ROS 2 Humble package references to use correct ros-humble-ros-base package name ([e15774f](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/e15774f50d9e805f9d8b1c8157369d73ddf929f4))
* update upload-artifact from v3 to v4 to resolve deprecation error ([de28951](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/de2895198dd65779a9ddf7e4f4d18e7e5e9f35e9))
* use direct string concatenation for gz_args to avoid Substitution type conflicts in ROS 2 Humble ([f370d96](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f370d965083327eac3060e53fc570da099e3f036))
* use ubuntu-22.04 for ROS 2 Humble compatibility ([1efb55d](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/1efb55da6ad394d4ca470504f6a10fb5d3df6b35))


### Features

* add Aether-SL drone specifications for manual control environment ([a52b258](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/a52b258b2af3956a66ac14fc162dd7f1108ee95b))
* Add comprehensive integration test environment ([e26595f](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/e26595faa968be8e506413c5b3c601a4f397ae70))
* Add quick test mode and main integration workflow ([8a8b227](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/8a8b2272fac5b5b225272bd07546b09aca672fcb))
* **ci:** add lightweight RL mini test (CPU) to validate RL pipeline without exhausting disk space ([1fad475](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/1fad47573ca29b577e19f73fc12b1ab0b54c846b))
* **drone-rl:** 世界水準RL環境の雛形実装・テスト・ランダム化・ROS2連携対応 ([5a2df79](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/5a2df79ee825f9a8fb26da71aaf28fdfc02233fe))
* enhance error handling and logging for world-class standards ([42b9381](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/42b93818734ccd1c5576e2184f7f4b02ec95f8e6))
* enhance manual control CI with Aether-SL drone validation ([45ce6c3](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/45ce6c3f4b726a42e3ab1c0e82ef8db21286b0a8))
* implement orchestrated CI/CD with individual and integration tests ([7435fa7](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7435fa7ac24be2671437e06e195dfe5e3a8e1a3e))
* integrate manual control stack for pre-RL validation ([043e617](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/043e617a7a5465f9e51b55882750a196a331729c))
* optimize Docker build performance ([7f84425](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/7f844256b4b2a1b661800a65989196ed769bd471))
* optimize manual_control Dockerfile with separate runtime base ([cfb9478](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/cfb9478583b66de99f3de6c5f400500ba230fe81))
* バクの修正 ([f3504c6](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/f3504c6bf99c43ad9c907105125909f52cd7c590))
* 統合最適化 - Docker BuildKit最適化、docker-compose profiles、Makefile統合、CI/CD最適化、Fast DDS QoS最適化 ([3e9ec71](https://github.com/hinata-koizumi/drone_avoidance_rl/commit/3e9ec7169e25f2e40030052f4bd15b4f97b63ee2))
