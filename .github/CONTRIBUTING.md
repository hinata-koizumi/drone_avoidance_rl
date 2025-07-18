# Contributing to drone_avoidance_rl

Thank you for considering contributing to this project! We welcome all contributions that help improve the platform. Please read the following guidelines to ensure a smooth contribution process.

## How to Contribute
- Fork this repository and create your branch from `main`.
- Make your changes with clear, descriptive commit messages (use [Conventional Commits](https://www.conventionalcommits.org/)).
- Ensure all tests pass locally before submitting a pull request (PR).
- Submit your PR to the `main` branch. Fill out the PR template and describe your changes clearly.

## Branching Strategy
- `main`: Stable, production-ready code. All releases are tagged from here.
- `feature/<name>`: New features. Branch from `main` and use descriptive names (e.g., `feature/rl-agent`).
- `fix/<name>`: Bug fixes. Branch from `main` and use descriptive names (e.g., `fix/sim-crash`).
- Keep branches focused and up-to-date with `main`.

## Pull Request Process
- Use the provided PR template.
- All PRs must pass CI/CD checks (build, test, lint, etc.) before review.
- At least one review is required before merging.
- Squash and merge is preferred for a clean history.

## Setting Up Your Development Environment
1. Clone the repository:
   ```sh
   git clone https://github.com/<your-username>/drone_avoidance_rl.git
   cd drone_avoidance_rl
   ```
2. Install [Docker](https://docs.docker.com/get-docker/) and [docker-compose](https://docs.docker.com/compose/install/).
3. Build and start the environment:
   ```sh
   docker-compose up --build
   ```
4. For ROS 2 development, source the appropriate setup files inside the container.
5. Run tests:
   ```sh
   docker-compose exec <service> pytest
   ```
6. See the `README.md` and `docs/` for more details. 