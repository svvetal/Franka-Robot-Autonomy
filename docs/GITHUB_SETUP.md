# Publish on GitHub (Ubuntu)

Suggested GitHub repository name: `Franka-Robot-Autonomy`

Suggested repository description: `Collision-aware motion planning for the Franka Panda: kinematics, SAT collision detection, RRT, PRM and MuJoCo simulation.`

Suggested GitHub topics: `robotics`, `franka-panda`, `motion-planning`, `mujoco`, `rrt`, `prm`, `collision-detection`, `inverse-kinematics`, `python`

> **Use a private repository until you have confirmed that sharing the course solutions and templates is permitted.**

## Option A: GitHub website + Git command line

1. Open https://github.com/new. Create an **empty** `Franka-Robot-Autonomy` repository under your GitHub account. Choose **Private** initially. Do not prepopulate it with a README, license or `.gitignore` because the package already has files.
2. Open a terminal in the **extracted `Franka-Robot-Autonomy` directory**.
3. Run:

```bash
git init
git add .
git commit -m "Initial commit: Franka robot kinematics and motion planning"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/Franka-Robot-Autonomy.git
git push -u origin main
```

Replace `YOUR_USERNAME` with your GitHub handle.

GitHub does **not** allow your account password for HTTPS Git push. Configure GitHub CLI authentication (`gh auth login`), Git Credential Manager, a personal access token, or SSH authentication first.

## Option B: GitHub CLI (when installed)

```bash
gh auth login
git init
git add .
git commit -m "Initial commit: Franka robot kinematics and motion planning"
git branch -M main
gh repo create Franka-Robot-Autonomy --private --source=. --remote=origin --push
```

If you decide to make the repository public after checking course policy, use the repository's **Settings → General → Danger Zone → Change repository visibility** (or GitHub CLI equivalents).

## After publishing

1. Add a repository description and topics from the top of this document.
2. Add the RRT and PRM videos as README links (already present).
3. Optionally create a short GIF from a simulation you recorded yourself and link it at the top of the README.
4. Pin the repository to your GitHub profile and use the project-card copy in `docs/PORTFOLIO_COPY.md`.
