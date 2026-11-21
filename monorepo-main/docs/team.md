# RAMION team setup

Give each teammate a personal Linux login and access to the private
`ramion-cloud/monorepo` repository through their own GitHub account. The
[server runbook](server-ops.md#add-a-teammate-and-their-public-key) covers Linux
account creation. GitHub access and server login are separate grants.

After accepting the GitHub invitation, connect from your laptop. Replace
`alice` with your Linux username:

```bash
ssh alice@ramion-server-01
```

In your own server shell, with `gh` installed, sign into your GitHub account
and configure Git to use that login over HTTPS. On a headless server, follow
the device code instructions in your laptop's browser. Prefer a personal GitHub
SSH key if `gh` reports that it will save a token in plaintext.

```bash
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git --hostname github.com
gh auth status --hostname github.com
mkdir -p "$HOME/src"
git clone https://github.com/ramion-cloud/monorepo.git "$HOME/src/ramion-monorepo"
cd "$HOME/src/ramion-monorepo"
git config user.name 'Your Name'
git config user.email 'YOUR_GITHUB_EMAIL_OR_NOREPLY_ADDRESS'
```

For SSH Git access, register a key from your own account with GitHub and use
`git@github.com:ramion-cloud/monorepo.git` as the clone URL. Keep private keys
and tokens in your own account. Use your clone for branches and pull requests.
An operator packages reviewed commits for the running services. See the
[GitHub CLI login guide](https://cli.github.com/manual/gh_auth_login),
[Git integration](https://cli.github.com/manual/gh_auth_setup-git),
[development commands](../README.md#develop), and [deployment guide](deployment.md).

Keep shared notebook source in Git with cell outputs cleared. Use a separate
Python environment for each notebook workflow and record dependencies and data
requirements alongside the notebook. Transfer reviewed source files individually
when consolidating existing work. Keep credentials, environment files, local
virtual environments, captures, datasets and generated outputs on the host.

Set up notebook kernels in your own clone and use your own credentials for the
model workflows you need. Pick a free Jupyter port on the server and use the
same port in your SSH tunnel:

```bash
cd "$HOME/src/ramion-monorepo/apps/arm-workbench"
bash notebooks/setup.sh core
RAMION_JUPYTER_PORT=8889 bash notebooks/launch.sh
```

From a separate laptop terminal:

```bash
ssh -N -L 8889:127.0.0.1:8889 alice@ramion-server-01
```

Open the loopback URL printed by Jupyter. Use the
[notebook guide](../apps/arm-workbench/notebooks/README.md) for model-specific
kernels and credential setup. The account and checkout paths there describe
the original operator setup. Substitute your own paths.

Preserve the workbench's `LICENSE`, `NOTICE`, package notices and the license
for the SO-101 model and meshes. The [controller import notes](../apps/arm-workbench/IMPORT.md)
record its Machinekind provenance.

The dashboard serves the browser and proxies requests to the controller. The
controller owns serial access, camera capture and control enforcement. Its
disconnected preview serves cameras without opening a serial port. The 3D viewer
uses measured pose when available. Notebook and `soarm-lab` work use separate
environments from the deployed services.
