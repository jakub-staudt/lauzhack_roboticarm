# RAMION server ops

Host baseline recorded: 2026-11-03. App development and deployment are now managed in the private monorepo; see [deployment](deployment.md) and [control architecture](control-and-notebooks.md). Run Bash blocks **on the server as `kwiscion`** unless labelled laptop.
SSH, Tailscale, GPU and camera-preview autostart were verified after reboot on 2026-11-03.
Firmware power-loss recovery and physical arm motion remain unverified.

| Item | Verified / known state |
|---|---|
| Host / administrator | `ramion-server-01` / `kwiscion` |
| OS | Ubuntu Server 24.04.5 LTS, amd64 |
| GPU | RTX 3060, 12 GB; NVIDIA 595.91.07; `nvidia-smi` works |
| Driver packages | `nvidia-headless-no-dkms-595-server-open`, `nvidia-utils-595-server` |
| Network | Ethernet; Tailscale installed and verified by operator |
| Storage | Ubuntu on Kingston A400 480 GB; WD 500 GB left unchanged; expected 2 TB disk not detected |
| RAMION | Tailscale-only dashboard http://100.85.56.14:8780/, controller API port 8765, viewer port 8766 |
| Historical source | Local so-arm101 commit `24abbb6`; 131 hardware-free tests, lint and type checking passed |
| Cameras | Overhead: Innomaker 720P; wrist: Innomaker 1080p; both live, stable by-id paths |
| Historical arm check | /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219949-if00; six servo IDs verified; live 20 Hz feedback, no faults |
| Pending | Team logins; supervised physical motion/STOP acceptance |

## Connect / develop

On a laptop joined to the team's Tailscale network:

```bash
ssh kwiscion@ramion-server-01
```

If MagicDNS does not resolve that name, run `tailscale ip -4` on the server and use that IP.
In VS Code install **Remote - SSH**, then **Remote-SSH: Connect to Host**.
Each developer connects as their own Linux user. See [team setup](team.md) for
personal GitHub access, clone commands and notebook setup. The command above
uses the existing administrator account for maintenance.

## Development and deployment access

The controller currently runs as `kwiscion` with `video` and `dialout` access.
The dashboard runs as `kwikiel`. Both existing accounts have `sudo`. Normal
developer onboarding grants SSH access to a personal account. Grant device or
administrator access separately when the person's role requires it.

The [deployment guide](deployment.md) records the current service locations
and a proposed shared runtime under `/srv/ramion`. Personal development stays
under each teammate's home directory in either layout.

## Add a teammate and their public key

Replace `alice` and the entire example public-key string. Both Ed25519 and RSA public keys are accepted.
Run the complete block once per **new** user. It refuses existing accounts and validates the key
before creating anything. Only the public `.pub` key goes here; private keys stay on laptops.

```bash
(
set -euo pipefail
new_user='alice'
public_key='ssh-ed25519 REPLACE_WITH_FULL_PUBLIC_KEY alice@laptop'

[[ "$new_user" =~ ^[a-z][a-z0-9_-]{0,31}$ ]] || { echo 'Invalid username.' >&2; exit 1; }
if getent passwd "$new_user" >/dev/null; then
  echo 'Account already exists; refusing to overwrite its keys.' >&2
  exit 1
fi
[[ "$public_key" != *$'\n'* && "$public_key" != *$'\r'* ]] || { echo 'Public key must be one line.' >&2; exit 1; }
case "$public_key" in
  ssh-ed25519\ *|ssh-rsa\ *) ;;
  *) echo 'Expected an Ed25519 or RSA public key.' >&2; exit 1 ;;
esac

key_file="$(mktemp)"
trap 'rm -f "$key_file"' EXIT
printf '%s\n' "$public_key" > "$key_file"
ssh-keygen -lf "$key_file"

sudo adduser --disabled-password --gecos '' "$new_user"
sudo chmod 750 "/home/$new_user"
sudo install -d -m 700 -o "$new_user" -g "$new_user" "/home/$new_user/.ssh"
sudo install -m 600 -o "$new_user" -g "$new_user" "$key_file" "/home/$new_user/.ssh/authorized_keys"
sudo /usr/sbin/sshd -t
printf 'Created %s. Test from their laptop: ssh %s@ramion-server-01\n' "$new_user" "$new_user"
)
```

The user has key-based login, no password login, and no sudo access. No SSH restart is needed.
Authorize the teammate in Tailscale too, then test from **their** laptop before closing your admin session.
If creation partially fails, inspect the account before retrying; the block will refuse to overwrite it.

Only if this person should administer the machine, set a password for sudo and grant membership:

```bash
sudo passwd alice
sudo usermod -aG sudo alice
```

Log in again for group membership to apply. Setting a password also enables password login
wherever SSH permits it.

## Boot baseline (run once)

Enables remote-access services, prevents suspend, retains daily security updates without automatic
machine reboots, and keeps up to 500 MB of persistent journal logs.

```bash
(
set -euo pipefail
sudo /usr/sbin/sshd -t
sudo systemctl enable --now ssh tailscaled
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo timedatectl set-timezone Europe/Warsaw
sudo apt install unattended-upgrades

printf '%s\n' \
  'APT::Periodic::Update-Package-Lists "1";' \
  'APT::Periodic::Unattended-Upgrade "1";' \
  'Unattended-Upgrade::Automatic-Reboot "false";' \
  | sudo tee /etc/apt/apt.conf.d/99-ramion-ops >/dev/null
sudo systemctl enable --now apt-daily.timer apt-daily-upgrade.timer

sudo install -d -m 755 /etc/systemd/journald.conf.d
printf '%s\n' '[Journal]' 'Storage=persistent' 'SystemMaxUse=500M' \
  | sudo tee /etc/systemd/journald.conf.d/ramion.conf >/dev/null
sudo systemctl restart systemd-journald
sudo journalctl --flush

systemctl is-enabled ssh tailscaled
systemctl is-active ssh tailscaled
apt-config dump | grep -E 'APT::Periodic::(Update-Package-Lists|Unattended-Upgrade)|Unattended-Upgrade::Automatic-Reboot'
)
```

Check locally once: firmware boots the Kingston disk; remove the installer USB; no unattended-boot
password/keyboard prompt; Ethernet reconnects. Set firmware **Restore after AC power loss = Power On**
if automatic recovery after an outage is wanted. These firmware settings are not yet verified.
Use a router DHCP reservation if a stable LAN IP is needed; no router port forwarding is required.

In the Tailscale admin console, review this server's key expiry. For unattended availability, optionally
disable key expiry for this trusted server; revoke it if compromised or retired. Enabling `tailscaled`
uses the existing login; do not routinely log out or force reauthentication.

Security updates can still restart affected services on Ubuntu 24.04; disabling automatic reboot
does not guarantee uninterrupted jobs.

## Updates / planned reboot

Notify the team, save/checkpoint work, and stop robot motion and experiments **before** this block.
Review logged-in users and GPU processes. Run while local recovery is available for the first test.

```bash
who
nvidia-smi
sudo apt update
sudo apt upgrade
sudo shutdown -r +1 'RAMION maintenance: save work now.'
```

Cancel a pending scheduled reboot with `sudo shutdown -c`.
After about a minute, reconnect from the laptop with `ssh kwiscion@ramion-server-01`, then:

```bash
uptime -s
systemctl is-enabled ssh tailscaled
systemctl is-active ssh tailscaled
systemctl --failed --no-pager
tailscale status
nvidia-smi
df -h /
```

If unreachable: use the local console and check `ip -br address`,
`sudo journalctl -b -u ssh -u tailscaled --no-pager -n 80`, and
`sudo journalctl -b -1 -p warning --no-pager -n 80`.
A changed SSH host key needs verification at the server, not blind removal of the warning.

## Jobs / cameras

`tmux` survives SSH disconnection, **not reboot**. Training needs checkpoints; robot movement and
experiments require an explicit start after reboot. Use dedicated systemd services for persistent
camera/application servers once configured, rather than terminal sessions.

Camera preview is deployed under `/home/kwiscion/apps/ramion-workbench/src` and starts at boot.
The arm-control service is manual and never auto-restarts. Full setup/mode instructions:
[Linux workbench ops](../apps/arm-workbench/ops/linux/README.md).

```bash
systemctl --user status ramion-workbench-preview.service --no-pager
systemctl --user restart ramion-workbench-preview.service
journalctl --user -u ramion-workbench-preview.service -n 60 --no-pager
curl -fsS http://100.85.56.14:8765/api/state
```

Preview never opens the robot port. Control mode starts only after identifying the adapter and an
explicit supervised switch; startup enables torque. Stop experiments before service changes.
One app process owns both cameras; do not start uStreamer on the same nodes.

References: [Ubuntu SSH](https://ubuntu.com/server/docs/how-to/security/openssh-server/),
[Ubuntu automatic updates](https://ubuntu.com/server/docs/how-to/software/automatic-updates/),
[Tailscale Linux](https://tailscale.com/docs/install/linux),
[systemd journal](https://www.freedesktop.org/software/systemd/man/255/journald.conf.html).

