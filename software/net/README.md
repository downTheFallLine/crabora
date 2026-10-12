# net

Network setup for the robot. One script, run on the Pi.

## hotspot.sh — be its own WiFi access point

```bash
cd ~/crabora/software/net
./hotspot.sh            # set it up as a fallback
./hotspot.sh --status   # what is it doing right now
./hotspot.sh --now      # switch to AP immediately
./hotspot.sh --off      # stop being an AP
```

### Fallback, not a switch

The Pi has **one radio**, and concurrent AP+client on `brcmfmac` is
flaky — drops, wedged interfaces, poor throughput. So it is one or the
other, and the only interesting question is which one, when.

The answer here is priority, not a toggle. Known networks keep a high
`autoconnect-priority`; the AP profile gets a low one. An AP always
succeeds, where an infrastructure profile only succeeds if that network
is in range — so without the priority gap the AP would win at every
boot. With it:

- **at home** → joins the house network, as before
- **in the garden** → no house network, so it becomes `crabora` and you
  connect straight to it at `http://10.42.0.1:5000/`

Nothing to remember and nothing to toggle, which is the point.

### While it is an AP, the robot has no internet

No `git pull`, no `apt`. That is the one radio, not a flaw in the setup.
Pull changes before you walk away from the house network.

### Two things it does not fix

**macOS Local Network permission.** Connecting straight to the Pi still
puts you on a LAN, so Chrome and Safari still need that permission, and
the microphone still needs a secure context. The SSH tunnel remains the
answer — just pointed at the AP address:

```bash
ssh -N -L 5050:localhost:5000 ubuntu@10.42.0.1
```

**Authentication.** There is none. Anyone who joins the AP reaches the
agentix UI, and the ARM button is the only thing between them and the
servos. Hence WPA2 with a real passphrase, and why the script refuses to
make an open network. Once the robot is routinely off the house network,
nginx with TLS is worth doing properly — it would also retire the tunnel.

### The NetworkManager step is the risky one

Ubuntu Server defaults to netplan with `systemd-networkd`, not
NetworkManager. If the script finds NM missing it offers to install it
and hand it `wlan0` — **which briefly drops the network, and over SSH
that means dropping the session you are typing into.** Do that part with
ethernet plugged in, or sitting in front of the robot with a keyboard.
The script warns and asks before touching anything.

### Not tested on hardware

Written on a Mac with no Pi attached. The logic was exercised against a
stubbed `nmcli` — profile creation, the priority rules, `--status`,
`--off`, and the passphrase checks — but no real radio has run it.
