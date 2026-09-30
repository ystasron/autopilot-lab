# Autopilot Lab

**A browser-based self-driving car simulation in a small city.**
No build step, no dependencies, no server. Open the file and the green car starts driving.

Made by **Ron Funiestas**.

---

## What it is

Autopilot Lab is a self-driving car you can watch think. A single car drives a small
4 × 3 city grid, and the whole point of the project is that you can *see inside its head*.

Every frame, the car senses the world, decides what to do, and acts — and the interface
renders each of those three stages as a toggleable overlay: a 360° LiDAR point cloud,
a camera field of view filled with detection boxes, the lane corridor it believes it is
driving in, and the pure-pursuit path it is steering along.

It is not a toy that teleports between waypoints. The car obeys red lights, brakes for
queues, signals its turns, yields to pedestrians on crosswalks, and takes the *shortest
route* to a destination you pin anywhere on the map. The rest of the traffic drives by
the same rules, so the city behaves like a city.

Everything is drawn on a single `<canvas>` with plain 2D context calls and vanilla
JavaScript — no libraries, no bundler, no assets.

---

## Highlights

- **Live perception overlays** — LiDAR sweep, camera FOV with labelled detections,
  lane corridor, planned path, and the traffic signal it is watching.
- **Real routing** — Dijkstra over all 48 intersection states (12 nodes × 4 headings),
  no U-turns, with parking at the destination pin.
- **Rule-following traffic** — every other car routes, obeys one speed limit, brakes
  for yellow only when it can do so comfortably, and yields on unprotected left turns.
- **Driver personalities** — cautious, normal and hurried traffic, each with its own
  speed, following gap, braking comfort and green-light reaction time.
- **Pedestrians** — walkers who use the sidewalk graph and only step off the kerb when
  the light for that road is red.
- **Night mode** — headlights, brake lights, indicator blinkers and 48 street lamps.
- **Two camera modes** — *Free* for the whole city, *Follow* for a zoomed, eased
  chase view that stays locked on the car.
- **Apple-style interface** — SF Pro type scale, grouped inset lists, frosted materials,
  UISwitch-style toggles and a 44pt-minimum tap target layout.
- **Zero dependencies** — three files, ~40 KB, works offline.

---

## Quick start

Open `index.html` in any modern browser. That is the whole install.

If you prefer a local server (handy on mobile, and avoids any `file://` quirks):

```bash
python3 -m http.server 8000
```

Then visit <http://localhost:8000>.

---

## Controls

| Control | What it does |
|---|---|
| **Autopilot: on / Manual: arrow keys** | Toggles self-driving. When off, drive with the arrow keys — `↑` accelerate, `↓` brake/reverse, `←` `→` steer. |
| **Night: off / on** | Toggles the night lighting pass. |
| **Add 5 cars / Add 10 people** | Spawns more traffic (up to 31 cars, 40 pedestrians). |
| **Clear traffic / Reset car** | Removes all traffic, or re-spawns the ego car. |
| **Cancel trip** | Aborts the current destination and releases the car. |
| **Click the map** | Pins a destination (see *Click* menu), or flips a traffic light. |
| **Next turn** | Overrides the car's random turn choice: left, straight or right. |
| **Top speed** | Sets the ego car's speed limit, 30–120 km/h. Traffic is always 60. |
| **Follow / Free** | Camera mode, above the map. |

### Camera modes

- **Free** (default) — the full 4 × 3 city, exactly as before.
- **Follow** — eases toward the green car and zooms in to 1.9×, keeping the car centred.
  The camera is clamped to the city box so you never see past the edge of the world, and
  it eases rather than snapping, so switching modes feels calm rather than jarring.

Clicking the map still works in Follow mode — the screen position is converted back into
world coordinates before the destination is placed, so you pin exactly where you tapped.

---

## How it works

### The city

A 4 × 3 grid of intersections (12 total) with two-way lanes, sidewalks, crosswalks,
stop lines, kerbs, and a traffic light at every approach. Some blocks are parks. The
whole city is rendered once into an offscreen canvas at load and blitted each frame, so
the static layer costs nothing.

Traffic lights run a **21-second cycle** — 8 s green, 1.5 s yellow, then red — with
per-intersection phase offsets so the grid does not blink in unison, plus all-red gaps.

### Sense

The car maintains a **forward gap sensor** with a 130 px lookahead. It finds the nearest
obstacle inside a cone around its heading, subtracts a time-gap based on its own
personality, and uses that to modulate its target speed. Vehicles are briefly treated as
ghosts after a long stall, so traffic does not deadlock.

The **LiDAR overlay** is a visualisation of the same idea at higher fidelity: 90 rays
sweeping a full 360° circle out to 130 px, each stopping at the first obstacle. Points are
drawn blue when they hit something static and red when they hit something moving. The
sweeping line gives it motion.

The **camera** is a 70° cone. Everything inside it and within 170 px becomes a detection
box labelled with class, range in metres and a confidence score — and the traffic signal
it is watching gets its own box.

### Plan

At every intersection the car picks a turn, either randomly, from the *Next turn* menu,
or by continuing the route it is already committed to. It then follows a quadratic Bézier
from its stop line through the corner to the next lane.

Speed is limited by whatever matters most at that moment: the remaining route distance,
a red or yellow light (comfortably or not, depending on its braking threshold), the
corner it is about to take, a queue, its own indicator, or a pedestrian. It only enters
an intersection when the exit is not blocked.

### Act

Steering is **pure pursuit**: it aims at a look-ahead point a few points down its own
path, scaled by speed, and turns toward it. This is the classic Stanley/pure-pursuit
family of controllers — simple, stable, and easy to reason about.

If the car sits still for too long it marks nearby vehicles as ghosts and creeps forward,
so traffic jams resolve themselves instead of deadlocking.

### Destinations and routing

Click anywhere on the map to drop a destination pin. The car runs **Dijkstra over the
12 × 4 state graph** — each intersection combined with each heading, so no U-turns are
ever planned — and follows the shortest route. Vertical links are 190 px, horizontal
links 230 px.

The pin snaps at least 72 px away from each intersection so the car parks at a sensible
spot on the road rather than on top of a junction. On arrival the car parks, the pin
disappears, and it waits there until you pin somewhere new.

### Other drivers

Traffic cars are rule-following "humans", not scripted paths. Each one has a destination,
takes the shortest route to it, obeys a single speed limit, and:

- stops at red, and at yellow if it can brake comfortably;
- yields to oncoming traffic on unprotected left turns;
- yields to pedestrians on the crosswalk;
- does not enter an intersection if the exit is blocked.

Each driver gets one of three personalities, which changes its speed multiplier, following
gap, braking threshold and reaction delay at green:

| Personality | Speed | Following gap | Braking | Reaction |
|---|---|---|---|---|
| Cautious | ×0.85 | wide (×1.3) | gentle (110) | slow (1.2 s) |
| Normal | ×1.00 | medium (×0.9) | medium (150) | medium (0.7 s) |
| Hurried | ×1.12 | tight (×0.6) | firm (220) | fast (0.4 s) |

### Pedestrians

Walkers move along a sidewalk graph — four corners per intersection, connected around
the block and across streets. They only start a crossing when the light for that road is
red, which is the real pedestrian dilemma: they can only cross against the flow.

---

## Telemetry

The nine live metrics under the map:

| Metric | Meaning |
|---|---|
| **km/h** | Current speed of the ego car. |
| **clear ahead (px)** | Forward gap to the nearest obstacle; `–` when the road ahead is clear. |
| **signal** | The state of the light the car is watching, tinted green / yellow / red. |
| **trip** | `en route` while driving to a pin, `arrived` once parked. |
| **detections** | Objects currently inside the camera field of view. |
| **collisions** | Times the ego car touched anything. |
| **road exits** | Times the ego car left the road surface. |
| **cars / people** | Current traffic and pedestrian counts. |

Collision and road-exit counters are the honest scoreboard — a good run keeps both at `0`.

---

## Files

| File | Purpose |
|---|---|
| `index.html` | The whole page: structure, controls, copy. |
| `style.css` | The entire interface — design tokens, layout, controls, motion. |
| `sim.js` | City, traffic, pedestrians, driving logic and canvas rendering. |

Edit `sim.js` to change how the car drives. The UI is fully decoupled from it: the page
only depends on element IDs and `data-k` attributes, so you can restyle or restructure
the interface without touching a line of simulation code.

---

## Design notes

The interface follows Apple's Human Interface Guidelines rather than a generic dashboard:

- **System-first typography** — SF Pro on Apple platforms, falling back to the platform
  system font. Hierarchy comes from size and weight, not from colour.
- **Grouped inset surfaces** — frosted cards with hairline separators instead of hard
  borders, on a true-black background.
- **One accent** — a single system blue, reserved for the primary action and focus rings.
- **Native-feeling controls** — the sensor toggles are UISwitches, the camera selector is
  a segmented control, and the speed slider uses the platform's native slider.
- **Comfortable targets** — 44pt minimum tap targets, safe-area insets respected on
  notched displays, and `prefers-reduced-motion` honoured.

---

## Browser support

Any current version of Safari, Chrome, Edge or Firefox. `backdrop-filter`,
`env(safe-area-inset-*)` and `:focus-visible` are used where supported and degrade
gracefully elsewhere.

---

## Deploy on GitHub Pages

Push the files to the `main` branch, then go to **Settings → Pages** and choose the
`main` branch with the `/ (root)` folder. There is nothing to build.

---

## License

MIT — see [LICENSE](LICENSE).

---

## Credits

Built by **Ron Funiestas**. City layout, traffic behaviour, perception overlays and the
entire interface are original to this project.
