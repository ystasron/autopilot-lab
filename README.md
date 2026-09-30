# Autopilot Lab

A browser-based self-driving car simulation in a small city. No build step, no dependencies.

## Run it
Open `index.html` in a browser, or serve the folder: `python3 -m http.server`.

## Deploy on GitHub Pages
Push the files, then go to Settings > Pages and choose the `main` branch, root folder.

## What it does
- A 4x3 grid of streets with two-way lanes, sidewalks, crosswalks and a traffic light at every intersection.
- **Sense:** 9 lidar-style rays detect cars and pedestrians.
- **Plan:** at each intersection the car picks a turn (or use the "Next turn" menu) and follows the lane through it.
- **Act:** pure-pursuit steering; stops at red lights, queues behind cars, yields to people on crosswalks.
- Pedestrians walk the sidewalks and cross only when the light for that road is red.

## Controls
Add cars and people, click an intersection to flip its light, or turn autopilot off and drive with the arrow keys.

## Files
`index.html` page, `style.css` styling, `sim.js` city, traffic and driving logic.

## Realism
Bicycle-model steering, turn signals, brake lights, time-gap car following, left-turn yielding, stop lines, curbs, parks, and a night mode with headlights and street lamps.

## Destinations
Click anywhere on the map to pin a destination. The car plans the shortest route (Dijkstra over intersections, no U-turns), drives it, parks at the pin, removes the pin and waits there until you pin a new destination.

## Other drivers
Traffic cars are rule-following "humans": each has a destination and takes the shortest route, obeys one speed limit, stops for red and for yellow when they can brake comfortably, yields to oncoming traffic on left turns, yields to pedestrians, and does not enter an intersection if the exit is blocked. Personalities (cautious, normal, hurried) change their speed, following gap, braking comfort and reaction time at green lights.

## Sensor overlays
Toggle Planned path (pure-pursuit path and look-ahead point), Lane corridor, LiDAR (360 degree point cloud, blue = static, red = moving), Camera FOV (70 degree cone) and Detections (boxes with class, range in metres and confidence, plus the signal ahead).
