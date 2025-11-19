# Frontend Integration Guide for ROS2 Robot

**Target Audience:** Frontend developers building the web UI for the robot

**Last Updated:** November 19, 2025

---

## Quick Start

### 1. Connect to the Robot

```javascript
// Connect to rosbridge WebSocket
const ros = new ROSLIB.Ros({
  url: 'ws://192.168.1.100:9090'  // Replace with your robot's IP
});

ros.on('connection', function() {
  console.log('Connected to ROS!');
});

ros.on('error', function(error) {
  console.log('Error connecting to ROS: ', error);
});

ros.on('close', function() {
  console.log('Connection to ROS closed.');
});
```

### 2. Video Stream

```html
<!-- Display robot camera feed -->
<img src="http://192.168.1.100:8080/stream?topic=/ascamera/camera_publisher/rgb0/image" />
```

---

## Topics (Subscribe to Get Data)

Topics are **data streams** that you subscribe to. Think of them as event streams that continuously push data to your UI.

### `/available_maps` 
**What it is:** List of saved maps on the robot  
**Type:** `std_msgs/String`  
**Data Format:** JSON string: `["map_01", "map_02", "kitchen_map"]`  
**Update Frequency:** ~1 Hz (once per second)  
**Use it for:** Populating a dropdown/list for map selection  

```javascript
const mapsListener = new ROSLIB.Topic({
  ros: ros,
  name: '/available_maps',
  messageType: 'std_msgs/String'
});

mapsListener.subscribe(function(message) {
  const maps = JSON.parse(message.data);
  console.log('Available maps:', maps);
  // Update your UI dropdown with this array
  updateMapDropdown(maps);
});
```

---

### `/robot/state`
**What it is:** Current operational state of the robot  
**Type:** `std_msgs/String`  
**Data Format:** String with values: `"idle"`, `"slam"`, `"localization"`, `"navigating"`  
**Update Frequency:** ~1 Hz  
**Use it for:** 
- Show state badge in UI
- Enable/disable buttons (e.g., disable "Start SLAM" when already in SLAM mode)
- Show appropriate UI panels (mapping UI vs navigation UI)

```javascript
const stateListener = new ROSLIB.Topic({
  ros: ros,
  name: '/robot/state',
  messageType: 'std_msgs/String'
});

stateListener.subscribe(function(message) {
  const state = message.data;
  console.log('Robot state:', state);
  
  // Update UI based on state
  if (state === 'slam') {
    showMappingControls();
  } else if (state === 'localization') {
    showNavigationControls();
  }
});
```

---

### `/mode_manager/status`
**What it is:** Detailed status of the mode manager (more info than `/robot/state`)  
**Type:** `std_msgs/String`  
**Data Format:** JSON string with structure:
```json
{
  "mode": "localization",           // Current mode: "slam", "localization", "idle"
  "map": "map_01",                  // Currently active map name
  "slam_running": false,            // Is SLAM process running?
  "localization_running": true,     // Is Nav2/localization running?
  "last_operation": "loaded_map:map_01",  // Last successful operation
  "last_error": "",                 // Last error message (empty if no error)
  "use_filtered_scan": true         // Is laser filter enabled?
}
```
**Update Frequency:** ~1 Hz  
**Use it for:**
- Detailed debugging
- Show last operation/error in status bar
- Confirm mode transitions were successful

```javascript
const statusListener = new ROSLIB.Topic({
  ros: ros,
  name: '/mode_manager/status',
  messageType: 'std_msgs/String'
});

statusListener.subscribe(function(message) {
  const status = JSON.parse(message.data);
  console.log('Mode manager status:', status);
  
  if (status.last_error) {
    showError(status.last_error);
  }
  
  updateStatusBar({
    mode: status.mode,
    map: status.map,
    lastOp: status.last_operation
  });
});
```

---

### `/connected_clients`
**What it is:** List of clients connected to the robot  
**Type:** `std_msgs/String`  
**Data Format:** JSON array: `["webui", "mobile_app"]`  
**Update Frequency:** ~1 Hz  
**Use it for:** Show how many devices are connected (optional, nice-to-have)

```javascript
const clientsListener = new ROSLIB.Topic({
  ros: ros,
  name: '/connected_clients',
  messageType: 'std_msgs/String'
});

clientsListener.subscribe(function(message) {
  const clients = JSON.parse(message.data);
  document.getElementById('clientCount').textContent = clients.length;
});
```

---

### `/client_count`
**What it is:** Simple count of connected clients  
**Type:** `std_msgs/Int32`  
**Data Format:** Integer (e.g., `2`)  
**Update Frequency:** ~1 Hz  
**Use it for:** Quick connection status indicator

```javascript
const countListener = new ROSLIB.Topic({
  ros: ros,
  name: '/client_count',
  messageType: 'std_msgs/Int32'
});

countListener.subscribe(function(message) {
  console.log('Connected clients:', message.data);
});
```

---

### `/navigate_to_pose/status`
**What it is:** Status of active navigation goals  
**Type:** `action_msgs/GoalStatusArray`  
**Data Format:** Array of goal statuses (empty if no active navigation)  
**Update Frequency:** ~1 Hz (when Nav2 is running)  
**Use it for:**
- Show "Navigating..." indicator when goals are active
- Enable/disable "Cancel Navigation" button

```javascript
const navStatusListener = new ROSLIB.Topic({
  ros: ros,
  name: '/navigate_to_pose/status',
  messageType: 'action_msgs/GoalStatusArray'
});

navStatusListener.subscribe(function(message) {
  const hasActiveGoals = message.status_list.length > 0;
  
  if (hasActiveGoals) {
    showNavigatingIndicator();
    enableCancelButton();
  } else {
    hideNavigatingIndicator();
    disableCancelButton();
  }
});
```

---

### `/map` (Subscribe with Transient Local QoS)
**What it is:** The actual map data (occupancy grid)  
**Type:** `nav_msgs/OccupancyGrid`  
**Data Format:** Grid with metadata:
- `info.width`: map width in cells
- `info.height`: map height in cells
- `info.resolution`: meters per cell (e.g., 0.05 = 5cm per cell)
- `info.origin`: map origin pose (where (0,0) is in world coordinates)
- `data`: array of occupancy values (0=free, 100=occupied, -1=unknown)

**Update Frequency:** Only when map changes (SLAM updates ~1 Hz during mapping)  
**Use it for:** Rendering the 2D map visualization

**IMPORTANT:** Use **Transient Local QoS** to receive the last published map immediately on connect.

```javascript
const mapListener = new ROSLIB.Topic({
  ros: ros,
  name: '/map',
  messageType: 'nav_msgs/OccupancyGrid',
  // Transient Local QoS (important!)
  queueSize: 1,
  qos: {
    durability: 'transient_local',
    reliability: 'reliable'
  }
});

mapListener.subscribe(function(message) {
  console.log('Map received:', message.info.width, 'x', message.info.height);
  
  // Render map on canvas
  renderMap({
    width: message.info.width,
    height: message.info.height,
    resolution: message.info.resolution,
    origin: message.info.origin,
    data: message.data
  });
});
```

**How to render the map:**
```javascript
function renderMap(map) {
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  
  canvas.width = map.width;
  canvas.height = map.height;
  
  const imageData = ctx.createImageData(map.width, map.height);
  
  for (let i = 0; i < map.data.length; i++) {
    const value = map.data[i];
    let color;
    
    if (value === -1) {
      color = [128, 128, 128]; // Unknown = gray
    } else if (value === 0) {
      color = [255, 255, 255]; // Free = white
    } else {
      color = [0, 0, 0]; // Occupied = black
    }
    
    imageData.data[i * 4] = color[0];     // R
    imageData.data[i * 4 + 1] = color[1]; // G
    imageData.data[i * 4 + 2] = color[2]; // B
    imageData.data[i * 4 + 3] = 255;      // A
  }
  
  ctx.putImageData(imageData, 0, 0);
}
```

---

### `/scan` or `/scan_raw`
**What it is:** Real-time lidar laser scan data  
**Type:** `sensor_msgs/LaserScan`  
**Data Format:** Array of distance measurements in meters + angles  
**Update Frequency:** ~10-20 Hz (fast!)  
**Use it for:** 
- Live lidar visualization on map
- Show robot's perception range
- Debug sensor issues

```javascript
const scanListener = new ROSLIB.Topic({
  ros: ros,
  name: '/scan',  // Use '/scan_raw' if you want unfiltered data
  messageType: 'sensor_msgs/LaserScan'
});

scanListener.subscribe(function(message) {
  // message.ranges: array of distances [0.5, 1.2, 3.4, ...]
  // message.angle_min: start angle (radians)
  // message.angle_max: end angle (radians)
  // message.angle_increment: angle between measurements
  
  drawLidarScan(message);
});
```

---

### `/odom`
**What it is:** Robot's position estimate from wheel encoders + IMU  
**Type:** `nav_msgs/Odometry`  
**Data Format:** Pose (position + orientation) + velocity  
**Update Frequency:** ~50-100 Hz (very fast!)  
**Use it for:**
- Show robot position marker on map
- Show velocity/speed indicator
- Display heading arrow

```javascript
const odomListener = new ROSLIB.Topic({
  ros: ros,
  name: '/odom',
  messageType: 'nav_msgs/Odometry',
  throttle_rate: 100  // Throttle to 10 Hz for UI (optional)
});

odomListener.subscribe(function(message) {
  const x = message.pose.pose.position.x;
  const y = message.pose.pose.position.y;
  const orientation = message.pose.pose.orientation;
  
  // Update robot marker on map
  updateRobotMarker(x, y, orientation);
});
```

---

## Services (Call to Perform Actions)

Services are **remote procedure calls**. You call them once, they do something, and return a response.

### `/get_mode`
**What it does:** Gets current mode and active map  
**Type:** `std_srvs/Trigger`  
**Request:** Empty `{}`  
**Response:** 
```json
{
  "success": true,
  "message": "{\"mode\":\"localization\",\"map\":\"map_01\"}"
}
```
**Use it for:** One-time status check (prefer subscribing to `/mode_manager/status` for continuous updates)

```javascript
const getModeClient = new ROSLIB.Service({
  ros: ros,
  name: '/get_mode',
  serviceType: 'std_srvs/Trigger'
});

const request = new ROSLIB.ServiceRequest({});

getModeClient.callService(request, function(result) {
  const status = JSON.parse(result.message);
  console.log('Current mode:', status.mode);
  console.log('Active map:', status.map);
});
```

---

### `/set_mode`
**What it does:** Switches between SLAM and localization modes  
**Type:** `interfaces/SetString`  
**Request:** `{ data: "slam" }` or `{ data: "localization" }`  
**Response:** 
```json
{
  "success": true,
  "message": "{\"mode\":\"slam\",\"map\":\"\"}"
}
```
**Use it for:** Mode switching buttons in UI

```javascript
const setModeClient = new ROSLIB.Service({
  ros: ros,
  name: '/set_mode',
  serviceType: 'interfaces/SetString'
});

// Switch to SLAM mode
function startSLAMMode() {
  const request = new ROSLIB.ServiceRequest({
    data: 'slam'
  });
  
  setModeClient.callService(request, function(result) {
    if (result.success) {
      showNotification('SLAM mode started');
    } else {
      showError('Failed to start SLAM: ' + result.message);
    }
  });
}

// Switch to localization mode
function startLocalizationMode() {
  const request = new ROSLIB.ServiceRequest({
    data: 'localization'
  });
  
  setModeClient.callService(request, function(result) {
    if (result.success) {
      showNotification('Localization mode started');
    } else {
      showError('Failed to start localization: ' + result.message);
    }
  });
}
```

---

### `/start_slam`
**What it does:** Starts SLAM (mapping) mode  
**Type:** `std_srvs/Trigger`  
**Request:** Empty `{}`  
**Response:** `{ success: true, message: "slam started" }`  
**Use it for:** "Start Mapping" button

```javascript
const startSlamClient = new ROSLIB.Service({
  ros: ros,
  name: '/start_slam',
  serviceType: 'std_srvs/Trigger'
});

function startMapping() {
  const request = new ROSLIB.ServiceRequest({});
  
  startSlamClient.callService(request, function(result) {
    if (result.success) {
      showNotification('Mapping started! Drive the robot to explore.');
    }
  });
}
```

---

### `/stop_slam_and_save`
**What it does:** Saves the current map and stops SLAM  
**Type:** `interfaces/SetString`  
**Request:** `{ data: "my_map_name" }` (optional, auto-generates name if empty)  
**Response:** 
```json
{
  "success": true,
  "message": "my_map_name"  // The actual saved map name
}
```
**Use it for:** "Save Map" button in mapping interface

```javascript
const stopSlamClient = new ROSLIB.Service({
  ros: ros,
  name: '/stop_slam_and_save',
  serviceType: 'interfaces/SetString'
});

function saveMap(mapName) {
  const request = new ROSLIB.ServiceRequest({
    data: mapName || ''  // Empty = auto-generate timestamp name
  });
  
  stopSlamClient.callService(request, function(result) {
    if (result.success) {
      showNotification('Map saved as: ' + result.message);
      // Map will appear in /available_maps topic shortly
    } else {
      showError('Failed to save map: ' + result.message);
    }
  });
}
```

**UI Example:**
```html
<input type="text" id="mapNameInput" placeholder="Enter map name" />
<button onclick="saveMap(document.getElementById('mapNameInput').value)">
  Save Map
</button>
```

---

### `/load_map`
**What it does:** Loads a saved map and starts localization  
**Type:** `interfaces/SetString`  
**Request:** `{ data: "map_01" }`  
**Response:** 
```json
{
  "success": true,
  "message": "map_01"
}
```
**Use it for:** Map selection dropdown

```javascript
const loadMapClient = new ROSLIB.Service({
  ros: ros,
  name: '/load_map',
  serviceType: 'interfaces/SetString'
});

function loadMap(mapName) {
  const request = new ROSLIB.ServiceRequest({
    data: mapName
  });
  
  loadMapClient.callService(request, function(result) {
    if (result.success) {
      showNotification('Loaded map: ' + mapName);
    } else {
      showError('Failed to load map: ' + result.message);
    }
  });
}
```

**UI Example:**
```html
<select id="mapSelector" onchange="loadMap(this.value)">
  <!-- Populated from /available_maps topic -->
</select>
```

---

### `/list_maps`
**What it does:** Gets list of available maps (one-time call)  
**Type:** `std_srvs/Trigger`  
**Request:** Empty `{}`  
**Response:** 
```json
{
  "success": true,
  "message": "[\"map_01\",\"map_02\",\"kitchen\"]"
}
```
**Use it for:** Initial map list population (prefer subscribing to `/available_maps` topic)

```javascript
const listMapsClient = new ROSLIB.Service({
  ros: ros,
  name: '/list_maps',
  serviceType: 'std_srvs/Trigger'
});

function getMapList() {
  const request = new ROSLIB.ServiceRequest({});
  
  listMapsClient.callService(request, function(result) {
    const maps = JSON.parse(result.message);
    console.log('Available maps:', maps);
    populateMapDropdown(maps);
  });
}
```

---

### `/system/map/pois`
**What it does:** Gets Points of Interest (POIs) for a specific map  
**Type:** `interfaces/SetString`  
**Request:** `{ data: "map_01" }` (or empty to use current map)  
**Response:** 
```json
{
  "success": true,
  "message": "{\"map\":\"map_01\",\"pois\":[{\"name\":\"Dock\",\"x\":120,\"y\":220},{\"name\":\"Kitchen\",\"x\":350,\"y\":180}]}"
}
```
**Use it for:** 
- Display POI markers on map
- Create "Go to POI" navigation shortcuts
- Show saved locations

```javascript
const getPoisClient = new ROSLIB.Service({
  ros: ros,
  name: '/system/map/pois',
  serviceType: 'interfaces/SetString'
});

function loadPOIs(mapName) {
  const request = new ROSLIB.ServiceRequest({
    data: mapName || ''  // Empty = use current map
  });
  
  getPoisClient.callService(request, function(result) {
    const data = JSON.parse(result.message);
    const pois = data.pois;
    
    // Draw POI markers on map
    pois.forEach(poi => {
      drawPOIMarker(poi.name, poi.x, poi.y);
    });
  });
}
```

**POI File Format (FYI - stored on robot):**
```json
{
  "pois": [
    {"name": "Dock", "x": 120, "y": 220},
    {"name": "Kitchen", "x": 350, "y": 180}
  ]
}
```
Save as `<map_name>.pois.json` in the maps directory.

---

### `/system/map/info`
**What it does:** Gets complete map information (metadata + POIs)  
**Type:** `interfaces/SetString`  
**Request:** `{ data: "map_01" }` (or empty for current map)  
**Response:** 
```json
{
  "success": true,
  "message": "{\"map\":\"map_01\",\"meta\":{\"resolution\":0.05,\"origin\":[0,0,0]},\"pois\":[...]}"
}
```
**Use it for:** Getting all map details in one call

```javascript
const getMapInfoClient = new ROSLIB.Service({
  ros: ros,
  name: '/system/map/info',
  serviceType: 'interfaces/SetString'
});

function getMapInfo(mapName) {
  const request = new ROSLIB.ServiceRequest({
    data: mapName
  });
  
  getMapInfoClient.callService(request, function(result) {
    const info = JSON.parse(result.message);
    console.log('Map resolution:', info.meta.resolution);
    console.log('POIs:', info.pois);
  });
}
```

---

### `/system/map/select`
**What it does:** Selects active map (tells system_topics which map to track)  
**Type:** `interfaces/SetString`  
**Request:** `{ data: "map_01" }`  
**Response:** `{ success: true, message: "map_01" }`  
**Use it for:** UI map selection (complementary to `/load_map`)

**Note:** This service is mainly for system_topics tracking. Use `/load_map` to actually load and start localization with a map.

---

### `/navigate_to_pose/cancel`
**What it does:** Cancels active navigation goals  
**Type:** `action_msgs/CancelGoal`  
**Request:** Empty `{}` (cancels all goals)  
**Response:** `{ return_code: 0, goals_canceling: [] }`  
**Use it for:** "Stop Navigation" or "Cancel" button

```javascript
const cancelNavClient = new ROSLIB.Service({
  ros: ros,
  name: '/navigate_to_pose/cancel',
  serviceType: 'action_msgs/CancelGoal'
});

function cancelNavigation() {
  const request = new ROSLIB.ServiceRequest({});
  
  cancelNavClient.callService(request, function(result) {
    showNotification('Navigation cancelled');
  });
}
```

---

## Actions (For Navigation Goals)

Actions are like services but for **long-running tasks** with feedback. Use them for navigation.

### `/navigate_to_pose` Action
**What it does:** Sends robot to a target pose  
**Type:** `nav2_msgs/NavigateToPose`  
**Use it for:** "Go to" navigation commands

```javascript
const navigateClient = new ROSLIB.ActionClient({
  ros: ros,
  serverName: '/navigate_to_pose',
  actionName: 'nav2_msgs/NavigateToPose'
});

function navigateTo(x, y, theta) {
  const goal = new ROSLIB.Goal({
    actionClient: navigateClient,
    goalMessage: {
      pose: {
        header: {
          frame_id: 'map'
        },
        pose: {
          position: { x: x, y: y, z: 0.0 },
          orientation: quaternionFromYaw(theta)
        }
      }
    }
  });
  
  goal.on('feedback', function(feedback) {
    // Update progress indicator
    console.log('Navigation feedback:', feedback);
  });
  
  goal.on('result', function(result) {
    showNotification('Reached destination!');
  });
  
  goal.send();
}

// Helper: convert yaw angle to quaternion
function quaternionFromYaw(yaw) {
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw / 2),
    w: Math.cos(yaw / 2)
  };
}
```

**UI Example - Click on map to navigate:**
```javascript
mapCanvas.addEventListener('click', function(event) {
  const rect = mapCanvas.getBoundingClientRect();
  const x = (event.clientX - rect.left) * mapResolution;
  const y = (event.clientY - rect.top) * mapResolution;
  
  navigateTo(x, y, 0);  // Navigate to clicked point
});
```

---

## Publishing Topics (Send Data to Robot)

### `/initialpose` - Set Initial Robot Pose
**What it does:** Tells AMCL where the robot is on the map  
**Type:** `geometry_msgs/PoseWithCovarianceStamped`  
**Use it for:** "Set Initial Pose" button when robot position is unknown

```javascript
const initPosePub = new ROSLIB.Topic({
  ros: ros,
  name: '/initialpose',
  messageType: 'geometry_msgs/PoseWithCovarianceStamped'
});

function setInitialPose(x, y, theta) {
  const pose = new ROSLIB.Message({
    header: {
      frame_id: 'map',
      stamp: { sec: 0, nanosec: 0 }  // Use current time
    },
    pose: {
      pose: {
        position: { x: x, y: y, z: 0.0 },
        orientation: quaternionFromYaw(theta)
      },
      covariance: [
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.06853891945200942
      ]
    }
  });
  
  initPosePub.publish(pose);
  showNotification('Initial pose set');
}
```

**UI Example:**
```javascript
// Allow user to click on map to set initial position
let settingInitialPose = false;

document.getElementById('setInitPoseBtn').onclick = function() {
  settingInitialPose = true;
  showMessage('Click on map to set robot position');
};

mapCanvas.addEventListener('click', function(event) {
  if (settingInitialPose) {
    const rect = mapCanvas.getBoundingClientRect();
    const x = (event.clientX - rect.left) * mapResolution;
    const y = (event.clientY - rect.top) * mapResolution;
    
    setInitialPose(x, y, 0);  // Assume facing forward
    settingInitialPose = false;
  }
});
```

---

## Complete UI Example: Mode Switching Panel

```html
<!DOCTYPE html>
<html>
<head>
  <title>Robot Control</title>
  <script src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
</head>
<body>
  <div id="status">
    <h2>Robot Status</h2>
    <p>State: <span id="robotState">Connecting...</span></p>
    <p>Mode: <span id="currentMode">-</span></p>
    <p>Active Map: <span id="activeMap">-</span></p>
  </div>
  
  <div id="controls">
    <h2>Mode Control</h2>
    <button onclick="startMapping()">Start Mapping</button>
    <button onclick="saveCurrentMap()">Save Map</button>
    
    <h3>Load Map</h3>
    <select id="mapSelector"></select>
    <button onclick="loadSelectedMap()">Load Map</button>
  </div>
  
  <div id="map">
    <canvas id="mapCanvas" width="800" height="600"></canvas>
  </div>
  
  <script>
    // Connect to ROS
    const ros = new ROSLIB.Ros({ url: 'ws://192.168.1.100:9090' });
    
    // Robot state listener
    const stateListener = new ROSLIB.Topic({
      ros: ros,
      name: '/robot/state',
      messageType: 'std_msgs/String'
    });
    
    stateListener.subscribe(function(message) {
      document.getElementById('robotState').textContent = message.data;
    });
    
    // Mode manager status
    const statusListener = new ROSLIB.Topic({
      ros: ros,
      name: '/mode_manager/status',
      messageType: 'std_msgs/String'
    });
    
    statusListener.subscribe(function(message) {
      const status = JSON.parse(message.data);
      document.getElementById('currentMode').textContent = status.mode;
      document.getElementById('activeMap').textContent = status.map || 'None';
    });
    
    // Available maps
    const mapsListener = new ROSLIB.Topic({
      ros: ros,
      name: '/available_maps',
      messageType: 'std_msgs/String'
    });
    
    mapsListener.subscribe(function(message) {
      const maps = JSON.parse(message.data);
      const selector = document.getElementById('mapSelector');
      selector.innerHTML = '';
      maps.forEach(map => {
        const option = document.createElement('option');
        option.value = map;
        option.textContent = map;
        selector.appendChild(option);
      });
    });
    
    // Map visualization
    const mapListener = new ROSLIB.Topic({
      ros: ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      qos: { durability: 'transient_local', reliability: 'reliable' }
    });
    
    mapListener.subscribe(function(message) {
      renderMap(message);
    });
    
    // Services
    const startSlamClient = new ROSLIB.Service({
      ros: ros,
      name: '/start_slam',
      serviceType: 'std_srvs/Trigger'
    });
    
    const stopSlamClient = new ROSLIB.Service({
      ros: ros,
      name: '/stop_slam_and_save',
      serviceType: 'interfaces/SetString'
    });
    
    const loadMapClient = new ROSLIB.Service({
      ros: ros,
      name: '/load_map',
      serviceType: 'interfaces/SetString'
    });
    
    // Functions
    function startMapping() {
      startSlamClient.callService(new ROSLIB.ServiceRequest({}), function(result) {
        alert(result.success ? 'Mapping started!' : 'Failed to start mapping');
      });
    }
    
    function saveCurrentMap() {
      const mapName = prompt('Enter map name:');
      if (mapName) {
        stopSlamClient.callService(
          new ROSLIB.ServiceRequest({ data: mapName }),
          function(result) {
            alert(result.success ? 'Map saved!' : 'Failed to save: ' + result.message);
          }
        );
      }
    }
    
    function loadSelectedMap() {
      const mapName = document.getElementById('mapSelector').value;
      loadMapClient.callService(
        new ROSLIB.ServiceRequest({ data: mapName }),
        function(result) {
          alert(result.success ? 'Map loaded!' : 'Failed to load map');
        }
      );
    }
    
    function renderMap(mapData) {
      const canvas = document.getElementById('mapCanvas');
      const ctx = canvas.getContext('2d');
      // ... (see map rendering example above)
    }
  </script>
</body>
</html>
```

---

## Common Workflows

### Workflow 1: Mapping a New Area
1. User clicks "Start Mapping"
2. Call `/start_slam` service
3. Subscribe to `/map` topic to show live map updates
4. Drive robot around (use joystick or teleoperation)
5. User clicks "Save Map"
6. Call `/stop_slam_and_save` with map name
7. Map appears in `/available_maps` topic

### Workflow 2: Navigating with Saved Map
1. Subscribe to `/available_maps` topic
2. User selects map from dropdown
3. Call `/load_map` service
4. Subscribe to `/map` topic (will receive loaded map)
5. If robot position unknown: user sets initial pose via `/initialpose`
6. User clicks on map to set navigation goal
7. Send goal via `/navigate_to_pose` action
8. Subscribe to `/navigate_to_pose/status` to show progress
9. User can cancel via `/navigate_to_pose/cancel` service

### Workflow 3: Using POIs for Quick Navigation
1. After loading map, call `/system/map/pois` service
2. Display POI markers on map
3. User clicks POI marker
4. Send navigation goal to POI coordinates via `/navigate_to_pose` action

---

## Troubleshooting

### "Connection refused" on port 9090
- Check that `core_bringup.launch.py` is running on the robot
- Verify robot IP address
- Check firewall/network settings

### Map not displaying
- Ensure you're subscribing with **Transient Local QoS**
- Check that localization or SLAM mode is active
- Verify `/map` topic is being published: `ros2 topic hz /map`

### Navigation not working
- Make sure localization mode is active (not SLAM)
- Set initial pose if robot position is unknown
- Check that `/navigate_to_pose/status` shows active goals

### Maps not appearing in `/available_maps`
- Maps are auto-detected from `src/slam/maps/` directory
- Ensure map files exist: `<name>.yaml` and `<name>.pgm`
- Wait 1-2 seconds after saving for topic to update

---

## Summary Cheat Sheet

| Task | Topic/Service | Type |
|------|---------------|------|
| **Get list of maps** | `/available_maps` (subscribe) | `std_msgs/String` |
| **Get robot state** | `/robot/state` (subscribe) | `std_msgs/String` |
| **Get detailed status** | `/mode_manager/status` (subscribe) | `std_msgs/String` |
| **Start mapping** | `/start_slam` (call) | `std_srvs/Trigger` |
| **Save map** | `/stop_slam_and_save` (call) | `interfaces/SetString` |
| **Load map** | `/load_map` (call) | `interfaces/SetString` |
| **Get POIs** | `/system/map/pois` (call) | `interfaces/SetString` |
| **View map** | `/map` (subscribe) | `nav_msgs/OccupancyGrid` |
| **Set initial pose** | `/initialpose` (publish) | `geometry_msgs/PoseWithCovarianceStamped` |
| **Navigate to goal** | `/navigate_to_pose` (action) | `nav2_msgs/NavigateToPose` |
| **Cancel navigation** | `/navigate_to_pose/cancel` (call) | `action_msgs/CancelGoal` |
| **View camera** | `http://<ip>:8080/stream?topic=/ascamera/...` | HTTP MJPEG |

---

## Need Help?

- Full topic reference: `docs/topics_summary.md`
- Launch file documentation: See docstrings in `src/bringup/launch/core_bringup.launch.py`
- Service contracts: `docs/interface.md`

**Questions? Ask the backend team!**
