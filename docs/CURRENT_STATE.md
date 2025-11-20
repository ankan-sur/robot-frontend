# Current State - November 2025

## Recent Updates and Fixes

### Frontend (React + TypeScript)
**Branch:** `nav-mode-workflow`

#### Mobile Improvements
- **Virtual Joystick**: Added touch-based joystick for mobile teleop (replaces button pad on small screens)
- **Battery Display**: Compact "BAT: XX%" display next to robot title on mobile
- **Map Save Dialog**: Modal dialog replaces browser `prompt()` for better mobile compatibility
  - Touch-friendly input field
  - Enter key to save, Escape to cancel
  - Works reliably on all mobile browsers

#### UI Cleanup
- **Removed**: All emoji characters replaced with text labels
- **Removed**: POI (Points of Interest) system completely disabled
- **Removed**: Core Topics/Services lists from settings panel
- **Mobile Layout**: Fixed button layout - DEBUG/SETTINGS on separate row

#### Mode Switching
- **Idle Mode Support**: Can view maps without starting localization
- **Map Viewer**: Dropdown to load and preview saved maps in idle mode
- **Fixed**: Mode switching reliability - no more frozen/greyed-out buttons
- **Fixed**: Idle mode map loading doesn't break operating state

#### Button Labels Cleaned Up
- SLAM → Map (SLAM)
- Localization → Map (Nav)
- Save Map & Stop SLAM → Save Map & Stop
- All buttons use consistent naming

#### Emergency Restart
- **Restart Button**: Located in settings panel
- Kills ROS stack and triggers automatic restart via Docker/systemd
- Use when modes are stuck or system unresponsive

### Backend (ROS2 Python)
**Branch:** `working-teleop-base`

#### Mode Manager Improvements
**File:** `robothome/src/mode_manager/mode_manager/mode_manager_node.py`

**CRITICAL BUG FIX:**
- Fixed indentation bug in `stop_slam_and_save()` - service call was nested inside wrong conditional block
- Maps now save correctly every time

**New Features:**
- Idle mode support: Can stop both SLAM and localization processes
- Comprehensive logging with prefixes: `[MODE_SWITCH]`, `[START_SLAM]`, `[START_LOC]`
- AMCL-only localization mode (bypasses Nav2 controller issues)

**Configuration:**
```python
# Default parameters
maps_dir: ~/ros2_ws/install/slam/share/slam/maps/
slam_launch: slam/slam_mode.launch.py
localization_launch: navigation/localization_amcl_only.launch.py
use_filtered_scan: false
```

#### AMCL-Only Localization
**File:** `robothome/src/navigation/launch/localization_amcl_only.launch.py`

Created simplified localization that avoids Nav2 controller_server failures:
- **Nodes**: map_server + amcl only
- **No Nav2 Stack**: No controller, planner, or behavior servers
- **Purpose**: Reliable pose estimation for demos without full autonomous navigation
- **Benefits**: Faster startup, fewer failure points, simpler debugging

#### System Topics Improvements
**File:** `robothome/src/system_topics/system_topics/system_topics_node.py`

**Restart Service Enhanced:**
- Tries graceful shutdown first: calls `~/.stop_ros.sh` if it exists
- Falls back to killing process group if script fails/missing
- Works correctly in Docker containers
- Auto-restart triggered by Docker restart policy or systemd

#### Manual Testing Script
**File:** `robothome/scripts/test_navigation_manual.sh`

New bash script for manual AMCL testing and RViz demos:
```bash
./test_navigation_manual.sh map_name
```
- Stops existing ROS stack gracefully
- Launches AMCL-only localization with specified map
- Provides RViz setup instructions
- For demonstrating navigation goals from PC

## Current Architecture

### Map Storage
Maps are saved to the **install space** (not source):
```
~/ros2_ws/install/slam/share/slam/maps/
  ├── map_name.yaml    # Map metadata
  └── map_name.pgm     # Map image
```

### Battery Topic
- **Topic:** `/ros_robot_controller/battery`
- **Type:** `std_msgs/UInt16`
- **Units:** Millivolts
- **Range:** 6000-8400 mV (2S LiPo)
- **Display:** Percentage bar + voltage on desktop, compact percentage on mobile

### Operating Modes

#### 1. SLAM Mode
- Runs slam_toolbox in sync mode
- Creates new maps from scratch
- Save with "Save Map & Stop" button (modal dialog prompts for name)
- Auto-switches to idle after save

#### 2. Localization Mode (AMCL-Only)
- Loads saved map
- Runs AMCL for pose estimation
- No autonomous navigation yet (controller_server issues)
- Reliable for showing localization in demos

#### 3. Idle Mode
- No SLAM or localization running
- Can view saved maps without starting processes
- Camera feed available
- Switch to SLAM or Localization from here

### Known Issues Being Addressed

#### Performance Warnings
- **EKF Update Rate**: "Failed to meet update rate" (2+ seconds lag)
  - Robot CPU overloaded, affects odometry quality
- **Lidar Timestamps**: "Scan timestamp earlier than TF cache"
  - CRITICAL: Prevents SLAM from matching scans to poses
- **Costmap Errors**: "Sensor origin out of map bounds"
  - CRITICAL: Breaks navigation

#### Nav2 Controller
- `controller_server` fails to configure
- Error: "async_send_request failed"
- **Current Workaround**: Using AMCL-only mode (no autonomous navigation)
- **Future Fix**: Debug controller config, add proper parameters

## Deployment Instructions

### Update Backend on Robot

1. **Pull latest code:**
```bash
cd ~/ros2_ws/src/robothome
git pull origin working-teleop-base
```

2. **Rebuild packages:**
```bash
cd ~/ros2_ws
colcon build --packages-select mode_manager navigation system_topics
source install/setup.bash
```

3. **Deploy manual test script:**
```bash
chmod +x ~/ros2_ws/src/robothome/scripts/test_navigation_manual.sh
# Optional: link to home directory for easier access
ln -s ~/ros2_ws/src/robothome/scripts/test_navigation_manual.sh ~/test_navigation.sh
```

4. **Restart ROS:**
```bash
~/.stop_ros.sh
# Or use web UI restart button
# System should auto-restart via Docker/systemd
```

### Update Frontend

Frontend is already deployed if using the web UI. For local development:
```bash
cd ~/path/to/frontend
git pull origin nav-mode-workflow
npm install  # If package.json changed
npm run build
```

## Testing Checklist

### Basic Functionality
- [ ] Connect to robot at `ws://ROBOT_IP:9090`
- [ ] Battery percentage displays correctly
- [ ] Camera feed loads at `http://ROBOT_IP:8080`
- [ ] Teleop works (virtual joystick on mobile, buttons on desktop)

### SLAM Mode
- [ ] Start SLAM mode from idle
- [ ] Map builds as robot moves
- [ ] Click "Save Map & Stop" button
- [ ] Modal dialog appears (not browser prompt)
- [ ] Enter map name and save
- [ ] Mode switches to idle automatically
- [ ] Check map files exist in `~/ros2_ws/install/slam/share/slam/maps/`

### Localization Mode
- [ ] Select saved map from dropdown in idle mode
- [ ] Switch to localization mode
- [ ] AMCL starts without controller_server errors
- [ ] Robot pose updates on map as robot moves
- [ ] Lidar scans display correctly

### Mode Switching
- [ ] Switch from idle → SLAM → idle
- [ ] Switch from idle → localization → idle
- [ ] No buttons freeze or grey out permanently
- [ ] Console logs show `[MODE_SWITCH]` messages

### Emergency Restart
- [ ] Click restart button in settings
- [ ] ROS stack stops
- [ ] System automatically restarts within 5-10 seconds
- [ ] Can reconnect and use robot normally

### Mobile Testing
- [ ] Virtual joystick appears and works
- [ ] Battery percentage visible next to title
- [ ] Map save dialog is touch-friendly
- [ ] All buttons are properly sized for touch
- [ ] Layout doesn't overflow on small screens

## Debugging

### View Mode Switch Logs
```bash
# On robot
ros2 topic echo /rosout | grep -E "\[MODE_SWITCH\]|\[START_"
```

### Check Map Files
```bash
ls -la ~/ros2_ws/install/slam/share/slam/maps/
```

### Test Manual Navigation
```bash
~/test_navigation.sh map_name
# Then open RViz on PC and send 2D Nav Goals
```

### Monitor Performance
```bash
# CPU usage
htop

# ROS node CPU/memory
ros2 run ros2_perf resource_monitor

# TF delays
ros2 run tf2_ros tf2_monitor
```

## Git Branches

### Frontend
- **Repository:** `ece480_capstone_henry_ford_health`
- **Active Branch:** `nav-mode-workflow`
- **Last Commit:** Modal dialog for map saving

### Backend
- **Repository:** `robothome`
- **Active Branch:** `working-teleop-base`
- **Last Commit:** Improved restart service with graceful stop

## Next Steps

### Short Term (Demo Prep)
1. Deploy backend changes to robot
2. Test SLAM map saving workflow
3. Demonstrate AMCL localization
4. Show manual navigation in RViz from PC

### Medium Term (Post-Demo)
1. Fix controller_server configuration
2. Add full Nav2 autonomous navigation
3. Address performance issues (EKF, lidar timing)
4. Optimize CPU usage

### Long Term (Production)
1. Add automatic map updates during localization
2. Implement proper error recovery
3. Add diagnostics dashboard
4. Set up logging/monitoring system

## Contact and Support

- Check `/rosout` topic for detailed logs
- Use emergency restart button if system hangs
- Performance issues expected under heavy load
- Controller_server issues are known, using AMCL-only workaround
