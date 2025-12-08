# ROS Flutter GUI App - AI Coding Agent Instructions

## Project Overview
**ROS Flutter GUI App** is a cross-platform robot control interface built with Flutter and Flame game engine, supporting ROS1/ROS2 via rosbridge WebSocket. It runs on Android, iOS, Web, Linux, and Windows.

## Architecture Overview

### Core Components
1. **ROS Communication Layer** (`lib/provider/ros_channel.dart`, `ros_bridge_player.dart`)
   - Manages WebSocket connection to ROS via `roslibdart` library
   - Central hub for all topic subscriptions/publications
   - Contains `RosChannel` class with state management for robot data (pose, velocity, battery, diagnostics)
   - Uses `RosBridgePlayer` for managing subscriptions and publisher lifecycle

2. **Rendering Engine** (`lib/page/main_flame.dart`, `main_page.dart`)
   - Built on Flame game framework for high-performance 2D rendering
   - Renders map, robot, paths, laser scans, point clouds via display components
   - Handles gestures (zoom, pan) for map navigation
   - Real-time 50ms update loop for robot pose tracking

3. **State Management** (`lib/provider/`)
   - **GlobalState**: Layer visibility toggles, interaction modes (normal/reloc/addNavPoint/mapEdit)
   - **ThemeProvider**: Dark/light mode theming
   - **NavPointManager**: Navigation waypoint management
   - **DiagnosticManager**: System diagnostic monitoring
   - All use Provider package for reactive updates

4. **Data Models** (`lib/basic/`)
   - **RobotPose**: Position (x, y) and orientation (theta)
   - **OccupancyMap**: Grid-based map representation
   - **NavPoint**: Waypoint with pose and metadata
   - **Transform**: TF frame conversions and interpolation
   - **LaserScan**, **PointCloud2**, **TopologyMap**: Sensor data structures

### Data Flow Patterns
```
ROS Bridge (WebSocket) 
  → roslibdart Topic objects
    → RosChannel (subscriptions to ValueNotifier<T>)
      → Provider listeners (GlobalState, ThemeProvider, etc.)
        → Flame rendering components
          → UI overlays (Flutter widgets)
```

## Key Workflows & Build Commands

### Development Setup
```powershell
flutter pub get          # Install dependencies including custom overrides
flutter generate         # Generate localization files (l10n)
```

### Local Development
```powershell
flutter run -d windows      # Run on Windows debug device
flutter run -d chrome       # Run on Web
flutter analyze            # Check lints per analysis_options.yaml
```

### Building for Deployment
```powershell
# Android (APK split per ABI)
flutter build apk --split-per-abi

# Windows (Release)
flutter build windows --release

# Web (Release - hosted on Firebase/CDN)
flutter build web --release

# Linux (Release)
flutter build linux --release
```

### Custom Dependencies (via pubspec.yaml overrides)
- `roslibdart`: Local fork at `thirdparty/roslibdart/` (customized ROS client)
- `gamepads_windows`: Local fork at `thirdparty/gamepads_windows/` (Windows gamepad support)

### Localization Workflow
- Edit `lib/language/l10n/app_zh.arb` for Chinese strings
- Run `flutter generate` to create `AppLocalizations` class
- Reference strings in code: `AppLocalizations.of(context)!.stringKey`

## Critical Patterns & Conventions

### ROS Topic Subscription Pattern
```dart
// In RosChannel, subscribe within init():
late Topic topic = ros.subscribe('/topic_name', 'geometry_msgs/Twist', 
  (message) {
    // Update ValueNotifier: robotSpeed_.value = RobotSpeed(...);
  });
```

### Flame Component Integration
- Every renderable element (map, paths, laser, robot) is a Flame `PositionComponent`
- Update method called 50ms: use `deltaTime` for animations
- Transform map coordinates to screen space using `camera.viewfinder.globalToLocal()`
- Layer management via `GlobalState.getLayerState('layerName')` before rendering

### Mode System (Global State)
```dart
enum Mode { normal, reloc, addNavPoint, robotFixedCenter, mapEdit }
globalState.mode.value = Mode.reloc;  // Switch modes
```
- **normal**: View/pan map
- **reloc**: Click to set robot initial pose
- **addNavPoint**: Click to add waypoint
- **mapEdit**: Edit costmap
- **robotFixedCenter**: Camera follows robot

### Theme & Localization
- `ThemeProvider.isDarkMode` controls colors
- Colors defined in provider, accessed in display components
- All UI text uses `AppLocalizations` for i18n support

## Integration Points & External Dependencies

### ROS Communication
- **rosbridge_suite** (ROS1/ROS2 server): Must be running; Flask backend handles launch
- **roslibdart**: Custom fork handles JSON-RPC over WebSocket
- Key topics subscribed in `RosChannel`: `/map`, `/amcl_pose`, `/cmd_vel`, `/diagnostics`, `/tf`

### Hardware Integration
- **Gamepads**: Windows/Linux support via `gamepads` package (custom Windows fork)
- **Wakelock**: Keeps device screen on during operation
- **MJPEG streams**: Optional camera feed via `flutter_mjpeg`

### Platform-Specific Code
- **Windows**: Native gamepad support (`gamepads_windows/`)
- **Linux**: SVG rendering for icons via `flame_svg`
- **Web**: MJPEG camera disabled (browser security), uses `web_socket_channel`

## File Organization
- `lib/page/`: Full pages (main_page, connect_page, setting_page, etc.)
- `lib/display/`: Flame components (map, laser, paths, costmaps)
- `lib/provider/`: State managers (RosChannel, GlobalState, managers)
- `lib/basic/`: Data models and utilities (RobotPose, OccupancyMap, transforms)
- `lib/service/`: High-level services (RobotControlService)
- `lib/global/`: Global config (Setting singleton with gamepad mappings)
- `ROS2/`: Python backend service for rosbridge launch (optional)
- `thirdparty/`: Local forks of dependencies

## Common Tasks

### Adding a New ROS Topic
1. Define data model in `lib/basic/`
2. Subscribe in `RosChannel.__init()` with callback
3. Create display component in `lib/display/`
4. Add to `MainFlame.onMount()` and wire to `GlobalState` layer toggle
5. Update layer settings in `GlobalState._layerStates`

### Adding UI Controls
- Use Flutter widgets in `main_page.dart` overlays
- Access RosChannel and GlobalState via `Provider.of(context)`
- Keep business logic in providers, UI logic in widgets

### Debugging ROS Communication
- Check RosChannel connection status: `rosConnectState_`
- Monitor subscribed topics in RosBridgePlayer
- Add print logs to topic callbacks (avoid in production)
- Use `TF2Dart` for frame lookup issues

## Code Quality
- Lint rules in `analysis_options.yaml` follow `flutter_lints` package
- No print/console output in production builds (use proper logging)
- Error handling via Flutter error callbacks and Provider error states
