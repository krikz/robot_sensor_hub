# Firmware Version Management Guide

This guide explains how to manage and update firmware versions for the Robot Sensor Hub project.

## Version System

The project uses semantic versioning: `MAJOR.MINOR.PATCH`

- **MAJOR**: Breaking changes to protocol or API
- **MINOR**: New features, backward compatible
- **PATCH**: Bug fixes and minor improvements

## Version Information Location

All version information is defined in `src/version.h`:

```c
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0
#define FW_VERSION_STRING "2.1.0"
```

## Updating the Version

### 1. Edit `src/version.h`

For a patch release (bug fixes):
```c
#define FW_VERSION_PATCH 1  // Increment patch
#define FW_VERSION_STRING "2.1.1"
```

For a minor release (new features):
```c
#define FW_VERSION_MINOR 2  // Increment minor
#define FW_VERSION_PATCH 0  // Reset patch
#define FW_VERSION_STRING "2.2.0"
```

For a major release (breaking changes):
```c
#define FW_VERSION_MAJOR 3  // Increment major
#define FW_VERSION_MINOR 0  // Reset minor
#define FW_VERSION_PATCH 0  // Reset patch
#define FW_VERSION_STRING "3.0.0"
```

### 2. Update Protocol Version (if needed)

If communication protocol changes:
```c
#define FW_PROTOCOL_VERSION "2.0"  // Increment protocol version
```

### 3. Build and Test

```bash
pio run -e default -t upload
pio device monitor
```

Verify version on startup:
```json
{"status":0,"message":"Robot Sensor Hub v2.1.1 - Target: ..."}
```

### 4. Query Version

Send command `5`:
```
> 5
{"status":0,"version":{
  "firmware":"2.1.1",
  "major":2,
  "minor":1,
  "patch":1,
  ...
}}
```

## Automatic Build Information

Build date and time are **automatically** set at compile time:
- `FW_BUILD_DATE` - Compilation date (e.g., "Jan 19 2026")
- `FW_BUILD_TIME` - Compilation time (e.g., "07:50:00")

No manual updates needed for these fields.

## Version History Example

| Version | Date | Changes |
|---------|------|---------|
| 2.1.0 | Jan 2026 | Added version system, target configuration |
| 2.0.0 | Jan 2026 | Request-response protocol, removed micro-ROS |
| 1.0.0 | Dec 2025 | Initial ESP-IDF + micro-ROS implementation |

## Python Client Version Check

Check firmware version from Python:

```python
from sensor_client import SensorHubClient

client = SensorHubClient('/dev/ttyUSB0')
version = client.get_version()

print(f"Firmware: {version['firmware']}")
print(f"Protocol: {version['protocol']}")
print(f"Target: {version['target']}")
print(f"Built: {version['build_date']} {version['build_time']}")

# Version comparison
major, minor, patch = version['major'], version['minor'], version['patch']
if major < 2:
    print("Warning: Old firmware version!")
```

## Protocol Compatibility

### Protocol Version 1.0
- Commands 0-5 (sensors, control, version)
- JSON response format
- Serial 115200 baud

When protocol changes incompatibly, increment `FW_PROTOCOL_VERSION`.

## Release Checklist

Before releasing a new version:

- [ ] Update version numbers in `src/version.h`
- [ ] Update `FW_VERSION_STRING` to match
- [ ] Update protocol version if protocol changed
- [ ] Test all commands (0-5)
- [ ] Verify version command returns correct info
- [ ] Test with Python client
- [ ] Update CHANGELOG (if exists)
- [ ] Build all targets (`default`, `full`)
- [ ] Tag release in git: `git tag v2.1.0`

## Git Tagging

Tag releases for easy tracking:

```bash
# Create annotated tag
git tag -a v2.1.0 -m "Release version 2.1.0 - Added version system"

# Push tag
git push origin v2.1.0

# List tags
git tag -l
```

## Version in Documentation

Update version references in:
- `README.md` - Project version at top
- `QUICKSTART.md` - Examples showing version
- `platformio.ini` - Comments if needed
- PR descriptions when merging

## Debugging Version Issues

If version info seems wrong:

1. **Clean build:**
   ```bash
   pio run -t clean
   pio run -e default
   ```

2. **Check compilation output:**
   ```bash
   pio run -e default -v | grep "FW_VERSION"
   ```

3. **Verify `version.h` is included:**
   - Check `#include "version.h"` in `main.cpp`
   - Verify file path is correct

4. **Flash and test:**
   ```bash
   pio run -e default -t upload
   # Send command 5
   ```

## Future Enhancements

Possible version system improvements:
- Git commit hash in build info
- Automatic version bumping script
- Version compatibility checker
- Over-the-air (OTA) update support with version validation
