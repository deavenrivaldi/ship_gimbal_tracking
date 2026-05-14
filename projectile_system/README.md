# Projectile Simulation System

This project provides a ROS 2 system to spawn and simulate projectiles in Gazebo. Currently, the system models only the projectile itself; however, a "virtual cannon" implementation can be added if required for future development.

## How it Works

When a `/fire` service request is received, the system performs the following steps:
1. **Spawn**: Spawns a sphere model at the specified position in Gazebo.
2. **Launch**: Applies an impulsive force in the requested direction to simulate the initial velocity.

## Getting Started

### Launching the System
To start the simulation environment, run the following command:

```bash
ros2 launch projectile_bringup projectile.launch.py
```

### Dependencies
Ensure that you add the following dependency to your `package.xml`:
```xml
<depend>projectile_msgs</depend>
```

---

## Usage

To fire a projectile, call the `/fire` service from your ROS 2 node.

### Example Code (Python)

```python
from projectile_msgs.srv import Fire

# Create the service client
client = self.create_client(Fire, '/fire')

# Prepare the request
request = Fire.Request()
request.position.x = 0.0  # Spawn position
request.position.y = 0.0
request.position.z = 5.0
request.direction.x = 1.0  # Direction vector
request.direction.y = 0.0
request.direction.z = 0.0
request.velocity = 15.5    # Initial speed (m/s)

# Call the service
future = client.call_async(request)
```

### Service Response
The service returns:
* `success` (bool): `True` if the projectile was successfully spawned.
* `projectile_name` (string): A unique identifier for the spawned projectile.

---

## Configuration

Simulation parameters are located in:
`projectile_bringup/config/projectile_params.yaml`

### Example YAML Configuration
```yaml
gz_interface:
  ros__parameters:
    world_name: world_test  # Gazebo world name
    world_file: world_test.sdf
    force: 1000.0           # Impulsive force magnitude
```

*Note: You can modulate the `force` parameter, though its utility may depend on your specific project requirements.*