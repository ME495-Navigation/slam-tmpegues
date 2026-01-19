

// Create parameter 'rate', default 100 Hz
// Create ~/reset service, type std/srv/Empty
    // Resets simulation
        // Right now, just resets the ~/timestep to 0
        // At next task, the position of the robot should be reset to x0, y0, theta0
            // Read the parameters at that time, not the values at initialization



// Create parameters x0, y0, theta0 for initial pose of red turtle
    // default all to 0.0
    // relative to nusim/world frame

// Walls
    // Create parameters arena_x_length, arena_y_length
    // Center arena at 0,0
    // Walls 0.25 tall
    // Walls are red
    // use visualization_msgs/MarkerArray on topic ~/real_walls
        // Check qos from assignment 2

// Obstacles
    // Not specified how to get this to work
    // Parameters
        // obstacles.x is a list of x coordinates, float64
        // obstacles.y is a list of y coordinates, float64
        // obstacles.r is the radius (all abstacles are the same radius)
    // All obstacles are .25 m tall
    // All obstacles are red
    // Publish visualization_msgs/MarkerArray on ~/real_obstacles
        // PUblish in the red namespace of Marker message
