import time
import mujoco
import mujoco.viewer
import random
import numpy as np
import scipy as sp
import cmpe434_dungeon as dungeon
import matplotlib.pyplot as plt
from a_star import AStarPlanner
from dynamic_window_approach import dwa_control, Config as DWAConfig

# Helper construsts for the viewer for pause/unpause functionality.
paused = False

Higher_dt_mode = False
dt_multiplier = 2

# Pressing SPACE key toggles the paused state.
def mujoco_viewer_callback(keycode):
    global paused
    if keycode == ord(' '):  # Use ord(' ') for space key comparison
        paused = not paused

def get_state(d):
    x, y = d.qpos[0], d.qpos[1]
    w, xq, yq, zq = d.qpos[3:7]
    yaw = np.arctan2(2*(w*zq + xq*yq), 1 - 2*(yq*yq + zq*zq))
    
    # Get velocity in robot frame
    vx, vy= d.qvel[0], d.qvel[1]
    v = vx*np.cos(yaw) + vy*np.sin(yaw)  # Forward speed
    
    # Angular velocity (yaw rate)
    omega = d.qvel[5]  
    
    return np.array([x, y, yaw, v, omega])

# function to represent obstacles edges
def densify(obstacleList,width, spacing):
    dense = []
    for (x,y) in obstacleList:
        # look at your 4‐neighborhood walls and interpolate…
        for dx in np.arange(-width, width,spacing):
            for dy in np.arange(-width, width, spacing):
                dense.append((x+dx, y+dy))
    return dense


def A_Star_path_finder(obstacleList,start_pos,final_pos,grid_resolution=0.5,robot_radius=0.2):
    ox , oy = zip(*obstacleList)
    sx , sy = start_pos[0] , start_pos[1]
    gx , gy = final_pos[0] , final_pos[1]

    # #Uncomment to see the walls on plot
    # plt.plot(ox, oy, ".k")
    # plt.plot(sx, sy, "og")
    # plt.plot(gx, gy, "xb")
    # plt.grid(True)
    # plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_resolution, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    rx.reverse()
    ry.reverse()

    # #Uncomment to see the A* algorithm on plot
    # plt.plot(rx, ry, "-r")
    # plt.pause(0.001)
    # plt.show()    

    return rx , ry

def main():

    # Uncomment to start with an empty model
    # scene_spec = mujoco.MjSpec() 

    # Load existing XML models
    scene_spec = mujoco.MjSpec.from_file("scenes/empty_floor.xml")

    tiles, rooms, connections = dungeon.generate(3, 2, 8)
    for index, r in enumerate(rooms):
        (xmin, ymin, xmax, ymax) = dungeon.find_room_corners(r)
        scene_spec.worldbody.add_geom(name='R{}'.format(index), type=mujoco.mjtGeom.mjGEOM_PLANE, size=[(xmax-xmin)+1, (ymax-ymin)+1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax), (ymin+ymax), 0])

    obstacleList=[]

    for pos, tile in tiles.items():
        if tile == "#":
            scene_spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[1, 1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[pos[0]*2, pos[1]*2, 0])
            
            #To plot the map the wall coordinates are listed.
            obstacleList.append(( pos[0], pos[1]))

    start_pos = random.choice([key for key in tiles.keys() if tiles[key] == "."])
    final_pos = random.choice([key for key in tiles.keys() if tiles[key] == "." and key != start_pos])

    scene_spec.worldbody.add_site(name='start', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.01], rgba=[0, 0, 1, 1],  pos=[start_pos[0]*2, start_pos[1]*2, 0])
    scene_spec.worldbody.add_site(name='finish', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.01], rgba=[1, 0, 0, 1],  pos=[final_pos[0]*2, final_pos[1]*2, 0])


    robot_spec = mujoco.MjSpec.from_file("models/mushr_car/model.xml")

    # Add robots to the scene:
    # - There must be a frame or site in the scene model to attach the robot to.
    # - A prefix is required if we add multiple robots using the same model.
    scene_spec.attach(robot_spec, frame="world", prefix="robot-")
    scene_spec.body("robot-buddy").pos[0] = start_pos[0] * 2
    scene_spec.body("robot-buddy").pos[1] = start_pos[1] * 2

    # Randomize initial orientation
    yaw = np.random.uniform(-np.pi, np.pi)
    euler = np.array([0.0, 0.0, yaw], dtype=np.float64)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(quat, euler, 'xyz')
    scene_spec.body("robot-buddy").quat[:] = quat

    # Add obstacles to the scene
    for i, room in enumerate(rooms):
        obs_pos = random.choice([tile for tile in room if tile != start_pos and tile != final_pos])
        scene_spec.worldbody.add_geom(
            name='Z{}'.format(i), 
            type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
            size=[0.2, 0.05, 0.1], 
            rgba=[0.8, 0.0, 0.1, 1],  
            pos=[obs_pos[0]*2, obs_pos[1]*2, 0.08]
        )

    raw = obstacleList
    dense = densify(raw,0.501,0.25)

    rx , ry = A_Star_path_finder(dense,start_pos,final_pos)
    target_x_list = [2 * x for x in rx]
    target_y_list = [2 * y for y in ry]
    target_points = list(zip(target_x_list, target_y_list))

    # add green tiles to show A* points 
    for i in range(len(rx)):
        scene_spec.worldbody.add_site(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.2, 0.01], rgba=[0, 1, 0, 1],  pos=[target_x_list[i],target_y_list[i], 0])

    # Initalize our simulation
    # Roughly, m keeps static (model) information, and d keeps dynamic (state) information. 
    m = scene_spec.compile()
    d = mujoco.MjData(m)
    DEFAULT_TIMESTEP = 0.002
    if Higher_dt_mode:
    # OPTIONAL PERFORMANCE TUNING:
    # If your real-time loop is running too slowly, you can trade off
    # fidelity for speed by taking larger physics steps and bumping
    # up the solver iterations to keep things stable:
        m.opt.timestep   = DEFAULT_TIMESTEP * dt_multiplier                         
        m.opt.iterations = dt_multiplier * 2

    obstacles = [m.geom(i).id for i in range(m.ngeom) if m.geom(i).name.startswith("Z")]
    uniform_direction_dist = sp.stats.uniform_direction(2)
    obstacle_direction = [[x, y, 0] for x,y in uniform_direction_dist.rvs(len(obstacles))]
    unused = np.zeros(1, dtype=np.int32)

    with mujoco.viewer.launch_passive(m, d, key_callback=mujoco_viewer_callback) as viewer:
      viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
      viewer.cam.fixedcamid = m.camera("robot-third_person").id

      # These actuator names are defined in the model XML file for the robot.
      # Prefixes distinguish from other actuators from the same model.
      velocity = d.actuator("robot-throttle_velocity")
      steering = d.actuator("robot-steering")

      start = time.time()


      # Cache hot lookups into locals
      _mj_step     = mujoco.mj_step
      _dwa_control = dwa_control
      _viewer_sync = viewer.sync


      # tune parameters
      dwa_config = DWAConfig() 
      dwa_config.max_speed = 3
      dwa_config.min_speed = -2
      dwa_config.max_yaw_rate = 360 * np.pi / 180
      dwa_config.max_accel = 15
      dwa_config.max_delta_yaw_rate = 8.5 * np.pi 
      dwa_config.v_resolution = 0.6
      dwa_config.yaw_rate_resolution = 48 * np.pi / 90
      dwa_config.dt = 0.1
      dwa_config.predict_time = .4
      dwa_config.to_goal_cost_gain = 3.0
      dwa_config.speed_cost_gain = .15
      dwa_config.obstacle_cost_gain = 2.0
      dwa_config.robot_radius = 0.1

      #low pass filter value
      alpha=0.1

      orientation_mode   = None 
      phase_start_time    = 0.0
      yaw_tol=np.deg2rad(60)
      orient_forward_t    = 1
      orient_reverse_t    = 1

      if Higher_dt_mode:
          orient_forward_t=1/np.sqrt(dt_multiplier)
          orient_reverse_t=1/np.sqrt(dt_multiplier)

      # how hard to steer (±max steering angle)
      steer_lock          = np.pi * 2 
      # how fast to go forward/back
      orient_speed        = 1.5
      orient_back_speed   = -1.5 

      prev_steer_cmd = 0.0

      max_path_id = len(target_x_list)-1
      path_id=0

      wall_xy = np.array([[x*2, y*2] for x,y in dense])

      arrive_dist_square = 1 ** 2
      finish_dist_square = 0.8 ** 2

      while viewer.is_running() and time.time() - start < 3000:
        step_start = time.time()

        if not paused:
            # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            state=get_state(d)
            dyn_xy  = np.array([[m.geom_pos[g][0], m.geom_pos[g][1]] for g in obstacles])
            dyn_densified = np.array(densify(dyn_xy, 0.161, 0.16))
            ob_xy = np.vstack([wall_xy, dyn_densified])

            path_id_selected = min(path_id + 1 , max_path_id)

            target_selected = target_points[path_id_selected]

            target_point_obstacle_check = np.sum((dyn_densified - target_selected)**2, axis=1)
            target_point_occupied = (target_point_obstacle_check < 1)

            # increase the path_id if there is a obstacle near it. 
            # this sets a better target for dwa algorithm
            while target_point_occupied.any() and path_id != max_path_id :
                path_id += 1
                path_id_selected = min(path_id + 1 ,max_path_id)
                target_selected = target_points[path_id_selected]                
                target_point_obstacle_check = np.sum((dyn_densified - target_selected)**2, axis=1)
                target_point_occupied = (target_point_obstacle_check < 1)


            curr_pos = state[:2]     

            dists = np.sum((ob_xy - curr_pos)**2, axis=1)
            ob_in_range = ob_xy[dists <= 1.2**2]

            delta_x=target_selected[0]-curr_pos[0]
            delta_y=target_selected[1]-curr_pos[1]
            target_yaw=np.arctan2(delta_y, delta_x)
            curr_yaw=state[2]
            raw_diff=target_yaw-curr_yaw
            # wrap into (-π, π]
            yaw_err   = (raw_diff + np.pi) % (2*np.pi) - np.pi

            if orientation_mode is None and abs(yaw_err) > yaw_tol:
                orientation_mode = 'rev'
                phase_start_time  = time.time()

            if orientation_mode is not None:
                now = time.time()
                # — forward locked steer —

                if orientation_mode == 'rev':
                    v_cmd   = orient_back_speed
                    raw_steer_cmd   = -np.sign(yaw_err) * steer_lock

                    if now - phase_start_time > orient_reverse_t:
                        # if still off by more than tol, do another 
                        if abs(yaw_err) > yaw_tol:
                            orientation_mode = 'fwd'
                            phase_start_time  = now
                        else:
                            # orientation corrected!
                            orientation_mode = None

                elif orientation_mode == 'fwd':
                    v_cmd   =  orient_speed
                    raw_steer_cmd   = np.sign(yaw_err) * steer_lock
                    # once time’s up, switch to reverse
                    if now - phase_start_time > orient_forward_t:
                        orientation_mode = 'rev'
                        phase_start_time  = now


              # low pass filtering data 
                steer_cmd = alpha * raw_steer_cmd + (1 - alpha) * prev_steer_cmd
        
                # apply to the actuators
                velocity.ctrl = v_cmd
                steering.ctrl = steer_cmd
         
            else:
                u, _traj= _dwa_control(state, dwa_config,target_selected , ob_in_range)
                v_cmd, raw_steer_cmd = u

                # #Uncomment to see steering and velocity values
                # print(f"Velocity value: {u[0]} Steering value: {u[1]}")
       
                raw_steer_cmd *= 2.5

                # low pass filtering data  
                # clipped steering cmd to avoid high peaks that make the car jump  
                steer_cmd = np.clip(alpha * raw_steer_cmd,-0.7,0.7) + (1 - alpha) * prev_steer_cmd
                
                # apply to the actuators
                velocity.ctrl = v_cmd
                steering.ctrl = np.clip(steer_cmd,-4,4)

            # remember for next frame
            prev_steer_cmd = steer_cmd

            dx      = state[0] - target_selected[0]
            dy      = state[1] - target_selected[1]
            dist_sq = dx*dx + dy*dy


            if dist_sq < arrive_dist_square:
                if path_id < max_path_id:
                    path_id += 1
                elif path_id == max_path_id and dist_sq < finish_dist_square:
                    velocity.ctrl = 0
                    steering.ctrl = 0
                else:
                    pass

            # Update obstables (bouncing movement)
            for i, x in enumerate(obstacles):
                dx = obstacle_direction[i][0]
                dy = obstacle_direction[i][1]

                px = m.geom_pos[x][0]
                py = m.geom_pos[x][1]
                pz = 0.02

                nearest_dist = mujoco.mj_ray(m, d, [px, py, pz], obstacle_direction[i], None, 1, -1, unused)

                if nearest_dist >= 0 and nearest_dist < 0.4:
                    obstacle_direction[i][0] = -dy
                    obstacle_direction[i][1] = dx

                obstacle_update = 0.001

                if Higher_dt_mode:
                    obstacle_update *= dt_multiplier

                m.geom_pos[x][0] = m.geom_pos[x][0]+dx*obstacle_update
                m.geom_pos[x][1] = m.geom_pos[x][1]+dy*obstacle_update

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            _mj_step(m, d)


            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            _viewer_sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
