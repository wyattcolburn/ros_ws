import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow
from matplotlib.collections import LineCollection
import matplotlib.animation as animation

class RaycastingVisualizer:
    def __init__(self, robot_pose, obstacles, lidar_config):
        """
        robot_pose: (x, y, yaw) in radians
        obstacles: list of (center_x, center_y, radius)
        lidar_config: dict with 'fov', 'num_beams', 'max_range'
        """
        self.robot_x, self.robot_y, self.robot_yaw = robot_pose
        self.obstacles = obstacles
        self.fov = lidar_config['fov']
        self.num_beams = lidar_config['num_beams']
        self.max_range = lidar_config['max_range']
        
        # Calculate beam angles
        start_angle = self.robot_yaw - self.fov / 2
        end_angle = self.robot_yaw + self.fov / 2
        self.beam_angles = np.linspace(start_angle, end_angle, self.num_beams)
        
        # Store results
        self.distances = []
        self.hit_points = []
        
    def ray_circle_intersection(self, origin, direction, circle_center, radius):
        """
        Calculate intersection between ray and circle.
        Returns distance t (or None if no valid intersection)
        """
        ox, oy = origin
        dx, dy = direction
        cx, cy = circle_center
        
        # Vector from circle center to ray origin
        mx = ox - cx
        my = oy - cy
        
        # Quadratic coefficients
        a = dx**2 + dy**2
        b = 2 * (mx * dx + my * dy)
        c = mx**2 + my**2 - radius**2
        
        # Discriminant
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return None  # No intersection
        
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        
        # Return smallest positive t
        valid_ts = [t for t in [t1, t2] if t > 0.001]  # Small epsilon to avoid self-intersection
        
        if valid_ts:
            return min(valid_ts)
        return None
    
    def cast_all_rays(self):
        """Cast all rays and compute distances"""
        origin = (self.robot_x, self.robot_y)
        
        for angle in self.beam_angles:
            direction = (np.cos(angle), np.sin(angle))
            
            # Find closest intersection with all obstacles
            min_distance = self.max_range
            closest_hit = None
            
            for obs_center, obs_radius in [(o[:2], o[2]) for o in self.obstacles]:
                t = self.ray_circle_intersection(origin, direction, obs_center, obs_radius)
                if t is not None and t < min_distance:
                    min_distance = t
                    closest_hit = (
                        origin[0] + t * direction[0],
                        origin[1] + t * direction[1]
                    )
            
            # If no hit, extend to max range
            if closest_hit is None:
                closest_hit = (
                    origin[0] + min_distance * direction[0],
                    origin[1] + min_distance * direction[1]
                )
            
            self.distances.append(min_distance)
            self.hit_points.append(closest_hit)
    
    def plot_static_view(self, show_steps=False):
        """Create a static visualization showing all rays"""
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Plot obstacles
        for cx, cy, r in self.obstacles:
            circle = Circle((cx, cy), r, color='red', alpha=0.3, label='Obstacle' if cx == self.obstacles[0][0] else '')
            ax.add_patch(circle)
            ax.plot(cx, cy, 'rx', markersize=8)
        
        # Plot robot
        robot_size = 0.3
        robot_circle = Circle((self.robot_x, self.robot_y), robot_size, 
                            color='blue', alpha=0.7, label='Robot')
        ax.add_patch(robot_circle)
        
        # Plot robot heading
        arrow_length = 0.8
        arrow = FancyArrow(self.robot_x, self.robot_y,
                          arrow_length * np.cos(self.robot_yaw),
                          arrow_length * np.sin(self.robot_yaw),
                          width=0.15, head_width=0.3, head_length=0.2,
                          color='blue', alpha=0.8)
        ax.add_patch(arrow)
        
        # Cast rays if not already done
        if not self.distances:
            self.cast_all_rays()
        
        # Plot all rays
        for i, (angle, hit_point) in enumerate(zip(self.beam_angles, self.hit_points)):
            color = 'green' if self.distances[i] < self.max_range else 'gray'
            alpha = 0.3 if self.distances[i] >= self.max_range else 0.6
            
            ax.plot([self.robot_x, hit_point[0]], 
                   [self.robot_y, hit_point[1]], 
                   color=color, alpha=alpha, linewidth=0.5)
            
            # Mark hit points
            if self.distances[i] < self.max_range:
                ax.plot(hit_point[0], hit_point[1], 'go', markersize=2)
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Hallucinated LiDAR: All Rays Cast from Robot Pose')
        
        # Set axis limits
        margin = 2
        all_x = [self.robot_x] + [o[0] for o in self.obstacles] + [p[0] for p in self.hit_points]
        all_y = [self.robot_y] + [o[1] for o in self.obstacles] + [p[1] for p in self.hit_points]
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        plt.tight_layout()
        return fig, ax
    
    def plot_step_by_step(self, step_indices=[0, 5, 10, 20]):
        """Create subplots showing progressive ray casting"""
        n_steps = len(step_indices)
        fig, axes = plt.subplots(2, 2, figsize=(16, 14))
        axes = axes.flatten()
        
        # Cast rays if not already done
        if not self.distances:
            self.cast_all_rays()
        
        for idx, (ax, step) in enumerate(zip(axes, step_indices)):
            # Plot obstacles
            for cx, cy, r in self.obstacles:
                circle = Circle((cx, cy), r, color='red', alpha=0.3)
                ax.add_patch(circle)
                ax.plot(cx, cy, 'rx', markersize=8)
            
            # Plot robot
            robot_size = 0.3
            robot_circle = Circle((self.robot_x, self.robot_y), robot_size, 
                                color='blue', alpha=0.7)
            ax.add_patch(robot_circle)
            
            # Plot robot heading
            arrow_length = 0.8
            arrow = FancyArrow(self.robot_x, self.robot_y,
                              arrow_length * np.cos(self.robot_yaw),
                              arrow_length * np.sin(self.robot_yaw),
                              width=0.15, head_width=0.3, head_length=0.2,
                              color='blue', alpha=0.8)
            ax.add_patch(arrow)
            
            # Plot rays up to current step
            for i in range(min(step + 1, len(self.beam_angles))):
                angle = self.beam_angles[i]
                hit_point = self.hit_points[i]
                distance = self.distances[i]
                
                # Current ray is highlighted
                if i == step:
                    color = 'orange'
                    alpha = 1.0
                    linewidth = 2.5
                else:
                    color = 'green' if distance < self.max_range else 'gray'
                    alpha = 0.3
                    linewidth = 0.5
                
                ax.plot([self.robot_x, hit_point[0]], 
                       [self.robot_y, hit_point[1]], 
                       color=color, alpha=alpha, linewidth=linewidth)
                
                # Mark hit points
                if distance < self.max_range:
                    marker_color = 'orange' if i == step else 'green'
                    marker_size = 8 if i == step else 2
                    ax.plot(hit_point[0], hit_point[1], 'o', 
                           color=marker_color, markersize=marker_size)
                    
                    # Add distance annotation for current ray
                    if i == step:
                        mid_x = (self.robot_x + hit_point[0]) / 2
                        mid_y = (self.robot_y + hit_point[1]) / 2
                        ax.annotate(f'd={distance:.2f}m', 
                                  xy=(mid_x, mid_y),
                                  fontsize=10, color='orange',
                                  bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title(f'Step {idx+1}: Beam {step+1}/{self.num_beams} (θ={np.degrees(self.beam_angles[step]):.1f}°)')
            
            # Set axis limits
            margin = 2
            all_x = [self.robot_x] + [o[0] for o in self.obstacles] + [p[0] for p in self.hit_points]
            all_y = [self.robot_y] + [o[1] for o in self.obstacles] + [p[1] for p in self.hit_points]
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        plt.tight_layout()
        return fig
    
    def plot_single_ray_detail(self, beam_index=10):
        """Detailed view of a single ray's intersection calculation"""
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Cast rays if not already done
        if not self.distances:
            self.cast_all_rays()
        
        angle = self.beam_angles[beam_index]
        direction = (np.cos(angle), np.sin(angle))
        origin = (self.robot_x, self.robot_y)
        
        # Plot obstacles with labels
        intersection_distances = []
        for i, (cx, cy, r) in enumerate(self.obstacles):
            circle = Circle((cx, cy), r, color='red', alpha=0.2, 
                          edgecolor='red', linewidth=2)
            ax.add_patch(circle)
            ax.plot(cx, cy, 'rx', markersize=10)
            ax.text(cx, cy - r - 0.5, f'Obstacle {i+1}', 
                   ha='center', fontsize=10, color='red')
            
            # Calculate and show intersection
            t = self.ray_circle_intersection(origin, direction, (cx, cy), r)
            if t is not None:
                hit_x = origin[0] + t * direction[0]
                hit_y = origin[1] + t * direction[1]
                ax.plot(hit_x, hit_y, 'mo', markersize=10, 
                       label=f'Obs {i+1} hit: d={t:.2f}m')
                intersection_distances.append((i+1, t))
        
        # Plot robot
        robot_size = 0.3
        robot_circle = Circle((self.robot_x, self.robot_y), robot_size, 
                            color='blue', alpha=0.7)
        ax.add_patch(robot_circle)
        
        # Plot ray extending to max range
        max_x = origin[0] + self.max_range * direction[0]
        max_y = origin[1] + self.max_range * direction[1]
        ax.plot([origin[0], max_x], [origin[1], max_y], 
               'b--', linewidth=1, alpha=0.3, label='Ray direction')
        
        # Plot ray to closest hit
        hit_point = self.hit_points[beam_index]
        ax.plot([origin[0], hit_point[0]], [origin[1], hit_point[1]], 
               'orange', linewidth=3, label=f'Closest hit: d={self.distances[beam_index]:.2f}m')
        ax.plot(hit_point[0], hit_point[1], 'o', color='orange', markersize=12)
        
        # Add direction vector annotation
        vec_scale = 2
        ax.arrow(self.robot_x, self.robot_y,
                vec_scale * direction[0], vec_scale * direction[1],
                head_width=0.3, head_length=0.2, fc='blue', ec='blue',
                linewidth=2, alpha=0.7)
        ax.text(self.robot_x + vec_scale * direction[0] * 1.3,
               self.robot_y + vec_scale * direction[1] * 1.3,
               f'd⃗=({direction[0]:.2f}, {direction[1]:.2f})',
               fontsize=11, color='blue',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
        
        # Add computation details text box
        details_text = f"Ray at angle: {np.degrees(angle):.1f}°\n"
        details_text += f"Direction: ({direction[0]:.3f}, {direction[1]:.3f})\n\n"
        details_text += "Intersections found:\n"
        for obs_id, dist in intersection_distances:
            details_text += f"  Obstacle {obs_id}: {dist:.2f}m\n"
        details_text += f"\nSelected: min = {self.distances[beam_index]:.2f}m"
        
        ax.text(0.02, 0.98, details_text,
               transform=ax.transAxes,
               fontsize=10,
               verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower right', fontsize=9)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Single Ray Detail: Beam {beam_index+1} (Finding Minimum Distance)')
        
        # Set axis limits
        margin = 2
        all_x = [self.robot_x] + [o[0] for o in self.obstacles]
        all_y = [self.robot_y] + [o[1] for o in self.obstacles]
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        plt.tight_layout()
        return fig


# Example usage
if __name__ == "__main__":
    # Define robot pose (x, y, yaw in radians)
    robot_pose = (0, 0, np.pi/4)  # 45 degrees
    
    # Define obstacles as (center_x, center_y, radius)
    obstacles = [
        (3, 2, 0.8),
        (4, -1, 0.6),
        (2, -2, 0.5),
        (-1, 3, 0.7),
        (5, 3, 0.9)
    ]
    
    # LiDAR configuration
    lidar_config = {
        'fov': np.pi,  # 180 degrees field of view
        'num_beams': 36,  # Number of beams
        'max_range': 10.0  # Maximum range in meters
    }
    
    # Create visualizer
    viz = RaycastingVisualizer(robot_pose, obstacles, lidar_config)
    
    # Generate plots
    print("Generating visualizations...")
    
    # 1. Single ray detail
    fig1 = viz.plot_single_ray_detail(beam_index=15)
    fig1.savefig('/mnt/user-data/outputs/raycasting_single_ray_detail.png', dpi=300, bbox_inches='tight')
    print("Saved: raycasting_single_ray_detail.png")
    
    # 2. Step-by-step progression
    fig2 = viz.plot_step_by_step(step_indices=[0, 8, 17, 35])
    fig2.savefig('/mnt/user-data/outputs/raycasting_steps.png', dpi=300, bbox_inches='tight')
    print("Saved: raycasting_steps.png")
    
    # 3. Complete scan view
    fig3, _ = viz.plot_static_view()
    fig3.savefig('/mnt/user-data/outputs/raycasting_complete.png', dpi=300, bbox_inches='tight')
    print("Saved: raycasting_complete.png")
    
    print("\nAll visualizations generated successfully!")
    print("These plots demonstrate:")
    print("1. Single ray detail - Shows how one ray finds the closest obstacle")
    print("2. Step-by-step - Shows progressive raycasting across beams")
    print("3. Complete scan - Shows all rays in final LiDAR scan")
