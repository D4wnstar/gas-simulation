use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    math::{
        bounding::{Bounded2d, IntersectsVolume},
        NormedVectorSpace,
    },
    prelude::*, // window::PrimaryWindow,
};
use rand;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Gas simulation".into(),
                ..default()
            }),
            ..default()
        }))
        .init_state::<PauseState>()
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (
                move_particles,
                check_particle_collisions,
                check_wall_collisions,
                // unstuck_particles,
            )
                .chain()
                .run_if(in_state(PauseState::Running)),
        )
        .add_systems(
            Update,
            (
                check_keyboard_input,
                drag_camera,
                update_histogram.run_if(in_state(PauseState::Running)),
            ),
        )
        .run();
}

#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
enum PauseState {
    #[default]
    Paused,
    Running,
}

#[derive(Component, Default)]
#[require(Transform, Mesh2d, MeshMaterial2d<ColorMaterial>)]
struct Particle {
    radius: f32,
    mass: f32,
    velocity: Vec2,
}

enum WallOrientation {
    Vertical,
    Horizontal,
}

#[derive(Component)]
#[require(Transform, Mesh2d, MeshMaterial2d<ColorMaterial>)]
struct Wall {
    width: f32,
    height: f32,
    orientation: WallOrientation,
}

#[derive(Component)]
struct HistogramBar {
    elems: u32,
    height_per_elem: f32,
}

#[derive(Resource)]
struct HistogramBins(Vec<f32>);

const BINS: u32 = 10;
const MAX_SPEED: f32 = 600.;

/* SYSTEMS */
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    // mut window: Query<&mut Window, With<PrimaryWindow>>,
) {
    // window.single_mut().set_maximized(true);

    commands.spawn(Camera2d::default());

    let radius = 10.0;

    let number_of_particles = 100;

    // Parameters for the spawn grid
    let top_left = Vec2::new(-300., 100.);
    let bottom_right = Vec2::new(300., -100.);
    let mut spawn_point = top_left;
    let x_gap = radius * 4.;
    let y_gap = radius * 4.;

    let mesh_id = meshes.add(Circle::new(radius));
    let material_id = materials.add(ColorMaterial::from_color(Color::WHITE));
    let mut rng = rand::thread_rng();
    for _ in 0..number_of_particles {
        let translation = Vec3::new(spawn_point.x, spawn_point.y, 0.);
        let velocity = Dir2::from_rng(&mut rng) * 200.;

        commands.spawn((
            Particle {
                radius,
                mass: 1.0,
                velocity,
            },
            Transform::from_translation(translation),
            Mesh2d(mesh_id.clone()),
            MeshMaterial2d(material_id.clone()),
        ));

        if spawn_point.x + x_gap < bottom_right.x {
            spawn_point.x += x_gap;
        } else {
            spawn_point.y -= y_gap;
            spawn_point.x = top_left.x;
        }
    }

    // Outer walls
    let thickness = 20.;
    let hori_width = 1000. - thickness;
    let vert_height = 700.;

    let horizontal_mesh = meshes.add(Rectangle::new(hori_width, thickness));
    let vertical_mesh = meshes.add(Rectangle::new(thickness, vert_height));
    let wall_material = materials.add(ColorMaterial::from_color(Color::BLACK));

    // Left wall
    commands.spawn((
        Wall {
            width: thickness,
            height: vert_height,
            orientation: WallOrientation::Vertical,
        },
        Transform::from_xyz(-500., 0., 0.),
        Mesh2d(vertical_mesh.clone()),
        MeshMaterial2d(wall_material.clone()),
    ));
    // Right wall
    commands.spawn((
        Wall {
            width: thickness,
            height: vert_height,
            orientation: WallOrientation::Vertical,
        },
        Transform::from_xyz(500., 0., 0.),
        Mesh2d(vertical_mesh),
        MeshMaterial2d(wall_material.clone()),
    ));
    // Top wall
    commands.spawn((
        Wall {
            width: hori_width,
            height: thickness,
            orientation: WallOrientation::Horizontal,
        },
        Transform::from_xyz(0., 340., 0.),
        Mesh2d(horizontal_mesh.clone()),
        MeshMaterial2d(wall_material.clone()),
    ));
    // Bottom wall
    commands.spawn((
        Wall {
            width: hori_width,
            height: thickness,
            orientation: WallOrientation::Horizontal,
        },
        Transform::from_xyz(0., -340., 0.),
        Mesh2d(horizontal_mesh),
        MeshMaterial2d(wall_material),
    ));

    // Histogram
    let mut spawn_point = Vec3::new(530., -100., 0.);
    let bar_width = 20.;
    let height_per_elem = 10.;
    let bar_mesh = meshes.add(Rectangle::new(20., height_per_elem));
    let bar_color = materials.add(ColorMaterial::from_color(Srgba::rgb(0., 100., 100.)));

    let thresholds: Vec<f32> = (0..=BINS)
        .map(|i| i as f32 * MAX_SPEED / BINS as f32)
        .collect();
    commands.insert_resource(HistogramBins(thresholds));

    // commands.spawn((Text2d::new("Hi!!!!"), Transform::from_xyz(550., 100., 0.)));

    for i in 0..BINS {
        commands.spawn((
            HistogramBar {
                elems: 0,
                height_per_elem,
            },
            Transform::from_translation(spawn_point),
            Mesh2d(bar_mesh.clone()),
            MeshMaterial2d(bar_color.clone()),
        ));

        commands.spawn((
            Text2d::new(format!("{i}")),
            Transform::from_xyz(spawn_point.x, spawn_point.y - 30., spawn_point.z),
        ));

        spawn_point.x += bar_width * 1.5;
    }
}

fn update_histogram(
    mut bar_query: Query<(&mut Transform, &mut HistogramBar)>,
    p_query: Query<&Particle>,
    thresholds: Res<HistogramBins>,
) {
    // Initialize array of bin contents
    let mut bins: Vec<u32> = Vec::new();
    for _ in 0..thresholds.0.len() {
        bins.push(0);
    }

    // Populate histogram bins
    for particle in &p_query {
        let speed = particle.velocity.norm();
        for i in 0..(thresholds.0.len() - 1) {
            if speed > thresholds.0[i] && speed < thresholds.0[i + 1] {
                bins[i] += 1;
                break;
            }
        }
    }

    // Update the histogram meshes. Assumes the number of meshes/bars is thresholds.0.len() - 1
    for (i, (mut transform, mut bar)) in bar_query.iter_mut().enumerate() {
        let change = bins[i] as i32 - bar.elems as i32;
        bar.elems = bins[i];
        transform.scale.y = bar.elems as f32;
        transform.translation.y += (change as f32 * bar.height_per_elem) / 2.;
    }
}

/// Move particles one time step.
fn move_particles(time: Res<Time>, mut query: Query<(&Particle, &mut Transform)>) {
    for (particle, mut transform) in &mut query {
        transform.translation.x += particle.velocity.x * time.delta_secs();
        transform.translation.y += particle.velocity.y * time.delta_secs();
    }
}

/// Handle collisions between particles.
fn check_particle_collisions(mut query: Query<(&mut Particle, &mut Transform)>) {
    let mut combinations = query.iter_combinations_mut();
    while let Some([(mut particle1, mut transform1), (mut particle2, mut transform2)]) =
        combinations.fetch_next()
    {
        let x1 = transform1.translation.xy();
        let x2 = transform2.translation.xy();
        let isometry1 = Isometry2d::new(x1, Rot2::IDENTITY);
        let isometry2 = Isometry2d::new(x2, Rot2::IDENTITY);
        let circle1 = Circle::new(particle1.radius).bounding_circle(isometry1);
        let circle2 = Circle::new(particle2.radius).bounding_circle(isometry2);

        if circle1.intersects(&circle2) {
            let v1 = particle1.velocity;
            let v2 = particle2.velocity;
            let m1 = particle1.mass;
            let m2 = particle2.mass;

            // Calculate the change in velocity due to an elastic collision
            let delta_v = compute_velocity_delta(x1, x2, v1, v2, m1, m2);
            particle1.velocity += delta_v;
            particle2.velocity -= delta_v;

            // "Unstuck" particles by moving them so that they do not overlap
            let shift = compute_overlap(x1, x2, particle1.radius, particle2.radius);
            transform1.translation += shift / 2.;
            transform2.translation -= shift / 2.;
        }
    }
}

/// Handle collisions between particles and walls.
fn check_wall_collisions(
    mut particles: Query<(&mut Particle, &Transform)>,
    mut walls: Query<(&Wall, &Transform)>,
) {
    for (mut particle, p_transform) in &mut particles {
        for (wall, w_transform) in &mut walls {
            let p_pos = p_transform.translation.xy();
            let w_pos = w_transform.translation.xy();
            let p_iso = Isometry2d::new(p_pos, Rot2::IDENTITY);
            let w_iso = Isometry2d::new(w_pos, Rot2::IDENTITY);
            let p_hitbox = Circle::new(particle.radius).bounding_circle(p_iso);
            let w_hitbox = Rectangle::new(wall.width, wall.height).aabb_2d(w_iso);

            if p_hitbox.intersects(&w_hitbox) {
                // Since walls are immovable objects, we just need to flip
                // the velocity in the right direction when a particle hits one
                match wall.orientation {
                    WallOrientation::Horizontal => particle.velocity.y = -particle.velocity.y,
                    WallOrientation::Vertical => particle.velocity.x = -particle.velocity.x,
                }
            }
        }
    }
}

fn check_keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<PauseState>>,
    mut next_state: ResMut<NextState<PauseState>>,
) {
    if keys.just_pressed(KeyCode::Space) {
        match state.get() {
            PauseState::Paused => next_state.set(PauseState::Running),
            PauseState::Running => next_state.set(PauseState::Paused),
        }
    }
}

/// Handle camera movement.
fn drag_camera(
    buttons: Res<ButtonInput<MouseButton>>,
    mut evread_motion: EventReader<MouseMotion>,
    mut evread_scroll: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut OrthographicProjection), With<Camera2d>>,
) {
    let (mut cam_transform, mut proj) = query.single_mut();
    if buttons.pressed(MouseButton::Left) {
        for ev in evread_motion.read() {
            // Weigh camera drag by the current projection scale (i.e. zoom) so that it feels
            // the same at every zoom level
            let delta_x = -2. * proj.scale * ev.delta.x;
            let delta_y = 2. * proj.scale * ev.delta.y;
            cam_transform.translation += Vec3::new(delta_x, delta_y, 0.);
        }
    }

    for ev in evread_scroll.read() {
        if ev.y > 0. {
            // Scroll up -> Zoom in
            proj.scale *= 0.95
        } else {
            // Scroll down -> Zoom out
            proj.scale *= 1.05
        }
    }
}

/* UTILITY FUNCTIONS */
fn compute_velocity_delta(x1: Vec2, x2: Vec2, v1: Vec2, v2: Vec2, m1: f32, m2: f32) -> Vec2 {
    let total_m = m1 + m2;
    let delta_v = v1 - v2;
    let delta_x = x1 - x2;

    return -2.0 * m2 / total_m * delta_v.dot(delta_x) / delta_x.norm_squared() * delta_x;
}

fn compute_overlap(x1: Vec2, x2: Vec2, radius1: f32, radius2: f32) -> Vec3 {
    let distance_between_centers = x1 - x2;
    let distance = distance_between_centers.norm().max(0.);
    let overlap = radius1 + radius2 - distance;
    let overlap_vec = overlap * distance_between_centers / distance;
    return Vec3::new(overlap_vec.x, overlap_vec.y, 0.);
}
