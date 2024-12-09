use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    input::mouse::{MouseMotion, MouseWheel},
    math::{
        bounding::{Bounded2d, IntersectsVolume},
        NormedVectorSpace,
    },
    prelude::*,
    sprite::Anchor, // window::PrimaryWindow,
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
        .add_plugins(FrameTimeDiagnosticsPlugin)
        .init_state::<PauseState>()
        .add_systems(Startup, (setup, setup_fps_counter))
        .add_systems(
            FixedUpdate,
            (
                move_particles,
                check_particle_collisions,
                check_wall_collisions,
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
                draw_distribution_overlay,
                update_fps,
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
}

#[derive(Resource)]
struct HistogramBins(Vec<f32>);

#[derive(Resource)]
struct MBDistribution(CubicCurve<f32>);

#[derive(Component)]
struct FpsText;

/* CONSTANTS */
// Particles
const PARTICLE_RADIUS: f32 = 5.0;
const PARTICLE_MASS: f32 = 1.;
const STARTING_SPEED: f32 = 200.0;
const NUMBER_OF_PARTICLES: u32 = 200;

// Parameters for the spawn grid
const SPAWN_TOP_LEFT: Vec2 = Vec2::new(-300., 250.);
const SPAWN_BOTTOM_RIGHT: Vec2 = Vec2::new(300., -100.);
const SPAWN_X_GAP: f32 = PARTICLE_RADIUS * 4.;
const SPAWN_Y_GAP: f32 = PARTICLE_RADIUS * 4.;

// Walls
const BOX_WIDTH: f32 = 1000.;
const BOX_HEIGHT: f32 = 700.;
const WALL_THICKNESS: f32 = 20.;

const HORI_OFFSET: f32 = BOX_WIDTH / 2.;
const VERT_OFFSET: f32 = (BOX_HEIGHT - WALL_THICKNESS) / 2.;
const LEFT_WALL_CENTER: Vec3 = Vec3::new(-HORI_OFFSET, 0., 0.);
const RIGHT_WALL_CENTER: Vec3 = Vec3::new(HORI_OFFSET, 0., 0.);
const TOP_WALL_CENTER: Vec3 = Vec3::new(0., VERT_OFFSET, 0.);
const BOTTOM_WALL_CENTER: Vec3 = Vec3::new(0., -VERT_OFFSET, 0.);

const BOX_BOTTOM_RIGHT: Vec3 = Vec3::new(
    HORI_OFFSET + WALL_THICKNESS / 2.,
    -VERT_OFFSET - WALL_THICKNESS / 2.,
    0.,
);
const BOX_TOP_RIGHT: Vec3 = Vec3::new(
    HORI_OFFSET + WALL_THICKNESS / 2.,
    VERT_OFFSET + WALL_THICKNESS / 2.,
    0.,
);

// Histogram
const BINS: u32 = 10;
const MAX_SPEED: f32 = STARTING_SPEED * 3.;
const BIN_WIDTH: f32 = MAX_SPEED / BINS as f32;

const BAR_WIDTH: f32 = 20.; // This is in world units, for the mesh geometry. Not to be confused with BIN_WIDTH.
const BAR_GAP: f32 = 10.;
const HEIGHT_PER_ELEM: f32 = 10.;
const LABEL_OFFSET: f32 = -30.;

const GAP_FROM_BOX: f32 = 20.;
const FIRST_BAR_INIITAL_CENTER: Vec3 = Vec3::new(
    BOX_BOTTOM_RIGHT.x + GAP_FROM_BOX + BAR_WIDTH / 2.,
    BOX_BOTTOM_RIGHT.y - LABEL_OFFSET + 10.,
    0.,
);
const LAST_BAR_INIITAL_CENTER: Vec3 = Vec3::new(
    FIRST_BAR_INIITAL_CENTER.x + (BINS as f32 - 1.) * (BAR_WIDTH + BAR_GAP),
    FIRST_BAR_INIITAL_CENTER.y,
    0.,
);
const HIST_WIDTH: f32 = LAST_BAR_INIITAL_CENTER.x - FIRST_BAR_INIITAL_CENTER.x + BAR_WIDTH;

// Physics
const BOLTZMANN_CONSTANT: f32 = 1.;

// Physically accurate values (need to implement a raycast check to avoid phasing through walls)
// const PARTICLE_MASS: f32 = 1.008 * ATOMIC_MASS_UNIT; // kg (mass of hydrogen-1)
// const STARTING_SPEED: f32 = 2599.0; // m/s (average speed of hydrogen at T = 273 K)
// const BOLTZMANN_CONSTANT: f32 = 1.38e-23;
// const ATOMIC_MASS_UNIT: f32 = 1.660e-27; // kg

/* SYSTEMS */
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    // mut window: Query<&mut Window, With<PrimaryWindow>>,
) {
    // window.single_mut().set_maximized(true);

    commands.spawn(Camera2d::default());

    let mesh_id = meshes.add(Circle::new(PARTICLE_RADIUS));
    let material_id = materials.add(ColorMaterial::from_color(Color::WHITE));

    // We need the square of the speeds to get the kinetic energy for temperature later
    let mut speeds_sq: Vec<f32> = Vec::new();

    let mut rng = rand::thread_rng();
    let mut spawn_point = SPAWN_TOP_LEFT;
    for _ in 0..NUMBER_OF_PARTICLES {
        let translation = Vec3::new(spawn_point.x, spawn_point.y, 0.);
        let velocity = Dir2::from_rng(&mut rng) * STARTING_SPEED;
        speeds_sq.push(velocity.norm_squared());

        commands.spawn((
            Particle {
                radius: PARTICLE_RADIUS,
                mass: PARTICLE_MASS,
                velocity,
            },
            Transform::from_translation(translation),
            Mesh2d(mesh_id.clone()),
            MeshMaterial2d(material_id.clone()),
        ));

        if spawn_point.x + SPAWN_X_GAP < SPAWN_BOTTOM_RIGHT.x {
            spawn_point.x += SPAWN_X_GAP;
        } else {
            spawn_point.y -= SPAWN_Y_GAP;
            spawn_point.x = SPAWN_TOP_LEFT.x;
        }
    }

    // Outer walls
    let horizontal_mesh = meshes.add(Rectangle::new(BOX_WIDTH - WALL_THICKNESS, WALL_THICKNESS));
    let vertical_mesh = meshes.add(Rectangle::new(WALL_THICKNESS, BOX_HEIGHT));
    let wall_material = materials.add(ColorMaterial::from_color(Color::BLACK));

    // Left wall
    commands.spawn((
        Wall {
            width: WALL_THICKNESS,
            height: BOX_HEIGHT,
            orientation: WallOrientation::Vertical,
        },
        Transform::from_translation(LEFT_WALL_CENTER),
        Mesh2d(vertical_mesh.clone()),
        MeshMaterial2d(wall_material.clone()),
    ));
    // Right wall
    commands.spawn((
        Wall {
            width: WALL_THICKNESS,
            height: BOX_HEIGHT,
            orientation: WallOrientation::Vertical,
        },
        Transform::from_translation(RIGHT_WALL_CENTER),
        Mesh2d(vertical_mesh),
        MeshMaterial2d(wall_material.clone()),
    ));
    // Top wall
    commands.spawn((
        Wall {
            width: BOX_WIDTH,
            height: WALL_THICKNESS,
            orientation: WallOrientation::Horizontal,
        },
        Transform::from_translation(TOP_WALL_CENTER),
        Mesh2d(horizontal_mesh.clone()),
        MeshMaterial2d(wall_material.clone()),
    ));
    // Bottom wall
    commands.spawn((
        Wall {
            width: BOX_WIDTH,
            height: WALL_THICKNESS,
            orientation: WallOrientation::Horizontal,
        },
        Transform::from_translation(BOTTOM_WALL_CENTER),
        Mesh2d(horizontal_mesh),
        MeshMaterial2d(wall_material),
    ));

    // Histogram
    let mut spawn_point = FIRST_BAR_INIITAL_CENTER;
    let bar_mesh = meshes.add(Rectangle::new(BAR_WIDTH, HEIGHT_PER_ELEM));
    let bar_color = materials.add(ColorMaterial::from_color(Srgba::rgb(0., 100., 100.)));

    let thresholds: Vec<f32> = (0..=BINS).map(|i| i as f32 * BIN_WIDTH as f32).collect();
    commands.insert_resource(HistogramBins(thresholds));

    commands.spawn((
        Text2d::new("2D Maxwell-Boltzmann\nspeed distribution"),
        Transform::from_translation(BOX_TOP_RIGHT + Vec3::new(GAP_FROM_BOX, 0., 0.)),
        Anchor::TopLeft,
    ));

    for i in 0..BINS {
        commands.spawn((
            HistogramBar { elems: 0 },
            Transform::from_translation(spawn_point),
            Mesh2d(bar_mesh.clone()),
            MeshMaterial2d(bar_color.clone()),
        ));

        if i % 2 == 0 {
            commands.spawn((
                Text2d::new(format!("{}", i as f32 * BIN_WIDTH)),
                Transform::from_xyz(spawn_point.x, spawn_point.y + LABEL_OFFSET, spawn_point.z),
            ));
        }

        spawn_point.x += BAR_WIDTH + BAR_GAP;
    }
    commands.spawn((
        Text2d::new(format!("{}", BINS as f32 * BIN_WIDTH)),
        Transform::from_xyz(spawn_point.x, spawn_point.y + LABEL_OFFSET, spawn_point.z),
    ));

    // Maxwell-Boltzmann distribution overlay
    // Temperature comes from the equipartition theorem for a 2D monoatomic ideal gas. We know the
    // kinetic energies, so we can reverse it to get the temperature of the system.
    let avg_kinetic_energy =
        PARTICLE_MASS / (2. * NUMBER_OF_PARTICLES as f32) * speeds_sq.iter().sum::<f32>();
    let temp = avg_kinetic_energy / BOLTZMANN_CONSTANT;
    info!("Temperature: {temp}");

    let sampling_points: Vec<f32> = (0..20).map(|i| i as f32 * MAX_SPEED / 20.).collect();
    let pdf_points: Vec<f32> = sampling_points
        .iter()
        .map(|v| maxwell_boltzmann_2d_pdf(*v, PARTICLE_MASS, temp))
        .collect();
    let pdf_curve = CubicCardinalSpline::new(0.5, pdf_points)
        .to_curve()
        .unwrap();
    commands.insert_resource(MBDistribution(pdf_curve));
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
        transform.translation.y += (change as f32 * HEIGHT_PER_ELEM) / 2.;
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

fn draw_distribution_overlay(mb_distr: Res<MBDistribution>, mut gizmos: Gizmos) {
    let curve = &mb_distr.0;
    let resolution = 100 * curve.segments().len();
    let starting_point = Vec2::new(
        FIRST_BAR_INIITAL_CENTER.x - BAR_WIDTH / 2.,
        FIRST_BAR_INIITAL_CENTER.y,
    );
    let points: Vec<Vec2> = curve
        .iter_positions(resolution)
        .enumerate()
        .map(|(i, p)| {
            // Probability needs to be weight by total number and bin width to bring it in histogram units
            let predicted_elems = p * NUMBER_OF_PARTICLES as f32 * BIN_WIDTH;
            Vec2::new(
                starting_point.x + HIST_WIDTH / resolution as f32 * i as f32,
                starting_point.y + predicted_elems * HEIGHT_PER_ELEM,
            )
        })
        .collect();

    gizmos.linestrip_2d(points, Srgba::rgb(100., 0., 100.));
}

fn setup_fps_counter(mut commands: Commands) {
    commands
        .spawn((
            Node {
                height: Val::Px(30.),
                width: Val::Px(40.),
                ..Default::default()
            },
            BackgroundColor(Color::BLACK),
        ))
        .with_child((Text("FPS".into()), FpsText));
}

fn update_fps(diagnostics: Res<DiagnosticsStore>, mut query: Query<&mut Text, With<FpsText>>) {
    let maybe_text = query.get_single_mut();
    if let Some(fps) = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|fps| fps.smoothed())
    {
        if let Ok(mut text) = maybe_text {
            text.0 = format!("{fps:.0}");
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

/// The probability density function for a 2D Maxwell-Boltzmann distribution.
fn maxwell_boltzmann_2d_pdf(speed: f32, mass: f32, temperature: f32) -> f32 {
    let a_sq = BOLTZMANN_CONSTANT * temperature / mass;
    let speed_sq = speed.powi(2);
    return speed / a_sq * (-speed_sq / (2. * a_sq)).exp();
}
