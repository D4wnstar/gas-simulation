use std::f32::consts::PI;

use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    input::mouse::{MouseMotion, MouseWheel},
    math::{
        bounding::{Bounded2d, IntersectsVolume},
        NormedVectorSpace,
    },
    prelude::*,
    sprite::Anchor,
    window::PrimaryWindow,
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
        .insert_resource(Time::<Fixed>::from_hz(256.))
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
                (update_temperature_text, update_entropy_text)
                    .chain()
                    .run_if(in_state(PauseState::Running)),
                update_slow_mo_text,
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
struct TemperatureText;

#[derive(Resource)]
struct Temperature(f32);

#[derive(Component)]
struct EntropyText;

#[derive(Resource)]
struct Entropy(f32);

#[derive(Component)]
struct SlowMoText;

#[derive(Resource)]
struct SlowMo(f32);

#[derive(Component)]
struct FpsText;

/* CONSTANTS */
// Particles
const PARTICLE_RADIUS: f32 = 5.0;
// const PARTICLE_MASS: f32 = 1.;
// const STARTING_SPEED: f32 = 200.0; // Velocity direction is randomized
const NUMBER_OF_PARTICLES: u32 = 400;
// Temperature comes from the equipartition theorem for a 2D monatomic ideal gas. We know the
// the speeds and therefore the kinetic energies, so we can reverse it to get the temperature of the system.
// TODO: Calculate temperature in real time to show movement to lowest energy state. Same goes for entropy.
const AVG_KINETIC_ENERGY: f32 = PARTICLE_MASS / 2. * STARTING_SPEED * STARTING_SPEED;
const TEMPERATURE: f32 = AVG_KINETIC_ENERGY / BOLTZMANN_CONSTANT;

// Parameters for the spawn grid
const SPAWN_TOP_LEFT: Vec2 = Vec2::new(
    BOX_TOP_LEFT.x + 2. * WALL_THICKNESS,
    BOX_TOP_LEFT.y - 2. * WALL_THICKNESS,
);
const SPAWN_BOTTOM_RIGHT: Vec2 = Vec2::new(
    BOX_BOTTOM_RIGHT.x - 2. * WALL_THICKNESS,
    BOX_BOTTOM_RIGHT.y + 2. * WALL_THICKNESS,
);
const SPAWN_X_GAP: f32 = PARTICLE_RADIUS * 4.;
const SPAWN_Y_GAP: f32 = PARTICLE_RADIUS * 4.;

// Walls
const BOX_WIDTH: f32 = 1000.;
const BOX_HEIGHT: f32 = 700.;
const WALL_THICKNESS: f32 = 10.;
const BOX_AREA: f32 = (BOX_WIDTH - WALL_THICKNESS) * (BOX_HEIGHT - WALL_THICKNESS);

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
const BOX_BOTTOM_LEFT: Vec3 = Vec3::new(
    -HORI_OFFSET - WALL_THICKNESS / 2.,
    -VERT_OFFSET - WALL_THICKNESS / 2.,
    0.,
);
const BOX_TOP_RIGHT: Vec3 = Vec3::new(
    HORI_OFFSET + WALL_THICKNESS / 2.,
    VERT_OFFSET + WALL_THICKNESS / 2.,
    0.,
);
const BOX_TOP_LEFT: Vec3 = Vec3::new(
    -HORI_OFFSET - WALL_THICKNESS / 2.,
    VERT_OFFSET + WALL_THICKNESS / 2.,
    0.,
);

// Histogram
const BINS: u32 = 10;
const MAX_SPEED: f32 = STARTING_SPEED * 3.;
const BIN_WIDTH: f32 = MAX_SPEED / BINS as f32;

const BAR_WIDTH: f32 = 20.; // This is in world units, for the mesh geometry. Not to be confused with BIN_WIDTH.
const HIST_HEIGHT: f32 = BOX_HEIGHT - 100.;
const BAR_GAP: f32 = 10.;
const HEIGHT_PER_ELEM: f32 = HIST_HEIGHT / (0.3 * NUMBER_OF_PARTICLES as f32);
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
// const BOLTZMANN_CONSTANT: f32 = 1.;
// const REDUCED_PLANCK_CONSTANT: f32 = 1.;

// Physically accurate values (need to implement a raycast check to avoid phasing through walls)
const PARTICLE_MASS: f32 = 4.002 * ATOMIC_MASS_UNIT; // kg (mass of helium-4)
const STARTING_SPEED: f32 = 1103.3; // m/s (average speed of helium at T = 293 K)
const BOLTZMANN_CONSTANT: f32 = 1.38e-23;
const REDUCED_PLANCK_CONSTANT: f32 = 1.055e-34; // Js
const ATOMIC_MASS_UNIT: f32 = 1.660e-27; // kg
const ELEMENTARY_CHARGE: f32 = 1.602e-19; // C, to convert to electronvolts

/* SYSTEMS */
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut window: Query<&mut Window, With<PrimaryWindow>>,
) {
    window.single_mut().set_maximized(true);

    commands.spawn((Camera2d::default(), Transform::from_xyz(160., 0., 0.)));

    let mesh_id = meshes.add(Circle::new(PARTICLE_RADIUS));
    let material_id = materials.add(ColorMaterial::from_color(Color::WHITE));

    let mut rng = rand::thread_rng();
    let mut spawn_point = SPAWN_TOP_LEFT;
    for _ in 0..NUMBER_OF_PARTICLES {
        let translation = Vec3::new(spawn_point.x, spawn_point.y, 0.);
        let velocity = Dir2::from_rng(&mut rng) * STARTING_SPEED;

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
        Text2d::new("2D Maxwell-Boltzmann\nspeed distribution\n(in number of particles)"),
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
                Text2d::new(format!("{:.0}", i as f32 * BIN_WIDTH)),
                Transform::from_xyz(spawn_point.x, spawn_point.y + LABEL_OFFSET, spawn_point.z),
            ));
        }

        spawn_point.x += BAR_WIDTH + BAR_GAP;
    }
    commands.spawn((
        Text2d::new(format!("{:.0}", BINS as f32 * BIN_WIDTH)),
        Transform::from_xyz(spawn_point.x, spawn_point.y + LABEL_OFFSET, spawn_point.z),
    ));
    commands.spawn((
        Text2d::new(format!("speed [m/s]")),
        Transform::from_xyz(
            BOX_BOTTOM_RIGHT.x + (LAST_BAR_INIITAL_CENTER - FIRST_BAR_INIITAL_CENTER).x / 2.,
            spawn_point.y + LABEL_OFFSET * 2.,
            spawn_point.z,
        ),
    ));

    // Maxwell-Boltzmann distribution overlay
    let sampling_points: Vec<f32> = (0..20).map(|i| i as f32 * MAX_SPEED / 20.).collect();
    let pdf_points: Vec<f32> = sampling_points
        .iter()
        .map(|v| maxwell_boltzmann_2d_pdf(*v, PARTICLE_MASS, TEMPERATURE))
        .collect();
    let pdf_curve = CubicCardinalSpline::new(0.5, pdf_points)
        .to_curve()
        .unwrap();
    commands.insert_resource(MBDistribution(pdf_curve));

    // Info text
    let speeds: Vec<f32> = vec![STARTING_SPEED; NUMBER_OF_PARTICLES as usize];
    let temperature = calculate_temperature(PARTICLE_MASS, NUMBER_OF_PARTICLES as f32, speeds);
    commands.spawn((
        Text2d::new(format!("Temperature: {TEMPERATURE:.1} K")),
        TemperatureText,
        Transform::from_translation(BOX_BOTTOM_RIGHT),
        Anchor::TopRight,
    ));
    commands.insert_resource(Temperature(temperature));

    let entropy = calculate_entropy(PARTICLE_MASS, NUMBER_OF_PARTICLES as f32, temperature);
    commands.spawn((
        Text2d::new(format!("Entropy: {:.2} eV/K", entropy / ELEMENTARY_CHARGE)),
        EntropyText,
        Transform::from_translation(BOX_BOTTOM_RIGHT - Vec3::new(0., 20., 0.)),
        Anchor::TopRight,
    ));
    commands.insert_resource(Entropy(entropy));

    commands.spawn((
        Text2d::new(format!("Box dimensions: {BOX_WIDTH} m x {BOX_HEIGHT} m")),
        Transform::from_translation(BOX_BOTTOM_LEFT),
        Anchor::TopLeft,
    ));

    let slow_mo = SlowMo(1.);
    commands.spawn((
        SlowMoText,
        Text2d::new(format!(
            "Slow motion: {}\n(Up/Down arrow to change, Spacebar to pause)",
            slow_mo.0
        )),
        Transform::from_translation(BOX_BOTTOM_LEFT - Vec3::new(0., 20., 0.)),
        Anchor::TopLeft,
    ));
    commands.insert_resource(slow_mo);

    commands.spawn((
        Text2d::new(format!(
            "Left click and drag to move camera, scroll to zoom"
        )),
        Transform::from_translation(BOX_TOP_LEFT),
        Anchor::BottomLeft,
    ));
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
fn move_particles(
    time: Res<Time>,
    mut query: Query<(&Particle, &mut Transform)>,
    slow_mo: Res<SlowMo>,
) {
    for (particle, mut transform) in &mut query {
        transform.translation.x += particle.velocity.x * time.delta_secs() / slow_mo.0;
        transform.translation.y += particle.velocity.y * time.delta_secs() / slow_mo.0;
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

            // Calculate the change in velocity due to an elastic collision
            let delta_v = compute_velocity_delta(x1, x2, v1, v2);
            particle1.velocity += delta_v;
            particle2.velocity -= delta_v;

            // "Unstuck" particles by moving them so that they do not overlap
            let shift = compute_particle_overlap(x1, x2, particle1.radius, particle2.radius);
            transform1.translation += shift / 2.;
            transform2.translation -= shift / 2.;
        }
    }
}

/// Handle collisions between particles and walls.
fn check_wall_collisions(
    mut particles: Query<(&mut Particle, &mut Transform)>,
    mut walls: Query<(&Wall, &Transform), Without<Particle>>,
) {
    for (mut particle, mut p_transform) in &mut particles {
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

                // "Unstuck" particles by moving them so that they do not overlap
                let shift = compute_wall_overlap(
                    p_pos,
                    w_pos,
                    particle.radius,
                    WALL_THICKNESS,
                    &wall.orientation,
                );
                p_transform.translation -= shift;
            }
        }
    }
}

fn check_keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<PauseState>>,
    mut next_state: ResMut<NextState<PauseState>>,
    mut slow_mo: ResMut<SlowMo>,
) {
    if keys.just_pressed(KeyCode::Space) {
        match state.get() {
            PauseState::Paused => next_state.set(PauseState::Running),
            PauseState::Running => next_state.set(PauseState::Paused),
        }
    }
    if keys.just_pressed(KeyCode::ArrowUp) {
        slow_mo.0 += 1.;
    }
    if keys.just_pressed(KeyCode::ArrowDown) && slow_mo.0 >= 2. {
        slow_mo.0 -= 1.;
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

fn update_temperature_text(
    mut text_query: Query<&mut Text2d, With<TemperatureText>>,
    particle_query: Query<&Particle>,
    mut temperature: ResMut<Temperature>,
) {
    let mut text = text_query.single_mut();
    let speeds: Vec<f32> = particle_query.iter().map(|p| p.velocity.norm()).collect();
    let new_temp = calculate_temperature(PARTICLE_MASS, NUMBER_OF_PARTICLES as f32, speeds);
    temperature.0 = new_temp;
    text.0 = format!("Temperature: {:.1} K", new_temp);
}

fn update_entropy_text(
    mut text_query: Query<&mut Text2d, With<EntropyText>>,
    temperature: Res<Temperature>,
    mut entropy: ResMut<Entropy>,
) {
    let mut text = text_query.single_mut();
    let new_entr = calculate_entropy(PARTICLE_MASS, NUMBER_OF_PARTICLES as f32, temperature.0)
        / ELEMENTARY_CHARGE;
    entropy.0 = new_entr;
    text.0 = format!("Entropy: {:.2} eV/K", new_entr);
}

fn update_slow_mo_text(mut query: Query<&mut Text2d, With<SlowMoText>>, slow_mo: Res<SlowMo>) {
    let mut text = query.single_mut();
    text.0 = format!(
        "Slow motion: {}\n(Up/Down arrow to change, Spacebar to pause)",
        slow_mo.0
    );
}

/* UTILITY FUNCTIONS */
/// Computes the velocity difference after an elastic collision of two rigid spheres of equal mass.
fn compute_velocity_delta(x1: Vec2, x2: Vec2, v1: Vec2, v2: Vec2) -> Vec2 {
    let delta_v = v1 - v2;
    let delta_x = x1 - x2;

    return -delta_v.dot(delta_x) / delta_x.norm_squared() * delta_x;
}

/// Computes the velocity difference after an elastic collision of two rigid spheres of different mass.
// fn compute_velocity_delta_masses(x1: Vec2, x2: Vec2, v1: Vec2, v2: Vec2, m1: f32, m2: f32) -> Vec2 {
//     let total_m = m1 + m2;
//     let delta_v = v1 - v2;
//     let delta_x = x1 - x2;

//     return -2.0 * m2 / total_m * delta_v.dot(delta_x) / delta_x.norm_squared() * delta_x;
// }

/// Compute the vector that describes the overlap between two intersecting spheres.
/// The direction of the vector is `x2` towards `x1`.
fn compute_particle_overlap(x1: Vec2, x2: Vec2, radius1: f32, radius2: f32) -> Vec3 {
    let distance_between_centers = x1 - x2;
    let distance = distance_between_centers.norm().max(0.);
    let overlap = radius1 + radius2 - distance;
    let overlap_vec = overlap * distance_between_centers / distance;
    return Vec3::new(overlap_vec.x, overlap_vec.y, 0.);
}

/// Compute the vector that describes the overlap between a sphere intersecting an axis-aligned rectangle.
/// Direction of the vector is towards the wall.
fn compute_wall_overlap(
    part_center: Vec2,
    wall_center: Vec2,
    radius: f32,
    wall_thickness: f32,
    wall_orientation: &WallOrientation,
) -> Vec3 {
    // Relies on walls being on opposite sides of the origin
    let half_thickness = wall_thickness / 2.;
    match wall_orientation {
        WallOrientation::Horizontal => {
            let y_overlap = if wall_center.y > 0. {
                // Top wall
                (part_center.y + radius) - (wall_center.y - half_thickness)
            } else {
                // Bottom wall
                -(wall_center.y + half_thickness) + (part_center.y - radius)
            };
            return Vec3::new(0., y_overlap, 0.);
        }
        WallOrientation::Vertical => {
            let x_overlap = if wall_center.x > 0. {
                // Right wall
                (part_center.x + radius) - (wall_center.x - half_thickness)
            } else {
                // Left wall
                -(wall_center.x + half_thickness) + (part_center.x - radius)
            };
            return Vec3::new(x_overlap, 0., 0.);
        }
    }
}

/// The probability density function for a 2D Maxwell-Boltzmann distribution.
fn maxwell_boltzmann_2d_pdf(speed: f32, mass: f32, temperature: f32) -> f32 {
    let a_sq = BOLTZMANN_CONSTANT * temperature / mass;
    let speed_sq = speed.powi(2);
    return speed / a_sq * (-speed_sq / (2. * a_sq)).exp();
}

/// Calculate the system temperature from the particle velocities. Assumes free particles.
fn calculate_temperature(particle_mass: f32, number_of_particles: f32, speeds: Vec<f32>) -> f32 {
    let avg_kinetic_energy =
        particle_mass / 2. * speeds.iter().map(|v| v * v).sum::<f32>() / number_of_particles;
    return avg_kinetic_energy / BOLTZMANN_CONSTANT;
}

/// Calculate the system entropy from the temperature using the Sackur-Tetrode equation for a 2D monatomic ideal gas.
fn calculate_entropy(particle_mass: f32, number_of_particles: f32, temperature: f32) -> f32 {
    // Order of magnitude is calculated manually to avoid floating point underflow
    // e-34 * e-34 / (e-27 * e-23) = e-68 / e-50 = e-14
    let de_broglie_thermal_wavelength_square =
        2. * PI * (REDUCED_PLANCK_CONSTANT * 1e34) * (REDUCED_PLANCK_CONSTANT * 1e34)
            / (particle_mass * 1e27 * BOLTZMANN_CONSTANT * 1e23 * temperature)
            * 1e-14;
    let particle_density = number_of_particles / BOX_AREA;

    return BOLTZMANN_CONSTANT
        * number_of_particles
        * (5. / 2. - ops::ln(particle_density * de_broglie_thermal_wavelength_square));
}
