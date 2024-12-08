use bevy::{
    math::{
        bounding::{Bounded2d, IntersectsVolume},
        NormedVectorSpace,
    },
    prelude::*,
    window::PrimaryWindow,
};
use rand::thread_rng;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Gas simulation".into(),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (
                move_particles,
                check_particle_collisions,
                check_wall_collisions,
            )
                .chain(),
        )
        .add_systems(Update, draw_gizmos)
        .run();
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

/* SYSTEMS */

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut window: Query<&mut Window, With<PrimaryWindow>>,
) {
    window.single_mut().set_maximized(true);

    commands.spawn(Camera2d::default());

    let radius = 10.0;
    let number_of_particles = 10;

    let mesh_id = meshes.add(Circle::new(radius));
    let material_id = materials.add(ColorMaterial::from_color(Color::WHITE));
    let mut rng = thread_rng();
    for _ in 0..number_of_particles {
        let trans2 = Dir2::from_rng(&mut rng) * 200.;
        let translation = Vec3::new(trans2.x, trans2.y, 0.);
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
    }

    // commands.spawn((
    //     Particle {
    //         radius,
    //         mass: 1.0,
    //         velocity: Vec2 { x: -200., y: 0. },
    //     },
    //     Transform::from_translation(Vec3 {
    //         x: 200.,
    //         y: 100.,
    //         z: 0.,
    //     }),
    //     Mesh2d(mesh_id),
    //     MeshMaterial2d(materials.add(ColorMaterial::from_color(Color::linear_rgb(0., 0., 200.)))),
    // ));

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
}

/// Move particles one time step.
fn move_particles(time: Res<Time>, mut query: Query<(&Particle, &mut Transform)>) {
    for (particle, mut transform) in &mut query {
        transform.translation.x += particle.velocity.x * time.delta_secs();
        transform.translation.y += particle.velocity.y * time.delta_secs();
    }
}

/// Handle collisions between particles.
fn check_particle_collisions(mut query: Query<(&mut Particle, &Transform)>) {
    let mut combinations = query.iter_combinations_mut();
    while let Some([(mut particle1, transform1), (mut particle2, transform2)]) =
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

            let delta_v = compute_velocity_delta(x1, x2, v1, v2, m1, m2);
            particle1.velocity += delta_v;
            particle2.velocity -= delta_v;
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

fn draw_gizmos(mut gizmos: Gizmos) {
    gizmos.rect_2d(
        Isometry2d::IDENTITY,
        Vec2 { x: 1000., y: 700. },
        Color::BLACK,
    );
}

/* UTILITY FUNCTIONS */
fn compute_velocity_delta(x1: Vec2, x2: Vec2, v1: Vec2, v2: Vec2, m1: f32, m2: f32) -> Vec2 {
    let total_m = m1 + m2;
    let delta_v = v1 - v2;
    let delta_x = x1 - x2;

    return -2.0 * m2 / total_m * delta_v.dot(delta_x) / delta_x.norm_squared() * delta_x;
}
