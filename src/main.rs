use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_inspector_egui::WorldInspectorPlugin;
use smooth_bevy_cameras::{controllers::fps::{FpsCameraBundle, FpsCameraController, FpsCameraPlugin}, LookTransformPlugin};
use bevy::render::primitives::Aabb;
use bevy_prototype_debug_lines::*;
use bevy::render::mesh::{self, PrimitiveTopology};
use std::f32::consts::PI;
use bevy::math::const_vec3;
use itertools::Itertools;
use rand::distributions::Uniform;
use rand::{Rng, thread_rng};
use bevy_rapier3d::math::Rot;

struct Field(Aabb);

fn main() {
    let field_width = 200.0;
    let field = Vec3::splat(field_width/2.0);
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(FpsCameraPlugin::default())
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(LookTransformPlugin)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .insert_resource(RapierConfiguration{
            gravity: Vec3::new(0.0, 0.0, 0.0),
            ..Default::default()
        })
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_system(cursor_grab_system)
        .add_system(draw_field_system)
        .add_system(keep_inside_field)
        //.add_system(debug_draw_boid)
        .add_system(boid_look_at_velocity)
        .insert_resource(Field(Aabb::from_min_max(-field, field)))
        .add_startup_system(spawn_boids)
        .add_system(update_boids)
        .insert_resource(BoidSettings::default())
        .add_system(enforce_min_speed.after(update_boids))
        .run();
}

fn enforce_min_speed(
    boid_settings: Res<BoidSettings>,
    mut boids: Query<&mut Velocity, With<Boid>>
) {
    for mut velocity in boids.iter_mut() {
        if velocity.linvel.length_squared() == 0.0 {
            velocity.linvel = Vec3::new(0.1, 0.0, 0.0);   
        } 
        velocity.linvel = velocity.linvel.clamp_length_min(boid_settings.min_speed);
    }
}

struct BoidSettings {
    view_radius: f32,
    avoid_raduis: f32,
    max_speed: f32,
    min_speed: f32,
    max_steer_force: f32,

    alignment_weight: f32,
    cohesion_weight: f32,
    seperation_weight: f32,
    boid_count: u64,
}

impl Default for BoidSettings {
    fn default() -> Self {
        BoidSettings {
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            seperation_weight: 1.0,

            max_steer_force: 3.0,
            min_speed: 15.0,
            max_speed: 40.0,

            view_radius: 40.,
            avoid_raduis: 10.,

            boid_count: 20,
        }
    }

}

fn update_boids(
    mut boids: Query<(Entity, &Transform, &mut ExternalForce, &Velocity), With<Boid>>,
    boid_settings: Res<BoidSettings>,
    children: Query<&Parent>,
    other_boids_transform: Query<(&Transform, With<Boid>, Option<&Parent>)>,
    rapier_context: Res<RapierContext>,
    mut lines: ResMut<DebugLines>,
) {
    let square_view_radius = boid_settings.view_radius * boid_settings.view_radius;
    let square_avoid_radius = boid_settings.avoid_raduis * boid_settings.avoid_raduis;

    let boid_data = boids.iter().map(|(entity, boid_transform, _, _)| {
        let mut flock_heading = Vec3::ZERO;
        let mut flock_center = Vec3::ZERO;
        let mut seperation_heading = Vec3::ZERO;
        let mut flock_size = 0;
        for (e1, e2, b) in rapier_context.intersections_with(entity) {
            let parent_opt = children.get(e2);
            if !parent_opt.is_ok() {
                continue;
            }

            let (other_boid_transform, is_boid, _) = other_boids_transform.get(parent_opt.unwrap().0).unwrap();

            if !is_boid || boid_transform == other_boid_transform {
                continue;
            }

            lines.line_colored(boid_transform.translation, other_boid_transform.translation, 0., Color::GREEN);

            let offset = other_boid_transform.translation - boid_transform.translation;
            let square_dist = offset.length_squared();
            if square_dist < square_view_radius {
                flock_size += 1; 
                flock_heading += other_boid_transform.forward();
                flock_center += other_boid_transform.translation;
                if square_dist < square_avoid_radius {
                    seperation_heading -= offset / square_dist;
                }
            }
        }
        return (flock_heading, flock_center, seperation_heading, flock_size);
    }).collect::<Vec<_>>();

    for (boid_cur_data, boid_calc_data) in itertools::zip(boids.iter_mut(), boid_data) {
        let (entity, transform, mut external_force, velocity) = boid_cur_data;
        let (flock_heading, flock_center, seperation_heading, flock_size) = boid_calc_data;
        external_force.force = Vec3::ZERO;
        if flock_size == 0 {
            continue;
        }
        let avg_flock_center = flock_center / (flock_size as f32);
        let flock_center_offset = avg_flock_center - transform.translation;

        let alignment_force = steer_towards(velocity.linvel, flock_heading, &boid_settings) * boid_settings.alignment_weight;
        let collision_avoidence_force = steer_towards(velocity.linvel, flock_center_offset, &boid_settings) * boid_settings.cohesion_weight;
        let seperation_force = steer_towards(velocity.linvel, seperation_heading, &boid_settings) * boid_settings.seperation_weight;

        external_force.force += alignment_force;
        external_force.force += collision_avoidence_force;
        external_force.force += seperation_force;
    }

}

fn steer_towards(velocity: Vec3, vector: Vec3, boid_settings: &Res<BoidSettings>) -> Vec3 {
    if vector.length_squared() < 0.1 {
        return Vec3::ZERO;
    }
    let new_vector = vector.normalize() * boid_settings.max_speed - velocity;
    return new_vector.clamp_length_max(boid_settings.max_steer_force);
}

fn keep_inside_field(
    field: Res<Field>,
    mut boids: Query<&mut Transform, With<Boid>>) {
    let min: Vec3 = field.0.min().into();
    let max: Vec3 = field.0.max().into();
    for mut transform in boids.iter_mut() {
        let mut pos = transform.translation;
        pos = Vec3::select(min.cmplt(pos), pos, max);
        pos = Vec3::select(pos.cmple(max), pos, min);
        transform.translation = pos;
    }
    
}

fn boid_look_at_velocity(mut boids: Query<(&mut Transform, &Velocity), With<Boid>>) {
    for (mut transform, velocity) in boids.iter_mut() {
        if velocity.linvel.length_squared() > 0.001 {
            let pos = transform.translation;
            let up = transform.up();
            transform.look_at(
                pos + velocity.linvel.normalize(),
                up
            );

        }
    }

}

fn debug_draw_boid(
    mut lines: ResMut<DebugLines>,
    boids: Query<(&Transform, &Velocity, &ExternalForce), With<Boid>>
) {
    for (transform, velocity, external_force) in boids.iter() {
        let pos = transform.translation;
        lines.line_colored(
            pos,
            pos + velocity.linvel.normalize(),
            0.0,
            Color::GREEN
        );

        lines.line_colored(
            pos,
            pos + external_force.force.normalize(),
            0.0,
            Color::RED
        );
    }

}
// hides mouse
pub fn cursor_grab_system(
    mut windows: ResMut<Windows>,
    btn: Res<Input<MouseButton>>,
    key: Res<Input<KeyCode>>,
    mut camera_controller_query: Query<&mut FpsCameraController>,
) {
    let window = windows.get_primary_mut().unwrap();

    let mut camera_controller = camera_controller_query.get_single_mut().unwrap();

    if btn.just_pressed(MouseButton::Left) {
        window.set_cursor_lock_mode(true);
        window.set_cursor_visibility(false);
        camera_controller.enabled = true;
    }

    if key.just_pressed(KeyCode::Escape) {
        window.set_cursor_lock_mode(false);
        window.set_cursor_visibility(true);
        camera_controller.enabled = false;
    }
}


fn setup_graphics(mut commands: Commands) {

    // light
    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    });

    let controller = FpsCameraController {
        enabled: false,
        smoothing_weight : 0.6,
        translate_sensitivity: 0.1,
        mouse_rotate_sensitivity: Vec2::splat(0.001),
        ..Default::default()
    };
    commands
        .spawn_bundle(PerspectiveCameraBundle::default())
        .insert_bundle(FpsCameraBundle::new(
            controller,
            PerspectiveCameraBundle {
                transform: Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
                ..Default::default()
            },
            Vec3::new(0., 0., 250.0),
            Vec3::new(0., 0., 0.),
        ));
}

//fn draw_bounding_box_system(
    //mut lines: ResMut<DebugLines>,
    //query: Query<&bevy::render::primitives::Aabb>
//) {
    //for aabb in query.iter() {
        //draw_bounding_box(&mut lines, aabb);
    //}

//}
//
fn draw_field_system(
    mut lines: ResMut<DebugLines>,
    field: Res<Field>,
) {
    draw_bounding_box(&mut lines, &field.0);
}

pub fn to_bvec3(bitmask: u8) -> BVec3 {
    BVec3::new(
        (bitmask & 0b100) != 0,
        (bitmask & 0b010) != 0,
        (bitmask & 0b001) != 0,
    )
}

pub fn draw_bounding_box(lines: &mut ResMut<DebugLines>, aabb: &Aabb) {
    let min = aabb.min().into();
    let max = aabb.max().into();

    let connections = [
        (0b000, 0b100),
        (0b000, 0b010),
        (0b000, 0b001),

        (0b100, 0b110),
        (0b100, 0b101),

        (0b010, 0b110),
        (0b010, 0b011),

        (0b001, 0b101),
        (0b001, 0b011),

        (0b011, 0b111),
        (0b101, 0b111),
        (0b110, 0b111),
    ];

    for (from, to) in connections {
        lines.line_colored(
            Vec3::select(to_bvec3(from), min, max),
            Vec3::select(to_bvec3(to), min, max),
            0.0,
            Color::GREEN
        );
    }
}

const FORWARD: Vec3 = const_vec3!([0.0, 0.0, -1.0]);
const UP: Vec3 = Vec3::Y;
const LEFT: Vec3 = Vec3::X;


fn create_boid_bundle(
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> BoidBundle {

    let a1 = (2.0 * PI) * 0.0/3.0; 
    let a2 = (2.0 * PI) * 1.0/3.0; 
    let a3 = (2.0 * PI) * 2.0/3.0; 

    let vertices: [([f32; 3], [f32; 2]); 4] = [
        ((2.0 * FORWARD).into(), [1.0, 1.0]),
        ((a1.sin() * LEFT + a1.cos() * UP).into(), [1.0, 1.0]),
        ((a2.sin() * LEFT + a2.cos() * UP).into(), [1.0, 1.0]),
        ((a3.sin() * LEFT + a3.cos() * UP).into(), [1.0, 1.0]),
    ];

    let indices_temp = vec![
        0, 1, 2,
        0, 2, 3,
        0, 3, 1,
        1, 3, 2,
    ];
    let indices = mesh::Indices::U32(indices_temp.clone());

    let mut positions = Vec::new();
    //let mut normals = Vec::new();
    let mut uvs = Vec::new();
    for (position,uv) in vertices.iter() {
        positions.push(*position);
        //normals.push(*normal);
        uvs.push(*uv);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_indices(Some(indices));
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.clone());
    //mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.duplicate_vertices();
    mesh.compute_flat_normals();

    // add entities to the world
    let pbr_bundle =  PbrBundle {
        mesh: meshes.add(mesh),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..Default::default()
    };

    let mut chunks = vec![];
    for chunk in indices_temp.chunks_exact(3) {
        chunks.push([chunk[0], chunk[1],chunk[2]]);
    }

    let collider = Collider::convex_mesh(
        positions.into_iter().map(|v| v.into()).collect(),
        chunks.as_slice(),
    ).unwrap();

    let mass_properties = MassProperties {
        mass: 10.0,
        ..Default::default()
    };

    let external_force = ExternalForce {
        force: Vec3::new(0.0, 0.0, 0.0),
        ..Default::default()
    };

    return BoidBundle {
        name: Name::new("Boid"),
        pbr_bundle,
        collider,
        mass_properties,
        rigid_body: RigidBody::Dynamic,
        external_force,
        boid: Boid,
        velocity: Velocity::default(),
    };

}

#[derive(Component)]
struct Boid;

#[derive(Bundle)]
struct BoidBundle {
    name: Name,
    #[bundle]
    pbr_bundle: PbrBundle,
    collider: Collider,
    rigid_body: RigidBody,
    mass_properties: MassProperties,
    external_force: ExternalForce,
    boid: Boid,
    velocity: Velocity,
}

fn spawn_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    field: Res<Field>,
    boid_settings: Res<BoidSettings>,
) {
    let mut rng = thread_rng();
    let radius = f32::min(
        field.0.min().min_element(),
        field.0.max().min_element()
    ).abs();
    let distribution = Uniform::new(-radius, radius);
    let rotation_distribution = Uniform::new(-1., 1.);
    for _ in 0..boid_settings.boid_count {
        let boid_bundle = create_boid_bundle(&mut meshes, &mut materials);

        let linvel = Vec3::new(
            rng.sample(rotation_distribution),
            rng.sample(rotation_distribution),
            rng.sample(rotation_distribution),
        );
        let velocity = Velocity {
            linvel,
            ..Default::default()
        };

        let translation = Vec3::new(
            rng.sample(distribution),
            rng.sample(distribution),
            rng.sample(distribution),
        );


        let transform = Transform::from_translation(translation)
            .with_scale(Vec3::splat(0.5));


        commands.spawn_bundle(boid_bundle)
            .insert(velocity)
            .insert(transform)
            .with_children(|children| {
                children.spawn()
                    .insert(Collider::ball(boid_settings.view_radius))
                    .insert(Sensor(true));
            });
    }
}
